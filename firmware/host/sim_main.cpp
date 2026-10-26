// Host build of the actuator firmware: runs N simulated actuators on one
// virtual CAN bus in real time and serves that bus as SLCAN over TCP, so
// the console (and the ROS 2 driver in Docker) connect to it exactly as
// they would to a USB-CAN adapter.
//
//   ./build/actuator_sim --nodes 2 --port 29536
//
// Every connected client sees every frame on the bus, like nodes on a real
// CAN bus (a client does not get its own frames echoed back).

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "node.hpp"
#include "slcan.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;
void on_signal(int) { g_running = 0; }

class FileStorage : public act::Storage {
public:
    explicit FileStorage(std::string path) : path_(std::move(path)) {}
    bool save(const uint8_t* data, size_t n) override {
        std::ofstream f(path_, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(n));
        return static_cast<bool>(f);
    }
    size_t load(uint8_t* data, size_t max_n) override {
        std::ifstream f(path_, std::ios::binary);
        if (!f) return 0;
        f.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(max_n));
        return static_cast<size_t>(f.gcount());
    }

private:
    std::string path_;
};

struct Client {
    int fd = -1;
    act::SlcanPort port;
    std::string out;
    std::string name;
};

const char* fault_names(uint16_t bits, char* buf, size_t n) {
    static const char* kNames[] = {"OVERCURRENT", "OVERTEMP", "UNDERVOLTAGE", "CMD_TIMEOUT", "FOLLOWING_ERROR",
                                   "OVERSPEED",   "ENCODER",  "STALL",        "ESTOP"};
    buf[0] = '\0';
    for (int i = 0; i < 9; ++i) {
        if (bits & (1u << i)) {
            if (buf[0]) std::strncat(buf, "|", n - std::strlen(buf) - 1);
            std::strncat(buf, kNames[i], n - std::strlen(buf) - 1);
        }
    }
    return buf;
}

act::PlantConfig plant_for(int node) {
    // Each joint hides something for the bring-up tools to find.
    act::PlantConfig c;
    c.seed = static_cast<uint32_t>(1000 + node);
    switch (node) {
        case 1:
            c.current_sensor_offset = 0.12;
            c.initial_position = 0.7;
            break;
        case 2:
            c.current_sensor_offset = -0.08;
            c.initial_position = -0.4;
            c.wiring_reversed = true;  // motor phases swapped on this joint
            break;
        default:
            c.current_sensor_offset = 0.05 * ((node % 3) - 1);
            c.initial_position = 0.1 * node;
            break;
    }
    return c;
}

void usage() {
    std::printf(
        "usage: actuator_sim [--nodes N] [--port P] [--bind ADDR] [--state-dir DIR] [--duration S] [--quiet]\n"
        "  --nodes N      simulated actuators, node IDs 1..N (default 2)\n"
        "  --port P       TCP port for SLCAN clients (default 29536)\n"
        "  --bind ADDR    address to listen on (default 127.0.0.1; 0.0.0.0 inside Docker)\n"
        "  --state-dir D  where saved parameters live (default .)\n"
        "  --duration S   exit after S seconds (default: run until Ctrl+C)\n");
}

}  // namespace

int main(int argc, char** argv) {
    int nodes = 2;
    int port = 29536;
    std::string bind_addr = "127.0.0.1";
    std::string state_dir = ".";
    double duration = 0.0;
    bool quiet = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--nodes") nodes = std::atoi(next("--nodes"));
        else if (a == "--port") port = std::atoi(next("--port"));
        else if (a == "--bind") bind_addr = next("--bind");
        else if (a == "--state-dir") state_dir = next("--state-dir");
        else if (a == "--duration") duration = std::atof(next("--duration"));
        else if (a == "--quiet") quiet = true;
        else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            usage();
            return 2;
        }
    }
    if (nodes < 1 || nodes > 16) {
        std::fprintf(stderr, "--nodes must be 1..16\n");
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    std::vector<std::unique_ptr<FileStorage>> storages;
    std::vector<std::unique_ptr<act::ActuatorNode>> bus_nodes;
    for (int n = 1; n <= nodes; ++n) {
        storages.push_back(std::make_unique<FileStorage>(state_dir + "/node" + std::to_string(n) + "_params.bin"));
        bus_nodes.push_back(
            std::make_unique<act::ActuatorNode>(static_cast<uint8_t>(n), plant_for(n), storages.back().get()));
    }

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        std::perror("socket");
        return 1;
    }
    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, bind_addr.c_str(), &addr.sin_addr) != 1) {
        std::fprintf(stderr, "bad --bind address: %s\n", bind_addr.c_str());
        return 2;
    }
    if (bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind (is another simulator already running on this port?)");
        return 1;
    }
    listen(listen_fd, 8);
    fcntl(listen_fd, F_SETFL, O_NONBLOCK);

    std::printf("actuator_sim: %d node(s) on SLCAN tcp://%s:%d  (Ctrl+C to stop)\n", nodes, bind_addr.c_str(), port);
    std::fflush(stdout);

    std::vector<std::unique_ptr<Client>> clients;
    const auto t0 = std::chrono::steady_clock::now();
    uint64_t ticks = 0;
    char fmt[act::SlcanPort::kMaxFormatted];
    char names[160];

    auto broadcast = [&](const act::CanFrame& f, const Client* from) {
        const size_t n = act::SlcanPort::format(f, fmt);
        for (auto& c : clients) {
            if (c.get() != from && c->port.is_open()) c->out.append(fmt, n);
        }
    };

    while (g_running) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (duration > 0 && elapsed >= duration) break;

        // --- Run the bus in 1 ms steps up to wall-clock time ------------------
        uint64_t target = static_cast<uint64_t>(elapsed * 1000.0);
        if (target > ticks + 50) ticks = target - 50;  // fell far behind (e.g. laptop slept): skip ahead
        while (ticks < target) {
            for (auto& node : bus_nodes) node->tick_1ms();
            for (auto& node : bus_nodes) {
                act::CanFrame f;
                while (node->pop_tx(f)) {
                    for (auto& other : bus_nodes) {
                        if (other.get() != node.get()) other->on_frame(f);
                    }
                    broadcast(f, nullptr);
                    if (!quiet && act::proto::function_code(f.id) == act::proto::FC_EMCY && f.id != act::proto::ID_ESTOP) {
                        act::proto::Emcy e;
                        act::proto::unpack_emcy(f, e);
                        std::printf("[%9.3f s] node %d fault: %s\n", static_cast<double>(ticks) / 1000.0, node->node_id(),
                                    fault_names(e.fault_code, names, sizeof(names)));
                    }
                }
            }
            ++ticks;
        }

        // --- Socket I/O ---------------------------------------------------------
        std::vector<pollfd> fds;
        fds.push_back({listen_fd, POLLIN, 0});
        for (auto& c : clients) {
            short ev = POLLIN;
            if (!c->out.empty()) ev |= POLLOUT;
            fds.push_back({c->fd, ev, 0});
        }
        poll(fds.data(), fds.size(), 1);

        if (fds[0].revents & POLLIN) {
            sockaddr_in caddr{};
            socklen_t clen = sizeof(caddr);
            int fd = accept(listen_fd, reinterpret_cast<sockaddr*>(&caddr), &clen);
            if (fd >= 0) {
                fcntl(fd, F_SETFL, O_NONBLOCK);
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
                auto c = std::make_unique<Client>();
                c->fd = fd;
                char ip[INET_ADDRSTRLEN] = "?";
                inet_ntop(AF_INET, &caddr.sin_addr, ip, sizeof(ip));
                c->name = std::string(ip) + ":" + std::to_string(ntohs(caddr.sin_port));
                if (!quiet) std::printf("[%9.3f s] client connected: %s\n", static_cast<double>(ticks) / 1000.0, c->name.c_str());
                clients.push_back(std::move(c));
            }
        }

        for (size_t i = 0; i < clients.size(); ++i) {
            Client& c = *clients[i];
            const short rev = fds.size() > i + 1 ? fds[i + 1].revents : 0;
            bool drop = false;
            if (rev & (POLLIN | POLLHUP | POLLERR)) {
                char buf[4096];
                ssize_t n = recv(c.fd, buf, sizeof(buf), 0);
                if (n <= 0) {
                    drop = true;
                } else {
                    for (ssize_t k = 0; k < n; ++k) {
                        act::SlcanPort::Result r;
                        if (c.port.feed(buf[k], r)) {
                            c.out.append(r.reply, r.reply_len);
                            if (r.has_frame) {
                                for (auto& node : bus_nodes) node->on_frame(r.frame);
                                broadcast(r.frame, &c);
                            }
                        }
                    }
                }
            }
            if (!drop && !c.out.empty()) {
                ssize_t n = send(c.fd, c.out.data(), c.out.size(), 0);
                if (n > 0) c.out.erase(0, static_cast<size_t>(n));
                if (c.out.size() > (1u << 20)) drop = true;  // client stopped reading
            }
            if (drop) {
                if (!quiet) std::printf("[%9.3f s] client disconnected: %s\n", static_cast<double>(ticks) / 1000.0, c.name.c_str());
                close(c.fd);
                c.fd = -1;
            }
        }
        for (size_t i = 0; i < clients.size();) {
            if (clients[i]->fd < 0) clients.erase(clients.begin() + static_cast<long>(i));
            else ++i;
        }
        std::fflush(stdout);
    }

    for (auto& c : clients) close(c->fd);
    close(listen_fd);
    std::printf("actuator_sim: stopped after %.1f s\n", static_cast<double>(ticks) / 1000.0);
    return 0;
}
