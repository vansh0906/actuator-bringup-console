// Minimal stand-in for GoogleTest, used only when GoogleTest cannot be
// downloaded (build with -DUSE_GTEST_LITE=ON). It supports the subset the
// tests use: TEST, EXPECT_/ASSERT_ TRUE, FALSE, EQ, NE, LT, LE, GT, GE, NEAR.
// The normal build (and CI) uses real GoogleTest via FetchContent.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace gtest_lite {

struct TestCase {
    const char* suite;
    const char* name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}
inline bool& current_failed() {
    static bool f = false;
    return f;
}
struct Registrar {
    Registrar(const char* s, const char* n, std::function<void()> fn) { registry().push_back({s, n, std::move(fn)}); }
};
struct AssertFail {};

template <typename A, typename B>
std::string show(const A& a, const B& b) {
    std::ostringstream os;
    os << "  actual: " << a << " vs " << b;
    return os.str();
}

inline void fail(const char* file, int line, const char* expr, const std::string& detail, bool fatal) {
    std::printf("%s:%d: Failure\n  %s\n%s\n", file, line, expr, detail.c_str());
    current_failed() = true;
    if (fatal) throw AssertFail{};
}

inline int run_all() {
    int failed = 0;
    std::printf("[==========] Running %zu tests.\n", registry().size());
    for (auto& t : registry()) {
        current_failed() = false;
        std::printf("[ RUN      ] %s.%s\n", t.suite, t.name);
        try {
            t.fn();
        } catch (const AssertFail&) {
        }
        if (current_failed()) {
            ++failed;
            std::printf("[  FAILED  ] %s.%s\n", t.suite, t.name);
        } else {
            std::printf("[       OK ] %s.%s\n", t.suite, t.name);
        }
    }
    std::printf("[==========] %zu tests ran.\n[  PASSED  ] %zu tests.\n", registry().size(), registry().size() - failed);
    if (failed) std::printf("[  FAILED  ] %d tests.\n", failed);
    return failed ? 1 : 0;
}

}  // namespace gtest_lite

#define TEST(suite, name)                                                                      \
    static void suite##_##name##_impl();                                                       \
    static gtest_lite::Registrar suite##_##name##_reg(#suite, #name, &suite##_##name##_impl); \
    static void suite##_##name##_impl()

#define GL_CHECK(cond, expr, detail, fatal) \
    do {                                    \
        if (!(cond)) gtest_lite::fail(__FILE__, __LINE__, expr, detail, fatal); \
    } while (0)

#define GL_CMP(a, b, op, fatal) GL_CHECK((a)op(b), #a " " #op " " #b, gtest_lite::show((a), (b)), fatal)

#define EXPECT_TRUE(c) GL_CHECK((c), #c, "", false)
#define EXPECT_FALSE(c) GL_CHECK(!(c), "!" #c, "", false)
#define ASSERT_TRUE(c) GL_CHECK((c), #c, "", true)
#define ASSERT_FALSE(c) GL_CHECK(!(c), "!" #c, "", true)
#define EXPECT_EQ(a, b) GL_CMP(a, b, ==, false)
#define EXPECT_NE(a, b) GL_CMP(a, b, !=, false)
#define EXPECT_LT(a, b) GL_CMP(a, b, <, false)
#define EXPECT_LE(a, b) GL_CMP(a, b, <=, false)
#define EXPECT_GT(a, b) GL_CMP(a, b, >, false)
#define EXPECT_GE(a, b) GL_CMP(a, b, >=, false)
#define ASSERT_EQ(a, b) GL_CMP(a, b, ==, true)
#define ASSERT_GT(a, b) GL_CMP(a, b, >, true)
#define ASSERT_GE(a, b) GL_CMP(a, b, >=, true)
#define EXPECT_NEAR(a, b, tol) \
    GL_CHECK(std::fabs((double)(a) - (double)(b)) <= (tol), #a " ~= " #b, gtest_lite::show((a), (b)), false)

#define GTEST_LITE_MAIN \
    int main() { return gtest_lite::run_all(); }
