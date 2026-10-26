"""Protocol and DBC tests: no simulator needed."""

import random

import pytest

from bringup.dbc import load_dbc
from bringup.protocol import (
    Codec,
    Frame,
    InjectKind,
    Mode,
    ParamOp,
    crc8_j1850,
    default_dbc_path,
    e2e_crc,
    fault_names,
    frame_bits,
)

# Frames packed by the C++ firmware (firmware/core/protocol.cpp). The
# Python codec must produce exactly the same bytes from the DBC.
GOLDEN = [
    ("command_n3", 0x203, "13C7CFFFFF004DF6"),
    ("command_n1_clear", 0x201, "2288130000000195"),
    ("estop_7", 0x080, "A507E2"),
    ("status_fast_n2", 0x182, "393000000CFEC409"),
    ("status_slow_n1", 0x281, "8E12380102012303"),
    ("param_req_n1", 0x601, "0205000000002040"),
    ("heartbeat_n4", 0x704, "0240E20100010001"),
]
G = {name: (can_id, bytes.fromhex(data)) for name, can_id, data in GOLDEN}


@pytest.fixture(scope="module")
def codec():
    return Codec()


def test_dbc_loads_every_message():
    db = load_dbc(default_dbc_path())
    names = {m.name for m in db.messages}
    assert names == {"ESTOP", "EMCY", "STATUS_FAST", "COMMAND", "STATUS_SLOW", "PARAM_RESP", "PARAM_REQ",
                     "HEARTBEAT", "FAULT_INJECT"}
    assert db.by_name("PARAM_REQ").signal("value").is_float
    assert db.by_name("STATUS_SLOW").signal("state").choices[3] == "FAULT"
    assert "CMD_TIMEOUT" in db.by_name("COMMAND").comment


def test_crc8_check_value():
    assert crc8_j1850(b"123456789") == 0x4B


def test_commands_match_firmware(codec):
    f = codec.command(3, Mode.POSITION, -1.2345, enable=True, clear_faults=False, counter=77)
    assert (f.can_id, f.data) == G["command_n3"]
    f = codec.command(1, Mode.VELOCITY, 0.5, enable=False, clear_faults=True, counter=1)
    assert (f.can_id, f.data) == G["command_n1_clear"]
    f = codec.estop(7)
    assert (f.can_id, f.data) == G["estop_7"]
    f = codec.param_request(1, ParamOp.WRITE, 5, 2.5)
    assert (f.can_id, f.data) == G["param_req_n1"]


def test_status_decodes_match_firmware(codec):
    d = codec.decode(Frame(*G["status_fast_n2"]))
    assert d.name == "STATUS_FAST" and d.node == 2
    assert d.signals["position"] == pytest.approx(1.2345, abs=1e-4)
    assert d.signals["velocity"] == pytest.approx(-0.5, abs=1e-3)
    assert d.signals["current"] == pytest.approx(2.5, abs=1e-3)
    d = codec.decode(Frame(*G["status_slow_n1"]))
    assert d.signals["bus_voltage"] == pytest.approx(47.5, abs=0.01)
    assert d.signals["motor_temp"] == pytest.approx(31.2, abs=0.05)
    assert fault_names(d.signals["fault_flags"]) == ["OVERTEMP", "ESTOP"]
    assert (d.signals["state"], d.signals["mode"], d.signals["e2e_errors"]) == (3, 2, 3)
    d = codec.decode(Frame(*G["heartbeat_n4"]))
    assert d.node == 4 and d.signals["uptime"] == pytest.approx(123.456) and d.signals["homed"] == 1


def test_e2e_crc_covers_id_and_payload(codec):
    f = codec.command(2, Mode.VELOCITY, 1.0, True, False, 9)
    assert f.data[7] == e2e_crc(0x202, f.data[:7])
    assert f.data[7] != e2e_crc(0x203, f.data[:7])


def test_node_mapping(codec):
    msg, node = codec.message_for(0x185)
    assert (msg.name, node) == ("STATUS_FAST", 5)
    assert codec.message_for(0x080)[0].name == "ESTOP"
    assert codec.message_for(0x7F0)[0].name == "FAULT_INJECT"
    assert codec.message_for(0x300) == (None, 0)
    f = codec.fault_inject(2, InjectKind.JAM, 40.0)
    d = codec.decode(f)
    assert (d.signals["node"], d.signals["kind"]) == (2, 4) and d.signals["value"] == pytest.approx(40.0)


def test_frame_bits_with_stuffing():
    # An 8-byte classic frame is 111 bits plus 0..24 stuff bits.
    rng = random.Random(1)
    for _ in range(200):
        data = bytes(rng.randrange(256) for _ in range(8))
        assert 111 <= frame_bits(rng.randrange(0x800), data) <= 135
    # Long runs of equal bits force stuffing.
    assert frame_bits(0x000, bytes(8)) > frame_bits(0x555, bytes([0x55] * 8))


def test_matches_cantools_when_installed(codec):
    cantools = pytest.importorskip("cantools")
    db = cantools.database.load_file(str(default_dbc_path()))
    rng = random.Random(7)
    for msg in codec.db.messages:
        ref = db.get_message_by_name(msg.name)
        for _ in range(20):
            values = {}
            for s in msg.signals:
                if s.is_float:
                    values[s.name] = rng.uniform(-100, 100)
                else:
                    lo = -(1 << (s.length - 1)) if s.signed else 0
                    hi = (1 << (s.length - 1)) - 1 if s.signed else (1 << s.length) - 1
                    values[s.name] = rng.randint(lo, hi) * s.factor + s.offset
            ours = msg.encode(values)
            theirs = ref.encode(values, scaling=True, strict=False)
            assert ours == theirs, msg.name
            decoded = ref.decode(ours, decode_choices=False)
            for k, v in msg.decode(ours).items():
                assert decoded[k] == pytest.approx(v, rel=1e-6, abs=1e-6), (msg.name, k)
