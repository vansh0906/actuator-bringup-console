"""SLCAN link: CAN frames over a TCP socket or a serial port.

    tcp://localhost:29536            host simulator (or any SLCAN-over-TCP bridge)
    /dev/cu.usbserial-0001           ESP32 / USB-CAN adapter over USB serial
    serial:///dev/ttyACM0?baud=921600

The link opens the channel (C, S6, O), then a reader thread parses
incoming "tIIILDD..\\r" lines and hands each frame to a callback.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable
from urllib.parse import parse_qs, urlparse

from .protocol import Frame

FrameCallback = Callable[[Frame], None]


class LinkError(RuntimeError):
    pass


class SlcanLink:
    def __init__(self, url: str, on_frame: FrameCallback | None = None, timeout: float = 3.0):
        self.url = url
        self.on_frame = on_frame
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._serial = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self.connected = False
        self.rx_frames = 0
        self.tx_frames = 0
        self.errors = 0  # BEL replies and malformed lines
        self.last_error = ""

    # --- open / close -------------------------------------------------------
    def open(self) -> "SlcanLink":
        parsed = urlparse(self.url)
        if parsed.scheme == "tcp":
            host, port = parsed.hostname or "localhost", parsed.port or 29536
            try:
                self._sock = socket.create_connection((host, port), timeout=self.timeout)
            except OSError as exc:
                raise LinkError(f"cannot connect to {host}:{port}: {exc}. Is the simulator running?") from exc
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock.settimeout(0.2)
        else:
            path = parsed.path if parsed.scheme == "serial" else self.url
            baud = int(parse_qs(parsed.query).get("baud", ["921600"])[0]) if parsed.scheme == "serial" else 921600
            try:
                import serial  # pyserial
            except ImportError as exc:
                raise LinkError("serial links need pyserial: pip install pyserial") from exc
            try:
                self._serial = serial.Serial(path, baudrate=baud, timeout=0.2)
            except Exception as exc:  # serial.SerialException
                raise LinkError(f"cannot open {path}: {exc}") from exc
            time.sleep(2.0)  # ESP32 boards reset when the port opens
            self._serial.reset_input_buffer()
        self._write(b"C\rS6\rO\r")
        self.connected = True
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, name="slcan-reader", daemon=True)
        self._reader.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self.connected:
            try:
                self._write(b"C\r")
            except Exception:
                pass
        self.connected = False
        if self._reader and self._reader is not threading.current_thread():
            self._reader.join(timeout=1.0)
        if self._sock:
            self._sock.close()
            self._sock = None
        if self._serial:
            self._serial.close()
            self._serial = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # --- tx -----------------------------------------------------------------
    def _write(self, data: bytes) -> None:
        with self._tx_lock:
            if self._sock:
                self._sock.sendall(data)
            elif self._serial:
                self._serial.write(data)
            else:
                raise LinkError("link is not open")

    def send(self, frame: Frame) -> None:
        if not self.connected:
            raise LinkError("link is not open")
        if not 0 <= frame.can_id <= 0x7FF or len(frame.data) > 8:
            raise ValueError(f"bad frame {frame}")
        line = f"t{frame.can_id:03X}{len(frame.data)}{frame.data.hex().upper()}\r".encode()
        try:
            self._write(line)
        except OSError as exc:
            self.connected = False
            self.last_error = str(exc)
            raise LinkError(f"send failed: {exc}") from exc
        self.tx_frames += 1

    # --- rx -----------------------------------------------------------------
    def _read_chunk(self) -> bytes:
        if self._sock:
            try:
                data = self._sock.recv(65536)
            except socket.timeout:
                return b""
            if not data:
                raise LinkError("connection closed by the other end")
            return data
        if self._serial:
            n = max(1, self._serial.in_waiting)
            return self._serial.read(n)
        raise LinkError("link is not open")

    def _read_loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._read_chunk()
            except (LinkError, OSError) as exc:
                self.last_error = str(exc)
                self.connected = False
                return
            if not chunk:
                continue
            now = time.time()
            if b"\x07" in chunk:
                self.errors += chunk.count(b"\x07")
                chunk = chunk.replace(b"\x07", b"")
            buf += chunk
            *lines, buf = buf.split(b"\r")
            for line in lines:
                self._handle_line(line, now)

    def _handle_line(self, line: bytes, now: float) -> None:
        if not line or line[:1] in (b"z", b"Z", b"V", b"v", b"N", b"F"):
            return  # OK / ack / info replies
        if line[:1] != b"t":
            self.errors += 1
            return
        frame = parse_slcan_frame(line, now)
        if frame is None:
            self.errors += 1
            return
        self.rx_frames += 1
        if self.on_frame:
            try:
                self.on_frame(frame)
            except Exception as exc:  # never let one bad frame stop the reader
                self.errors += 1
                self.last_error = f"frame handler: {exc!r}"



def parse_slcan_frame(line: bytes, timestamp: float | None = None) -> Frame | None:
    try:
        text = line.decode("ascii")
        if len(text) < 5 or text[0] != "t":
            return None
        can_id = int(text[1:4], 16)
        dlc = int(text[4])
        if dlc > 8 or len(text) != 5 + 2 * dlc:
            return None
        data = bytes.fromhex(text[5:5 + 2 * dlc])
    except (ValueError, UnicodeDecodeError):
        return None
    return Frame(can_id, data, timestamp if timestamp is not None else time.time())
