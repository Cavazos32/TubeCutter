"""El enlace manda ping al conectar y bajo demanda, no en un temporizador."""

from __future__ import annotations

import json
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from clients.andon import AndonClient
from clients.motion import MotionClient
from clients.plc import PlcClient
from clients.prefeeder import PreFeederClient
from core.tcp_link import ModuleTcpClient


class _PongServer(threading.Thread):
    def __init__(self, pong_limit: int | None = None) -> None:
        super().__init__(daemon=True)
        self._pong_limit = pong_limit
        self._pongs_sent = 0
        self.commands: list[str] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self._srv.settimeout(0.3)
        self.port = self._srv.getsockname()[1]

    def ping_count(self) -> int:
        with self._lock:
            return sum(1 for cmd in self.commands if cmd == "ping")

    def stop(self) -> None:
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass

    def run(self) -> None:
        conn: socket.socket | None = None
        buf = ""
        while not self._stop.is_set():
            try:
                if conn is None:
                    conn, _ = self._srv.accept()
                    conn.settimeout(0.3)
                    buf = ""
                    continue
                chunk = conn.recv(4096)
                if not chunk:
                    conn.close()
                    conn = None
                    continue
                buf += chunk.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    cmd = ""
                    try:
                        cmd = str(json.loads(line).get("command") or "")
                    except json.JSONDecodeError:
                        cmd = ""
                    with self._lock:
                        self.commands.append(cmd)
                    if (
                        self._pong_limit is None
                        or self._pongs_sent < self._pong_limit
                    ):
                        conn.sendall(b'{"type":"pong"}\n')
                        self._pongs_sent += 1
            except socket.timeout:
                continue
            except OSError:
                if conn is not None:
                    try:
                        conn.close()
                    except OSError:
                        pass
                    conn = None
                if self._stop.is_set():
                    return


class LinkPingTests(unittest.TestCase):
    def test_todos_los_clientes_usan_el_mismo_ping(self) -> None:
        for cls in (PlcClient, MotionClient, PreFeederClient, AndonClient):
            self.assertIs(cls._send_probe, ModuleTcpClient._send_probe)
            self.assertIs(cls.ping, ModuleTcpClient.ping)

    def test_ping_al_conectar_y_no_periodico(self) -> None:
        server = _PongServer()
        server.start()
        client = ModuleTcpClient("127.0.0.1", server.port)
        try:
            self.assertTrue(client.connect(timeout=2.0))
            self.assertEqual(server.ping_count(), 1)
            client.start_background()
            time.sleep(6.2)
            self.assertEqual(server.ping_count(), 1)
            self.assertTrue(client.ping())
            deadline = time.monotonic() + 1.0
            while server.ping_count() < 2 and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertEqual(server.ping_count(), 2)
            self.assertTrue(client.connected)
        finally:
            client.stop_background()
            server.stop()
            server.join(timeout=2.0)

    def test_ping_confirmed_espera_el_pong(self) -> None:
        server = _PongServer()
        server.start()
        client = ModuleTcpClient("127.0.0.1", server.port)
        try:
            self.assertTrue(client.connect(timeout=2.0))
            self.assertTrue(client.ping_confirmed(attempts=1, timeout_s=1.0))
        finally:
            client.stop_background()
            server.stop()
            server.join(timeout=2.0)

    def test_ping_confirmed_agota_intentos_sin_pong(self) -> None:
        server = _PongServer(pong_limit=1)
        server.start()
        client = ModuleTcpClient("127.0.0.1", server.port)
        try:
            self.assertTrue(client.connect(timeout=2.0))
            self.assertFalse(client.ping_confirmed(attempts=2, timeout_s=0.2))
        finally:
            client.stop_background()
            server.stop()
            server.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
