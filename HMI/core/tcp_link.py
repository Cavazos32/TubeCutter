"""Enlace TCP compartido hacia módulos ESP32 — verificación al conectar y reconexión.

No hay ping periódico. El lote comprueba el pong una vez, en paralelo, antes de
la primera actuación. Durante las piezas el silencio no corta el enlace: la
caída se ve por cierre del socket o porque la orden real se queda sin ACK.
"""

from __future__ import annotations

import json
import socket
import sys
import threading
import time
from typing import Any, Callable, Optional

CONNECT_TIMEOUT = 2.0
RECONNECT_SEC = 3.0
VERIFY_TIMEOUT_SEC = 4.0
RX_TIMEOUT_SEC = 0.5
RX_JOIN_TIMEOUT_SEC = 1.0
# Línea incompleta. Si el MCU no manda newline, el buffer no crece sin límite.
RX_BUF_MAX = 65536


def take_rx_lines(buf: str, overflow: bool, limit: int = RX_BUF_MAX) -> tuple[str, bool, list[str]]:
    """Parte frames por newline.

    Un frame mayor que ``limit`` se descarta entero. El resto del buffer
    sigue válido. No se interpreta una línea cortada.
    """
    lines: list[str] = []
    while True:
        nl = buf.find("\n")
        if nl < 0:
            if overflow or len(buf) > limit:
                return "", True, lines
            return buf, overflow, lines
        raw = buf[:nl]
        buf = buf[nl + 1 :]
        if raw.endswith("\r"):
            raw = raw[:-1]
        if overflow:
            overflow = False
            continue
        if len(raw) > limit:
            continue
        text = raw.strip()
        if text:
            lines.append(text)


def enable_tcp_keepalive(sock: socket.socket) -> None:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    if sys.platform == "win32":
        try:
            sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 3000, 1000))
        except (AttributeError, OSError):
            pass
    elif sys.platform == "linux":
        for opt, val in (
            (socket.TCP_KEEPIDLE, 3),
            (socket.TCP_KEEPINTVL, 1),
            (socket.TCP_KEEPCNT, 3),
        ):
            try:
                sock.setsockopt(socket.IPPROTO_TCP, opt, val)
            except OSError:
                pass


class ModuleTcpClient:
    """Cliente TCP con verificación al conectar y reconexión si el socket cae."""

    def __init__(
        self,
        host: str,
        port: int,
        on_message: Optional[Callable[[dict], None]] = None,
        on_connection: Optional[Callable[[bool], None]] = None,
    ) -> None:
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._bg_thread: Optional[threading.Thread] = None
        self._stop_rx = threading.Event()
        self._stop_bg = threading.Event()
        self._io_lock = threading.Lock()
        self._conn_lock = threading.Lock()
        self._on_message = on_message
        self._on_connection = on_connection
        self._connected = False
        self._session = 0
        self._last_rx_mono = 0.0
        self._pong_mono = 0.0
        self._pong_cv = threading.Condition()
        self._connect_gate = threading.Lock()
        self._handshake = False
        # Evita que los 4 módulos reconecten en el mismo instante (pico WiFi/CPU).
        self._reconnect_jitter = (hash(f"{host}:{port}") & 0xFF) / 256.0 * RECONNECT_SEC

    @property
    def connected(self) -> bool:
        return self._connected

    def ping(self) -> bool:
        """Ack de enlace. No consulta sensores ni cambia el estado del módulo."""
        return self.send_command(command="ping")

    def wait_pong_since(self, since: float, timeout_s: float) -> bool:
        """True si llegó un pong con marca posterior a ``since``."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._pong_cv:
            while self._pong_mono < since:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._pong_cv.wait(timeout=remaining)
            return True

    def ping_confirmed(self, *, attempts: int = 3, timeout_s: float = 1.0) -> bool:
        """Reintenta el ping, que no actúa. No reenvía órdenes de válvula ni de movimiento."""
        tries = max(1, int(attempts))
        for _ in range(tries):
            if not self.connected:
                self.reconnect(silent=True)
            since = time.monotonic()
            if not self.ping():
                continue
            if self.wait_pong_since(since, timeout_s):
                return True
        return False

    def _send_probe(self) -> bool:
        return self.ping()

    def start_background(self) -> None:
        if self._bg_thread and self._bg_thread.is_alive():
            return
        self._stop_bg.clear()
        self._bg_thread = threading.Thread(target=self._bg_loop, daemon=True)
        self._bg_thread.start()

    def stop_background(self) -> None:
        self._stop_bg.set()
        self.disconnect(silent=True)

    def reconnect(self, silent: bool = True) -> bool:
        # No notificar caída aquí: un reconnect intencional no debe latchear E06x.
        was = self._drop_link(notify=False)
        ok = self.connect(timeout=CONNECT_TIMEOUT, silent=silent)
        if was and not ok:
            self._schedule_connection(False)
        return ok

    def connect(self, timeout: float = CONNECT_TIMEOUT, silent: bool = False) -> bool:
        # Serializar: bg reconnect + /api/network/reconnect no se pisan.
        with self._connect_gate:
            return self._connect_locked(timeout=timeout, silent=silent)

    def _connect_locked(self, timeout: float, silent: bool) -> bool:
        with self._conn_lock:
            if self._connected and self._sock is not None:
                return True
        self._drop_link(notify=False)

        session = 0
        sock: Optional[socket.socket] = None
        session_start = 0.0
        try:
            sock = socket.create_connection((self._host, self._port), timeout=timeout)
            sock.settimeout(RX_TIMEOUT_SEC)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            enable_tcp_keepalive(sock)
        except OSError as exc:
            self._connected = False
            if not silent:
                self._emit_error(f"Conexion fallida: {exc}")
            return False

        with self._conn_lock:
            with self._io_lock:
                self._session += 1
                session = self._session
                self._sock = sock
                self._last_rx_mono = 0.0
                self._stop_rx.clear()
                session_start = time.monotonic()
                self._rx_thread = threading.Thread(
                    target=self._rx_loop, args=(session, sock), daemon=True
                )
                self._rx_thread.start()

        self._handshake = True
        try:
            self._send_probe()
            if not self._wait_verified(session_start):
                if not silent:
                    self._emit_error(
                        f"Sin respuesta de {self._host}:{self._port} tras conectar"
                    )
                self._drop_link(notify=False)
                return False
        finally:
            self._handshake = False

        with self._conn_lock:
            if session != self._session or self._sock is not sock:
                return False
            self._connected = True

        self._schedule_connection(True)
        return True

    def disconnect(self, silent: bool = False) -> None:
        was = self._drop_link(notify=False)
        if was and not silent:
            self._schedule_connection(False)

    def _drop_link(self, *, notify: bool) -> bool:
        """
        Cierra socket y RX sin callbacks en este hilo.
        Nunca espera al RX mientras otro callback pueda querer _conn_lock.
        """
        sock: Optional[socket.socket] = None
        rx: Optional[threading.Thread] = None
        was = False
        with self._conn_lock:
            was = self._connected
            self._connected = False
            self._stop_rx.set()
            self._session += 1
            with self._io_lock:
                sock = self._sock
                self._sock = None
            rx = self._rx_thread
            self._rx_thread = None

        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

        # Join fuera de cualquier lock — el RX puede estar en callback diferido.
        if rx and rx is not threading.current_thread() and rx.is_alive():
            rx.join(timeout=RX_JOIN_TIMEOUT_SEC)

        if notify and was:
            self._schedule_connection(False)
        return was

    def _schedule_connection(self, connected: bool) -> None:
        """Callbacks diferidos; ignora eventos obsoletos tras reconnect."""
        if not self._on_connection:
            return
        gen = self._session
        want = connected

        def run() -> None:
            try:
                if want:
                    if not self._connected or self._session != gen:
                        return
                elif self._connected:
                    return
                self._on_connection(want)
            except Exception:
                pass

        threading.Thread(target=run, daemon=True).start()

    def send_command(self, **fields: Any) -> bool:
        payload = {"type": "command", **fields}
        return self._send_json(payload)

    def _wait_verified(self, since: float) -> bool:
        deadline = time.monotonic() + VERIFY_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._last_rx_mono >= since - 0.001:
                return True
            time.sleep(0.05)
        return False

    def _bg_loop(self) -> None:
        while not self._stop_bg.is_set():
            if not self._connected:
                self._stop_bg.wait(self._reconnect_jitter)
                self.connect(silent=True)
                self._stop_bg.wait(RECONNECT_SEC)
                continue

            self._stop_bg.wait(0.5)

    def _send_json(self, payload: dict) -> bool:
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        data = line.encode("utf-8")
        with self._io_lock:
            # Durante handshake aún no hay _connected; sí hay socket y RX activo.
            if not self._sock:
                return False
            if not self._connected and not self._handshake:
                return False
            try:
                self._sock.sendall(data)
                return True
            except OSError:
                # Invalidar socket ya; disconnect async notifica y limpia RX.
                self._sock = None
        threading.Thread(
            target=lambda: self.disconnect(silent=False), daemon=True
        ).start()
        return False

    def _rx_loop(self, session: int, sock: socket.socket) -> None:
        buf = ""
        overflow = False
        while not self._stop_rx.is_set() and session == self._session:
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                if session != self._session:
                    break
                self._last_rx_mono = time.monotonic()
                buf += chunk.decode("utf-8", errors="replace")
                was_overflow = overflow
                buf, overflow, lines = take_rx_lines(buf, overflow)
                if overflow and not was_overflow:
                    self._emit_error("RX descartado: linea sin fin")
                for line in lines:
                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        self._emit_error(f"JSON invalido: {line[:120]}")
                        continue
                    if isinstance(msg, dict) and msg.get("type") == "pong":
                        with self._pong_cv:
                            self._pong_mono = time.monotonic()
                            self._pong_cv.notify_all()
                    if self._on_message:
                        try:
                            self._on_message(msg)
                        except Exception:
                            pass
            except socket.timeout:
                continue
            except OSError:
                break

        if session != self._session:
            return

        was = False
        with self._conn_lock:
            with self._io_lock:
                if self._sock is sock:
                    self._sock = None
            was = self._connected
            self._connected = False
        try:
            sock.close()
        except OSError:
            pass

        if was:
            self._schedule_connection(False)

    def _emit_error(self, text: str) -> None:
        if self._on_message:
            try:
                self._on_message({"type": "local_error", "message": text})
            except Exception:
                pass
