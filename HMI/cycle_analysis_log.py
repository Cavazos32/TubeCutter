"""Registro .md por lote para análisis de tiempos (HMI/logs/cycle_runs/).

Desactivar: variable de entorno HMI_CYCLE_ANALYSIS=0
"""

from __future__ import annotations

import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

HMI_ROOT = Path(__file__).resolve().parent
RUNS_DIR = HMI_ROOT / "logs" / "cycle_runs"

# Columnas fijas (segundos acumulados por pieza).
COLUMNS = ("pf", "feed", "lineal", "cut", "dep", "clr", "home", "ack", "otr")

_LOCK = threading.Lock()
_ACTIVE = False
_PATH: Path | None = None
_PIECE_BUCKETS: dict[str, float] = {}
_LOT_ROWS: list[dict[str, Any]] = []
_EVENT_COUNT = 0


def _enabled() -> bool:
    return os.environ.get("HMI_CYCLE_ANALYSIS", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _file_slug(meta: dict[str, Any]) -> str:
    ts = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    l_mm = int(round(float(meta.get("length_mm") or 0)))
    qty = int(meta.get("cycle_qty") or 0)
    return f"{ts}_L{l_mm}_n{qty}"


def _bucket(op_name: str) -> str:
    n = str(op_name).strip().lower()
    if n.startswith("wait_ack:"):
        return "ack"
    if "depósito" in n or "deposito" in n:
        return "dep"
    if "despeje" in n:
        return "clr"
    if n.startswith("pf") or n.startswith("pf_"):
        return "pf"
    if n.startswith("feed"):
        return "feed"
    if n.startswith("lineal"):
        return "lineal"
    if n in ("corte", "cutter") or n.startswith("cutter"):
        return "cut"
    if n == "home" or n.startswith("home"):
        return "home"
    return "otr"


def _fmt_cell(sec: float) -> str:
    s = max(0.0, float(sec))
    if s < 0.005:
        return "—"
    if s < 10.0:
        return f"{s:.2f}"
    return f"{s:.1f}"


def _append_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(text)


def _esc_cell(text: str) -> str:
    return str(text).replace("|", "/").replace("\n", " ").strip()


def is_lot_active() -> bool:
    with _LOCK:
        return bool(_ACTIVE and _PATH is not None)


def classify_cycle_log_message(text: str) -> tuple[str, str] | None:
    """Devuelve (tipo, detalle) si el mensaje debe ir al registro de eventos."""
    t = text.strip()
    if not t or t.startswith("Análisis ciclo →"):
        return None
    low = t.lower()
    noise = (
        "wip delivery",
        "plc ack ·",
        "feed start lados",
        "feed ok lados",
        "lineal move tcp ok",
        "lineal move tcp: comando",
        "home/wip total ·",
        "pinzas on (",
        "pinzas off (",
        "holder+encoder on (",
        "cortador on (set)",
        "cortador off (res)",
        "delay ·",
        "enc_set0: omitido",
        "prefeeder: in process on",
        "cycle flow:",
        " · move=",
        "trigger prefee",
        "r(0x4c)=",
        "cycle busy on",
        "cycle busy off",
        "cycle config guardada",
    )
    if any(sub in low for sub in noise):
        return None

    if t.startswith("Cycle Start"):
        return ("inicio", t)
    if t.startswith("Cycle Stop"):
        return ("stop", t)
    if t.startswith("Cycle Pause"):
        return ("pausa", t)
    if t.startswith("Cycle Resume"):
        return ("resume", t)
    if t.startswith("Cycle Reset"):
        return ("reset", t)
    if t.startswith("Cycle Materialista") or (
        t.startswith("E050:") and "materialista" in low
    ):
        return ("materialista", t)
    if t.startswith("E050:"):
        return ("e050", t)
    if t.startswith("Recovery:"):
        return ("recovery", t)
    if "purga" in low or t.startswith("Purge"):
        return ("purga", t)
    if t.startswith("Refill") or low.startswith("refill"):
        return ("refill", t)
    if t.startswith("Paso a paso"):
        return ("paso", t)
    if t.startswith("Tray"):
        return ("tray", t)
    if ("Pieza " in t or t.startswith("Ciclo ")) and " OK" in t:
        return ("pieza", t)
    if t.startswith("Lote completado") or t.startswith("Refill completo"):
        return ("fin", t)
    if t.startswith("Refill cancelado"):
        return ("refill", t)
    if "exception" in low or t.startswith("Pieza sin avance") or "abortado" in low:
        return ("error", t)
    if t.startswith("PreFeeder:") and any(
        x in t for x in ("falló", "timeout", "Error", "EXXX", "Start 0x2A")
    ):
        return ("prefeeder", t)
    if "Start rechazado" in t:
        return ("inicio", t)
    if t.startswith("CMD_MOVE:") or t.startswith("Cycle exception"):
        return ("error", t)
    if t.startswith("E0") and " · " in t:
        return ("error", t)
    if t.startswith("Cycle:"):
        return ("error", t)
    if "omitido" in low and ("feed" in low or "trigger" in low):
        return ("info", t)
    if "falló" in low and ("envío" in low or "tcp" in low):
        return ("error", t)
    return None


def record_event(kind: str, detail: str, *, piece: str = "") -> None:
    global _EVENT_COUNT
    if not _enabled():
        return
    with _LOCK:
        if not _ACTIVE or _PATH is None:
            return
        path = _PATH
        if not piece:
            rep = _PIECE_BUCKETS.get("rep")
            qty = _PIECE_BUCKETS.get("qty")
            if rep and qty:
                piece = f"{int(rep)}/{int(qty)}"
            else:
                piece = "—"
        _EVENT_COUNT += 1
    line = (
        f"| {_utc_stamp()} | {_esc_cell(kind)} | {_esc_cell(piece)} | "
        f"{_esc_cell(detail)} |\n"
    )
    _append_text(path, line)


def begin_lot(meta: dict[str, Any]) -> Path | None:
    """Abre un .md nuevo al arrancar el hilo de lote."""
    global _ACTIVE, _PATH, _PIECE_BUCKETS, _LOT_ROWS, _EVENT_COUNT
    if not _enabled():
        return None
    with _LOCK:
        if _PATH is not None and _ACTIVE:
            return _PATH
        _PIECE_BUCKETS = {}
        _LOT_ROWS = []
        _EVENT_COUNT = 0
        slug = _file_slug(meta)
        mode = str(meta.get("mode") or "lote")
        path = RUNS_DIR / f"{slug}_{mode}.md"
        _PATH = path
        _ACTIVE = True
        sides = str(meta.get("feed_sides") or "—")
        order = meta.get("order_qty")
        cycles = meta.get("cycle_qty")
        ppr = int(meta.get("pieces_per_rep") or 1)
        rpm = meta.get("rpm")
        length = meta.get("length_mm")
        title = "Refill material" if mode == "refill" else "Lote"
        lines = [
            f"# {title} {_utc_stamp()}\n",
            "\n",
            "| L (mm) | pedido (pz) | ciclos | rpm | lados | pz/ciclo |\n",
            "|-------:|------------:|-------:|----:|:-----:|---------:|\n",
            f"| {float(length):g} | {int(order)} | {int(cycles)} | {float(rpm):g} "
            f"| {sides} | {ppr} |\n",
            "\n",
            "## Eventos\n",
            "\n",
            "| UTC | tipo | pza | detalle |\n",
            "|-----|:-----|:---:|:--------|\n",
            "\n",
            "## Tiempos por pieza\n",
            "\n",
            "| # | CT | "
            + " | ".join(COLUMNS)
            + " |\n",
            "|--:|---:|"
            + "|".join("---:" for _ in COLUMNS)
            + "|\n",
        ]
        _append_text(path, "".join(lines))
        note = str(meta.get("note") or "").strip()
    if note:
        record_event("inicio", note)
    return path


def begin_piece(rep: int, qty: int) -> None:
    global _PIECE_BUCKETS
    if not _enabled():
        return
    with _LOCK:
        if not _ACTIVE:
            return
        _PIECE_BUCKETS = {"rep": int(rep), "qty": int(qty)}


def record_op(op_name: str, sec: float, *, ok: bool = True) -> None:
    if not _enabled() or not ok:
        return
    s = max(0.0, float(sec))
    if s <= 0.0:
        return
    key = _bucket(op_name)
    with _LOCK:
        if not _ACTIVE:
            return
        _PIECE_BUCKETS[key] = float(_PIECE_BUCKETS.get(key, 0.0)) + s


def end_piece(rep: int, qty: int, piece_sec: float) -> None:
    global _PIECE_BUCKETS, _LOT_ROWS
    if not _enabled():
        return
    with _LOCK:
        if not _ACTIVE or _PATH is None:
            return
        buckets = {k: float(_PIECE_BUCKETS.get(k, 0.0)) for k in COLUMNS}
        row = {
            "rep": int(rep),
            "qty": int(qty),
            "ct": max(0.0, float(piece_sec)),
            **buckets,
        }
        _LOT_ROWS.append(row)
        _PIECE_BUCKETS = {"rep": int(rep), "qty": int(qty)}
        cells = [_fmt_cell(row["ct"])] + [_fmt_cell(row[c]) for c in COLUMNS]
        line = f"| {int(rep)} | " + " | ".join(cells) + " |\n"
        _append_text(_PATH, line)


def end_lot(
    *,
    ok: bool,
    ct_sec: float,
    pieces_done: int,
    cycle_qty: int,
    fault: str = "",
) -> None:
    global _ACTIVE, _PATH, _PIECE_BUCKETS, _LOT_ROWS, _EVENT_COUNT
    if not _enabled():
        return
    with _LOCK:
        if _PATH is None:
            return
        rows = list(_LOT_ROWS)
        path = _PATH
        events = int(_EVENT_COUNT)
        _ACTIVE = False
        _PATH = None
        _PIECE_BUCKETS = {}
        _LOT_ROWS = []
        _EVENT_COUNT = 0
    status = "OK" if ok else f"falló ({fault or 'error'})"
    cts = [float(r["ct"]) for r in rows if float(r["ct"]) > 0]
    avg = sum(cts) / len(cts) if cts else 0.0
    mn = min(cts) if cts else 0.0
    mx = max(cts) if cts else 0.0
    lines = [
        "\n",
        "## Resumen\n",
        f"- Estado: **{status}**\n",
        f"- Eventos registrados: **{events + 1}**\n",
        f"- Piezas en tabla: **{pieces_done}/{cycle_qty}** ciclos\n",
        f"- CT lote: **{max(0.0, float(ct_sec)):.1f} s**\n",
    ]
    if cts:
        lines.append(
            f"- CT/pieza: prom **{avg:.2f} s**, min **{mn:.2f} s**, max **{mx:.2f} s**\n"
        )
    elif not rows:
        lines.append("- _Sin filas de tiempos (solo eventos)._ \n")
    fin = (
        f"| {_utc_stamp()} | fin | — | "
        f"{_esc_cell(f'Cierre — {status}')} |\n"
    )
    _append_text(path, fin + "".join(lines))
