"""Historial append-only de checklist y reset de mantenimiento (JSONL en logs/)."""

from __future__ import annotations

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

HMI_ROOT = Path(__file__).resolve().parent
CHECKLIST_HISTORY_PATH = HMI_ROOT / "logs" / "checklist_history.jsonl"
_HISTORY_LOCK = threading.Lock()


def append_checklist_history(record: dict[str, Any]) -> None:
    line = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
    with _HISTORY_LOCK:
        CHECKLIST_HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
        with CHECKLIST_HISTORY_PATH.open("a", encoding="utf-8") as f:
            f.write(line + "\n")


def read_checklist_history(limit: int = 50) -> list[dict[str, Any]]:
    lim = max(1, min(int(limit), 500))
    path = CHECKLIST_HISTORY_PATH
    if not path.is_file():
        return []
    with _HISTORY_LOCK:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError:
            return []
    out: list[dict[str, Any]] = []
    for raw in lines[-lim:]:
        raw = raw.strip()
        if not raw:
            continue
        try:
            row = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if isinstance(row, dict):
            out.append(row)
    out.reverse()
    return out
