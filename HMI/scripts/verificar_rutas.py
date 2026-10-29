#!/usr/bin/env python3
"""Comprueba rutas clave del HMI tras mover carpetas (sin red ni Flask)."""

from __future__ import annotations

import sys
from pathlib import Path

HMI_ROOT = Path(__file__).resolve().parent.parent
REQUIRED = [
    HMI_ROOT / "app.py",
    HMI_ROOT / "iniciar_hmi.py",
    HMI_ROOT / "Iniciar-HMI.bat",
    HMI_ROOT / "cycle.py",
    HMI_ROOT / "requirements.txt",
    HMI_ROOT / "config" / "cycle.json",
    HMI_ROOT / "config" / "model.json",
    HMI_ROOT / "core" / "state.py",
    HMI_ROOT / "core" / "tcp_link.py",
    HMI_ROOT / "clients" / "motion.py",
    HMI_ROOT / "clients" / "plc.py",
    HMI_ROOT / "clients" / "prefeeder.py",
    HMI_ROOT / "frontend" / "package.json",
]
VENV_HINT = HMI_ROOT.parent / ".venv"


def main() -> int:
    ok = True
    for p in REQUIRED:
        if p.is_file():
            print(f"OK  {p.relative_to(HMI_ROOT)}")
        else:
            print(f"FALTA  {p}", file=sys.stderr)
            ok = False
    dist = HMI_ROOT / "frontend" / "dist" / "index.html"
    if dist.is_file():
        print(f"OK  {dist.relative_to(HMI_ROOT)}")
    else:
        print(
            "AVISO  frontend/dist/index.html (Iniciar-HMI.bat compilara en el primer arranque)"
        )
    if VENV_HINT.is_dir():
        print(f"OK  venv en {VENV_HINT}")
    else:
        print(f"AVISO  sin {VENV_HINT} — usar python del sistema o crear venv en la raiz del repo")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
