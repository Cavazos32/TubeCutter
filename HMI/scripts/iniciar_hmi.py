#!/usr/bin/env python3
"""Delega en HMI/iniciar_hmi.py (compatibilidad si ejecutas desde scripts/)."""

from __future__ import annotations

import os
import sys
from pathlib import Path

HMI_ROOT = Path(__file__).resolve().parent.parent
ROOT_LAUNCHER = HMI_ROOT / "iniciar_hmi.py"

if not ROOT_LAUNCHER.is_file():
    print(f"No se encontro {ROOT_LAUNCHER}", file=sys.stderr)
    raise SystemExit(1)

os.chdir(HMI_ROOT)
os.execv(sys.executable, [sys.executable, "-u", str(ROOT_LAUNCHER), *sys.argv[1:]])
