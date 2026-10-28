#!/usr/bin/env python3
"""Arranca TCM HMI y abre Google Chrome.

  python3 scripts/iniciar_hmi.py              # servidor en esta terminal + Chrome
  python3 scripts/iniciar_hmi.py --detach     # reinicia servidor en segundo plano + Chrome
  python3 scripts/iniciar_hmi.py --install-shortcut
  python3 scripts/iniciar_hmi.py --port 5050 --no-browser
"""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parent
HMI_ROOT = _SCRIPTS_DIR.parent
TCM_ROOT = HMI_ROOT.parent
PORT_DEFAULT = 5050
LAUNCHER = Path(__file__).resolve()
