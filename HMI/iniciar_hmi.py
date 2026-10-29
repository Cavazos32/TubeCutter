#!/usr/bin/env python3
"""Arranca TCM HMI y abre Google Chrome.

  python3 iniciar_hmi.py              # servidor en esta terminal + Chrome
  python3 iniciar_hmi.py --detach     # reinicia servidor en segundo plano + Chrome (icono)
  python3 iniciar_hmi.py --install-shortcut
  python3 iniciar_hmi.py --port 5050 --no-browser
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

HMI_ROOT = Path(__file__).resolve().parent
TCM_ROOT = HMI_ROOT.parent
PORT_DEFAULT = 5050
LAUNCHER = Path(__file__).resolve()


def _venv_python() -> Path | None:
    if sys.platform == "win32":
        candidate = TCM_ROOT / ".venv" / "Scripts" / "python.exe"
    else:
        candidate = TCM_ROOT / ".venv" / "bin" / "python"
    return candidate if candidate.is_file() else None


def _running_in_venv() -> bool:
    venv = _venv_python()
    if venv is None:
        return False
    try:
        return Path(sys.executable).resolve() == venv.resolve()
    except OSError:
        return False


def _ensure_venv_python(*, require_venv: bool = False) -> None:
    if _running_in_venv():
        os.environ.setdefault("PYTHONUNBUFFERED", "1")
        return
    venv = _venv_python()
    if venv is None:
        if require_venv:
            repo = TCM_ROOT.name
            print(
                f"No se encontro {repo}/.venv (raiz del repo).\n"
                f"  cd {repo}\n"
                "  python -m venv .venv\n"
                "  .venv\\Scripts\\activate   # Linux/macOS: source .venv/bin/activate\n"
                "  pip install -r HMI/requirements.txt",
                file=sys.stderr,
            )
            sys.exit(1)
        os.environ.setdefault("PYTHONUNBUFFERED", "1")
        return
    os.environ["PYTHONUNBUFFERED"] = "1"
    os.execv(str(venv), [str(venv), "-u", str(LAUNCHER), *sys.argv[1:]])


def _port_open(port: int, timeout_s: float = 0.4) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout_s)
    try:
        sock.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def _wait_port(port: int, attempts: int = 60, delay_s: float = 0.5) -> bool:
    for _ in range(attempts):
        if _port_open(port):
            return True
        time.sleep(delay_s)
    return False


def _chrome_exe() -> Path | None:
    if sys.platform == "win32":
        candidates = [
            os.path.expandvars(r"%ProgramFiles%\Google\Chrome\Application\chrome.exe"),
            os.path.expandvars(r"%ProgramFiles(x86)%\Google\Chrome\Application\chrome.exe"),
            os.path.expandvars(r"%LocalAppData%\Google\Chrome\Application\chrome.exe"),
        ]
        for raw in candidates:
            p = Path(raw)
            if p.is_file():
                return p
        return None
    for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
        found = shutil.which(name)
        if found:
            return Path(found)
    return None


def _chrome_profile_dir() -> Path | None:
    env = os.environ.get("HMI_CHROME_PROFILE", "").strip()
    if env:
        p = Path(env).expanduser()
        return p if p.is_dir() else None
    if sys.platform == "win32":
        return None
    default = Path.home() / "chrome-profile" / "Fork-3"
    return default if default.is_dir() else None


def _open_chrome(url: str) -> None:
    chrome = _chrome_exe()
    if chrome is None:
        print(f"No se encontro Chrome. Abre manualmente: {url}")
        return
    profile = _chrome_profile_dir()
    cmd: list[str | Path] = [chrome, "--new-window"]
    if profile is not None:
        cmd.extend(
            [
                "--no-sandbox",
                f"--user-data-dir={profile}",
                "--password-store=basic",
                "--proxy-bypass-list=localhost;127.0.0.1;[::1]",
            ]
        )
    cmd.append(url)
    flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
    try:
        subprocess.Popen(
            [str(x) for x in cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=flags,
        )
    except OSError as exc:
        print(f"No se pudo abrir Chrome ({exc}). URL: {url}")


def _kill_port(port: int) -> None:
    if sys.platform == "win32":
        ps = (
            f"Get-NetTCPConnection -LocalPort {port} -ErrorAction SilentlyContinue | "
            "ForEach-Object { Stop-Process -Id $_.OwningProcess -Force -ErrorAction SilentlyContinue }"
        )
        subprocess.run(
            ["powershell", "-NoProfile", "-Command", ps],
            cwd=HMI_ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    if shutil.which("fuser"):
        subprocess.run(
            ["fuser", "-k", f"{port}/tcp"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    if shutil.which("lsof"):
        proc = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True,
            text=True,
        )
        for pid in proc.stdout.split():
            subprocess.run(["kill", "-9", pid], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _npm_cmd() -> list[str]:
    return ["npm.cmd"] if sys.platform == "win32" else ["npm"]


def _ensure_frontend_deps() -> None:
    frontend = HMI_ROOT / "frontend"
    vite_bin = frontend / "node_modules" / ".bin" / (
        "vite.cmd" if sys.platform == "win32" else "vite"
    )
    if vite_bin.is_file():
        return
    print("Instalando dependencias UI (npm install)...")
    result = subprocess.run(
        [*_npm_cmd(), "install"],
        cwd=frontend,
        shell=(sys.platform == "win32"),
    )
    if result.returncode != 0:
        print("npm install fallo", file=sys.stderr)
        _pause("Enter para cerrar ")
        sys.exit(1)


def _build_frontend_if_needed() -> None:
    if (HMI_ROOT / "frontend" / "dist" / "index.html").is_file():
        return
    _ensure_frontend_deps()
    print("Compilando UI (frontend)...")
    result = subprocess.run(
        [*_npm_cmd(), "run", "build"],
        cwd=HMI_ROOT / "frontend",
        shell=(sys.platform == "win32"),
    )
    if result.returncode != 0:
        print("npm run build fallo", file=sys.stderr)
        _pause("Enter para cerrar ")
        sys.exit(1)


def _pause(msg: str) -> None:
    try:
        input(msg)
    except EOFError:
        pass


def _apply_launch_env(port: int) -> None:
    os.environ.setdefault("HMI_PORT", str(port))
    os.environ.setdefault("HMI_CHECKLIST_SKIP", "1")
    os.environ["PYTHONUNBUFFERED"] = "1"


def _server_log_path() -> Path:
    env = os.environ.get("HMI_LOG", "").strip()
    if env:
        return Path(env)
    if sys.platform == "win32":
        return HMI_ROOT / "logs" / "hmi-server.log"
    return Path("/tmp/tcm-hmi-server.log")


def _start_server_background() -> Path:
    log_path = _server_log_path()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = open(log_path, "a", encoding="utf-8")
    subprocess.Popen(
        [sys.executable, "-u", str(HMI_ROOT / "app.py")],
        cwd=HMI_ROOT,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return log_path


def _run_detach(port: int, url: str, *, open_browser: bool) -> int:
    _kill_port(port)
    time.sleep(1)
    _build_frontend_if_needed()
    _apply_launch_env(port)
    log_path = _start_server_background()
    if not _wait_port(port):
        print(f"El servidor no respondio en {url} (ver {log_path})", file=sys.stderr)
        return 1
    if open_browser:
        _open_chrome(url)
    print(f"TCM HMI en segundo plano — {url}")
    print(f"Log: {log_path}")
    return 0


def _install_shortcut(port: int) -> int:
    bat = HMI_ROOT / "Iniciar-HMI.bat"

    if sys.platform == "win32":
        if not bat.is_file():
            print(f"No se encontro {bat}", file=sys.stderr)
            return 1
        desktop = Path.home() / "Desktop"
        lnk = desktop / "TCM HMI.lnk"
        ps = f"""
$Wsh = New-Object -ComObject WScript.Shell
$s = $Wsh.CreateShortcut('{lnk}')
$s.TargetPath = '{bat}'
$s.WorkingDirectory = '{HMI_ROOT}'
$s.Description = 'TCM HMI — abrir Chrome; cerrar con Ctrl+C en la ventana negra'
$s.IconLocation = "$env:SystemRoot\\System32\\shell32.dll,13"
$s.Save()
"""
        subprocess.run(["powershell", "-NoProfile", "-Command", ps], check=True)
        print(f"Acceso directo: {lnk}")
        print(f"  → ejecuta {bat.name} (terminal visible + Chrome)")
        if port != PORT_DEFAULT:
            print(f"  (Puerto distinto de {PORT_DEFAULT}: edita Iniciar-HMI.bat o usa --port en iniciar_hmi.py)")
        return 0

    py = _venv_python() or Path(sys.executable)
    args = f'-u "{LAUNCHER}" --restart --port {port}'

    desktop_dir = None
    for d in (Path.home() / "Escritorio", Path.home() / "Desktop"):
        if d.is_dir():
            desktop_dir = d
            break
    if desktop_dir is None:
        desktop_dir = Path.home() / "Desktop"
        desktop_dir.mkdir(parents=True, exist_ok=True)

    desktop_file = desktop_dir / "TCM-HMI.desktop"
    exec_line = f'"{py}" {args}'
    desktop_file.write_text(
        "\n".join(
            [
                "[Desktop Entry]",
                "Version=1.0",
                "Type=Application",
                "Name=TCM HMI",
                "Comment=Reinicia el servidor y abre Chrome",
                f"Exec={exec_line}",
                "Icon=applications-engineering",
                "Terminal=false",
                "Categories=Utility;",
                "",
            ]
        ),
        encoding="utf-8",
    )
    desktop_file.chmod(desktop_file.stat().st_mode | 0o111)
    if shutil.which("gio"):
        subprocess.run(
            ["gio", "set", str(desktop_file), "metadata::trusted", "true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    print(f"Acceso directo: {desktop_file}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Arranca TCM HMI y abre Chrome")
    parser.add_argument("--port", type=int, default=PORT_DEFAULT)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "--detach",
        action="store_true",
        help="Reinicia el servidor en segundo plano y abre Chrome (acceso directo)",
    )
    parser.add_argument(
        "--install-shortcut",
        action="store_true",
        help="Crea icono en el Escritorio (--detach + Chrome)",
    )
    parser.add_argument(
        "--restart",
        action="store_true",
        help="Si el puerto esta ocupado, detiene el servidor anterior y arranca de nuevo",
    )
    args = parser.parse_args()

    if args.install_shortcut:
        _ensure_venv_python(require_venv=False)
        return _install_shortcut(args.port)

    _ensure_venv_python(require_venv=False)

    port = args.port
    url = f"http://127.0.0.1:{port}"
    open_browser = not args.no_browser

    if args.detach:
        return _run_detach(port, url, open_browser=open_browser)

    _apply_launch_env(port)

    if _port_open(port):
        if args.restart:
            print(f"Reiniciando servidor HMI en puerto {port}…")
            _kill_port(port)
            time.sleep(1.2)
            if _port_open(port):
                print("No se pudo liberar el puerto. Cierra la terminal del servidor manualmente.")
                return 1
        else:
            print(f"TCM HMI ya esta en ejecucion ({url})")
            print("(El codigo nuevo NO se carga hasta reiniciar el servidor.)")
            print(f"  python iniciar_hmi.py --restart --port {port}")
            if open_browser:
                _open_chrome(url)
            print("Si la pagina esta en blanco: Ctrl+F5")
            _pause("Enter para salir ")
            return 0

    _build_frontend_if_needed()

    if open_browser:
        threading.Thread(
            target=lambda: _wait_port(port) and _open_chrome(url),
            daemon=True,
        ).start()

    print(f"TCM HMI - {url}")
    print("Deja esta ventana abierta. Para apagar el HMI: Ctrl+C.")
    print("(En Windows puedes usar Iniciar-HMI.bat o el acceso directo del Escritorio.)")
    print()

    result = subprocess.run(
        [sys.executable, "-u", str(HMI_ROOT / "app.py")],
        cwd=HMI_ROOT,
    )
    code = int(result.returncode or 0)
    if code != 0:
        print(f"\nEl servidor salio con error {code}")
        _pause("Enter para cerrar ")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
