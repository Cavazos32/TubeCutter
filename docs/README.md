# TCM HMI — Flask + React

Interfaz maestro en Python (Flask) + React/Vite para los módulos de máquina vía TCP JSON.

| Módulo | IP:puerto por defecto |
|--------|------------------------|
| Motion | `10.10.32.20:8767` |
| PLC | `10.10.32.51:8766` |
| PreFeeder | `10.10.32.100:8768` |

## Estructura

```
HMI/
├── app.py              # Servidor Flask, API REST, SSE, sirve UI compilada
├── iniciar_hmi.py      # Lanzador: venv, build UI, Chrome, acceso directo
├── state.py            # Estado + lógica de enlace
├── cycle.py            # Secuencia de ciclo
├── motion.py           # Cliente TCP Motion
├── plc.py              # Cliente TCP PLC
├── prefeeder.py        # Cliente TCP PreFeeder
├── config/
│   ├── models.json
│   └── cycle_config.json
├── frontend/           # UI React (Vite + Tailwind) — fuente
│   └── dist/           # Build de producción (`npm run build`)
├── deploy/
│   └── tcm-hmi.service # systemd (Raspberry Pi)
└── requirements.txt
```

## Requisitos

- Python 3.10+
- Node.js 18+ (build/desarrollo de la UI)
- Módulos firmware con servidor TCP en la misma red

## Instalación

```bash
# Desde la raiz TCM
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r HMI/requirements.txt

cd HMI/frontend
npm install
npm run build
```

## Ejecutar

```bash
# Con el venv de la raiz TCM activado
cd HMI
python app.py
```

Abrir: **http://localhost:5050**

Al arrancar, si `frontend/src` es más reciente que `dist`, `app.py` ejecuta `npm run build` solo (desactivar con `HMI_AUTO_BUILD_UI=0`). Flask **siempre** sirve `frontend/dist`, no el código fuente.

### Arranque (`iniciar_hmi.py`)

Usa **`TCM/.venv`**, compila la UI si falta `frontend/dist`, levanta `app.py` y abre **Google Chrome** (perfil opcional: `HMI_CHROME_PROFILE` o `~/chrome-profile/Fork-3` en Linux).

```bash
cd HMI
python3 iniciar_hmi.py
```

| Modo | Comando |
|------|---------|
| Desarrollo (servidor en la terminal) | `python3 iniciar_hmi.py` |
| Icono / kiosk (reinicia servidor en 2.º plano + Chrome) | `python3 iniciar_hmi.py --detach` |
| Crear acceso directo en el Escritorio | `python3 iniciar_hmi.py --install-shortcut` |

Opciones: `--port N`, `--no-browser`. Si el servidor ya corre, solo abre Chrome.

Log del servidor en `--detach`: `HMI_LOG` o `/tmp/tcm-hmi-server.log` (Linux) / `HMI/logs/hmi-server.log` (Windows).

### Desarrollo UI (hot reload)

Terminal 1 — backend:
```bash
cd HMI && python app.py
```

Terminal 2 — frontend:
```bash
cd HMI/frontend && npm run dev
```

Abrir: **http://localhost:3000** (proxy `/api` → `:5050`).

## Funciones de la UI

- **Máquina**: Start/Stop/Resume, modelo, progreso de lote, log
- **Cycle**: secuencia del ciclo (esperas en `cycle.py`); depósito/alimentación en `config/cycle_config.json`
- **Motion**: ASDA B3, encoder, feeder, offsets
- **PLC**: válvulas, reset, all off
- **PreFeeder**: start/stop/reset/materialist, sensores L/R
- **Ajustes** (☰): tema, idioma ES/EN, enlaces TCP

Variables: `HMI_HOST` (default `0.0.0.0`), `HMI_PORT` (default `5050`).

## Raspberry Pi

1. Copiar `HMI/` a la RPi.
2. Ajustar IPs en `motion.py`, `plc.py`, `prefeeder.py`.
3. `npm run build` en `frontend/`, luego `python app.py`.
4. Arranque automático: `deploy/tcm-hmi.service`.
