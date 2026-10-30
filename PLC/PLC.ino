// PLCA — sensores + válvulas por pulso. Ver Config.h (IP, TCP :8766, JSON+newline).

#include <WiFi.h>
#include <ArduinoOTA.h>
#include "Config.h"
#include "PlcStates.h"

WiFiServer tcpServer(PLCA_TCP_PORT);
WiFiClient tcpClient;

// --- Estado global ---
bool wifiReady = false;
bool tcpServicesUp = false;
bool tcpWasConnected = false;
bool peerLinkOk = false;

uint8_t lastBitmask = 0xFF;
uint8_t rawCandidateMask = 0xFF;
unsigned long rawCandidateSinceMs = 0;
unsigned long lastSnapshotTxMs = 0;
unsigned long lastStatusPushMs = 0;
// Grace de sensores desde fin de setup() (WiFi puede tardar > BOOT_GRACE).
unsigned long bootGraceStartMs = 0;
uint8_t sequence = 0;

bool stCutterR   = false;
bool stCutterL   = false;
bool stCutters   = false;
bool stGrippers  = false;
bool stHolder    = false;  // no auto-ON: lo activa rutina / manual
bool stEncoder   = false;  // válvula encoder (0x1D)
bool stBlower    = false;
bool stReset     = false;
static bool blowerTimedActive = false;
static unsigned long blowerStartMs = 0;
static unsigned long blowerHoldMs = 0;

char tcpRxLine[512];
uint16_t tcpRxLen = 0;
unsigned long wifiConnectStartedMs = 0;
bool staWasConnected = false;

static bool lastPeerCutterR = false;
static bool lastPeerCutterL = false;
static bool lastPeerCutters = false;
static bool lastPeerGrippers = false;
static bool lastPeerHolder = false;
static bool lastPeerEncoder = false;
static bool lastPeerBlower = false;
static bool lastPeerReset = false;
static uint8_t lastPeerMask = 0xFF;

static uint8_t plcTcpLastStateByte = 0;
static bool plcTcpNeedInit = false;
static bool plcTcpStopPending = false;

// --- Sensores ---
static uint8_t readSensorBitmask()
{
  uint8_t mask = 0;
  if (!digitalRead(PIN_CUTTER))  mask |= BIT_CUTTER;
  if (!digitalRead(PIN_GRIPPER)) mask |= BIT_GRIPPER;
  if (!digitalRead(PIN_HOLDER))  mask |= BIT_HOLDER;
  if (!digitalRead(PIN_ENCODER)) mask |= BIT_ENCODER;
  return mask;
}

static uint8_t currentStableMask()
{
  return (lastBitmask == 0xFF) ? 0 : lastBitmask;
}

static bool updateDebouncedMask(uint8_t raw, uint8_t& stableOut, unsigned long now)
{
  if (raw != rawCandidateMask)
  {
    rawCandidateMask = raw;
    rawCandidateSinceMs = now;
  }

  if ((now - rawCandidateSinceMs) < SENSOR_DEBOUNCE_MS)
  {
    stableOut = (lastBitmask == 0xFF) ? 0 : lastBitmask;
    return false;
  }

  stableOut = rawCandidateMask;
  if (lastBitmask == 0xFF)
    return true;
  return stableOut != lastBitmask;
}

// --- Válvulas (pulso ON/OFF) ---
static void blowerStop();
static bool plcDoHardwareReset(uint8_t ackByte, uint8_t flags, bool& deferred);

static void valveWritePin(uint8_t pin, bool on)
{
#if VALVE_ACTIVE_HIGH
  digitalWrite(pin, on ? HIGH : LOW);
#else
  digitalWrite(pin, on ? LOW : HIGH);
#endif
}

static void valveIdleAll()
{
  valveWritePin(PIN_OUT_CUTTER_R, false);
  valveWritePin(PIN_OUT_CUTTER_L, false);
  valveWritePin(PIN_OUT_GRIPPERS, false);
  valveWritePin(PIN_OUT_HOLDER,   false);
  valveWritePin(PIN_OUT_ENCODER,  false);
  valveWritePin(PIN_OUT_BLOWER,   false);
  valveWritePin(PIN_OUT_RESET,    false);
}

// El pulso KEEP (100 ms + hueco) no puede congelar loop(): ahí se atienden
// el socket y el ACK. Si el lazo se queda dentro del pulso, el ACK sube de
// ~80 ms a cientos y un write fallido cerraba el TCP (E066).
static const uint8_t VALVE_JOB_Q = 8;
static const uint8_t JOB_ACK = 1;
static const uint8_t JOB_STATUS = 2;
static const uint8_t LINE_Q = 8;

struct ValveJob {
  uint8_t pins[4];
  uint8_t n;
  uint8_t ackByte;
  uint8_t flags;
};

static ValveJob valveJobs[VALVE_JOB_Q];
static uint8_t valveJobHead = 0;
static uint8_t valveJobCount = 0;
static unsigned long lastValvePulseEndMs = 0;

static char lineQ[LINE_Q][240];
static uint8_t lineQHead = 0;
static uint8_t lineQn = 0;

enum : uint8_t { VP_IDLE = 0, VP_GAP = 1, VP_HIGH = 2 };
static uint8_t vpPhase = VP_IDLE;
static uint8_t vpPins[4];
static uint8_t vpN = 0;
static uint8_t vpAckByte = 0;
static uint8_t vpFlags = 0;
static unsigned long vpPhaseMs = 0;

static bool valveEnqueue(const uint8_t* pins, uint8_t n, uint8_t ackByte, uint8_t flags)
{
  if (!pins || n == 0 || n > 4) return false;
  if (valveJobCount >= VALVE_JOB_Q) return false;
  ValveJob& j = valveJobs[(valveJobHead + valveJobCount) % VALVE_JOB_Q];
  for (uint8_t i = 0; i < n; i++)
    j.pins[i] = pins[i];
  j.n = n;
  j.ackByte = ackByte;
  j.flags = flags;
  valveJobCount++;
  return true;
}

static void syncCuttersFlag()
{
  stCutters = stCutterR || stCutterL;
}

// Corta pulsos pendientes y baja GPIO. El HIGH a medias se apaga aquí.
static void valveAbortAll()
{
  if (vpPhase == VP_HIGH) {
    for (uint8_t i = 0; i < vpN; i++)
      valveWritePin(vpPins[i], false);
    lastValvePulseEndMs = millis();
  }
  vpPhase = VP_IDLE;
  vpN = 0;
  vpFlags = 0;
  valveJobCount = 0;
  valveJobHead = 0;
  lineQn = 0;
  lineQHead = 0;
  valveIdleAll();
}

// Cliente nuevo: no ACK de la sesión vieja. El HIGH en curso sí termina.
static void valveDropPending()
{
  valveJobCount = 0;
  valveJobHead = 0;
  lineQn = 0;
  lineQHead = 0;
  vpFlags = 0;
}

static bool valveArm(bool& st, uint8_t pin, bool wantOn, uint8_t ackByte, uint8_t flags, bool& deferred)
{
  deferred = false;
  if (st == wantOn) return true;
  if (!valveEnqueue(&pin, 1, ackByte, flags)) return false;
  st = wantOn;
  syncCuttersFlag();
  deferred = true;
  return true;
}

static void blowerStop()
{
  blowerTimedActive = false;
  blowerHoldMs = 0;
  valveWritePin(PIN_OUT_BLOWER, false);
  stBlower = false;
}

static void blowerStart(unsigned long durationMs)
{
  if (durationMs < BLOWER_MIN_MS) durationMs = BLOWER_MIN_MS;
  if (durationMs > BLOWER_MAX_MS) durationMs = BLOWER_MAX_MS;
  valveWritePin(PIN_OUT_BLOWER, true);
  stBlower = true;
  blowerTimedActive = true;
  blowerStartMs = millis();
  blowerHoldMs = durationMs;
}

static bool blowerSet(bool wantOn, unsigned long durationMs)
{
  if (!wantOn) {
    if (!stBlower && !blowerTimedActive) return false;
    blowerStop();
    return true;
  }
  blowerStart(durationMs);
  return true;
}

static bool nameEq(const char* a, const char* b)
{
  while (*a && *b) {
    char ca = *a;
    char cb = *b;
    if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 'a' + 'A');
    if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 'a' + 'A');
    if (ca != cb) return false;
    a++;
    b++;
  }
  return *a == 0 && *b == 0;
}

static bool setPlcOutputByName(const char* outName, bool state, bool& deferred)
{
  deferred = false;
  if (!outName || !outName[0]) return false;
  if (nameEq(outName, "CUTTER") || nameEq(outName, "CUTTERS")) {
    const bool needR = (stCutterR != state);
    const bool needL = (stCutterL != state);
    if (needR || needL) {
      uint8_t pins[2];
      uint8_t np = 0;
      if (needR) pins[np++] = PIN_OUT_CUTTER_R;
      if (needL) pins[np++] = PIN_OUT_CUTTER_L;
      if (!valveEnqueue(pins, np, 0, JOB_STATUS)) return false;
      if (needR) stCutterR = state;
      if (needL) stCutterL = state;
      syncCuttersFlag();
      deferred = true;
    }
  }
  else if (nameEq(outName, "CUTTER_R") || nameEq(outName, "CUTTERS_R")) {
    if (!valveArm(stCutterR, PIN_OUT_CUTTER_R, state, 0, JOB_STATUS, deferred)) return false;
  }
  else if (nameEq(outName, "CUTTER_L") || nameEq(outName, "CUTTERS_L")) {
    if (!valveArm(stCutterL, PIN_OUT_CUTTER_L, state, 0, JOB_STATUS, deferred)) return false;
  }
  else if (nameEq(outName, "GRIPPER") || nameEq(outName, "GRIPPERS")) {
    if (!valveArm(stGrippers, PIN_OUT_GRIPPERS, state, 0, JOB_STATUS, deferred)) return false;
  }
  else if (nameEq(outName, "HOLDER")) {
    if (!valveArm(stHolder, PIN_OUT_HOLDER, state, 0, JOB_STATUS, deferred)) return false;
  }
  else if (nameEq(outName, "FGTRAY") || nameEq(outName, "TRAY") || nameEq(outName, "ENCODER")) {
    if (!valveArm(stEncoder, PIN_OUT_ENCODER, state, 0, JOB_STATUS, deferred)) return false;
  }
  else if (nameEq(outName, "BLOWER"))
    blowerSet(state, BLOWER_DEFAULT_SEC * 1000UL);
  else if (nameEq(outName, "RESET")) {
    if (!plcDoHardwareReset(0, JOB_STATUS, deferred)) return false;
  }
  else return false;
  return true;
}

// HOME / All Off: un pulso OFF por válvula lógica ON, en cola (no bloquea).
static bool plcGoHomePulse()
{
  struct Item { bool* st; uint8_t pin; };
  Item items[5] = {
    { &stCutterR,  PIN_OUT_CUTTER_R },
    { &stCutterL,  PIN_OUT_CUTTER_L },
    { &stGrippers, PIN_OUT_GRIPPERS },
    { &stEncoder,  PIN_OUT_ENCODER },
    { &stHolder,   PIN_OUT_HOLDER },
  };
  uint8_t idx[5];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 5; i++) {
    if (*(items[i].st)) idx[n++] = i;
  }
  blowerStop();
  stReset = false;
  if (!n) {
    syncCuttersFlag();
    return false;
  }
  for (uint8_t k = 0; k < n; k++) {
    Item& it = items[idx[k]];
    uint8_t pin = it.pin;
    uint8_t flags = (k + 1 == n) ? JOB_STATUS : 0;
    if (!valveEnqueue(&pin, 1, 0, flags)) return true;
    *(it.st) = false;
  }
  syncCuttersFlag();
  return true;
}

// Reset 0x1E: GPIO idle + lógica OFF. El pulso de reset queda en cola.
static bool plcDoHardwareReset(uint8_t ackByte, uint8_t flags, bool& deferred)
{
  deferred = false;
  blowerStop();
  valveAbortAll();
  stCutterR = false;
  stCutterL = false;
  stGrippers = false;
  stHolder = false;
  stEncoder = false;
  stReset = false;
  syncCuttersFlag();
  uint8_t pin = PIN_OUT_RESET;
  if (!valveEnqueue(&pin, 1, ackByte, flags)) return false;
  deferred = true;
  return true;
}

static bool parseOnOffStr(const char* val)
{
  return val && (nameEq(val, "1") || nameEq(val, "true") || nameEq(val, "on"));
}

// --- TCP TX ---
static bool tcpLinkOk()
{
  return tcpClient && tcpClient.connected();
}

// Un write corto no es un enlace muerto. availableForWrite()==0 en el ESP32
// justo tras el accept es un falso "lleno": si no se escribe, la HMI no recibe
// hello/pong y la reconexión queda plc=OFF con el socket abierto.
static char txHold[700];
static bool txHoldOn = false;

static void txHoldReset()
{
  txHoldOn = false;
  txHold[0] = 0;
}

static bool tcpSendRaw(const char* out, size_t n)
{
  if (!out || !n || !tcpLinkOk()) return false;
  const int room = tcpClient.availableForWrite();
  if (room > 0 && room < (int)n) return false;
  const size_t sent = tcpClient.write((const uint8_t*)out, n);
  if (sent == n) return true;
  if (!tcpClient.connected()) {
    tcpClient.stop();
    peerLinkOk = false;
  }
  return false;
}

static bool tcpTx(const char* m)
{
  if (!m || !m[0]) return false;
  size_t n = strlen(m);
  char tmp[700];
  const char* out = m;
  if (m[n - 1] != '\n') {
    if (n + 2 > sizeof(tmp)) return false;
    memcpy(tmp, m, n);
    tmp[n] = '\n';
    tmp[n + 1] = 0;
    out = tmp;
    n++;
  }
  if (tcpSendRaw(out, n)) return true;
  if (!txHoldOn && n < sizeof(txHold)) {
    memcpy(txHold, out, n);
    txHold[n] = 0;
    txHoldOn = true;
  }
  return false;
}

static const uint8_t ACK_Q = 4;
static char ackQ[ACK_Q][192];
static uint8_t ackQHead = 0;
static uint8_t ackQn = 0;

static void ackReset()
{
  ackQHead = 0;
  ackQn = 0;
}

static void ackPush(const char* line)
{
  if (!line || !line[0]) return;
  if (tcpSendRaw(line, strlen(line))) return;
  if (!tcpLinkOk()) return;
  if (ackQn >= ACK_Q) {
    Serial.println("[TCP] ACK no enviado");
    return;
  }
  char* dst = ackQ[(ackQHead + ackQn) % ACK_Q];
  strncpy(dst, line, 191);
  dst[191] = 0;
  ackQn++;
}

static void tcpTxPump()
{
  if (!tcpLinkOk()) {
    if (!tcpClient) {
      txHoldReset();
      ackReset();
    }
    return;
  }
  if (txHoldOn) {
    if (tcpSendRaw(txHold, strlen(txHold))) txHoldReset();
    else return;
  }
  while (ackQn && tcpSendRaw(ackQ[ackQHead], strlen(ackQ[ackQHead]))) {
    ackQHead = (ackQHead + 1) % ACK_Q;
    ackQn--;
  }
}

static void plcSendAck(uint8_t cmdByte, bool ok, const char* message)
{
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"ack\",\"actuator\":\"plc\",\"byte\":%u,"
           "\"ok\":%s,\"message\":\"%s\"}\n",
           (unsigned)PLCA_PROTO_VER, (unsigned)cmdByte,
           ok ? "true" : "false", message ? message : "");
  ackPush(buf);
}

static bool plcInBootGrace()
{
  if (bootGraceStartMs == 0) return true;
  return (millis() - bootGraceStartMs) < BOOT_GRACE_MS;
}

static bool plcHasSensorError()
{
  if (plcInBootGrace()) return false;
  return currentStableMask() != 0;
}

static bool plcHasValveBusy()
{
  // Mientras el KEEP está en alto (o en cola) no reportar Idle: el ACK aún no salió.
  if (vpPhase != VP_IDLE || valveJobCount > 0) return true;
  return stCutterR || stCutterL || stGrippers || stEncoder || stBlower || stReset;
}

static uint8_t plcTcpResolveStateByte()
{
  if (plcHasSensorError()) return PLC_TX_ERROR;
  if (plcTcpStopPending) {
    if (plcHasValveBusy()) return PLC_TX_STOP;
    return PLC_TX_RETURN;
  }
  if (plcHasValveBusy()) return PLC_TX_BUSY;
  return PLC_TX_IDLE;
}

static const char* plcTcpStateName(uint8_t byteCode)
{
  switch (byteCode) {
    case PLC_TX_INIT:   return "InitState";
    case PLC_TX_IDLE:   return "IdleState";
    case PLC_TX_BUSY:   return "BusyState";
    case PLC_TX_ERROR:  return "ErrorState";
    case PLC_TX_STOP:   return "StoprState";
    case PLC_TX_RETURN: return "ReturnState";
    default:            return "unknown";
  }
}

static void plcTcpTxState(uint8_t byteCode, const char* name)
{
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"state\",\"actuator\":\"plc\",\"byte\":%u,\"name\":\"%s\"}",
           (unsigned)PLCA_PROTO_VER, (unsigned)byteCode, name);
  tcpTx(buf);
  plcTcpLastStateByte = byteCode;
}

static void plcTcpPushStateIfChanged()
{
  const uint8_t st = plcTcpResolveStateByte();
  if (st == plcTcpLastStateByte) return;
  plcTcpTxState(st, plcTcpStateName(st));
  if (st == PLC_TX_RETURN)
    plcTcpStopPending = false;
}

static void plcTcpPollStateEvents()
{
  if (!tcpLinkOk()) return;

  if (plcTcpNeedInit) {
    plcTcpTxState(PLC_TX_INIT, "InitState");
    plcTcpNeedInit = false;
  }

  plcTcpPushStateIfChanged();
}

static const char* jsonBool(bool v)
{
  return v ? "true" : "false";
}

static bool buildStatusJson(char* out, size_t n, const char* type)
{
  const uint8_t mask = currentStableMask();
  const int w = snprintf(
      out, n,
      "{\"ver\":%u,\"type\":\"%s\",\"role\":\"plca\",\"mask\":%u,\"seq\":%u,"
      "\"sensorGripper\":%s,\"sensorHolder\":%s,\"sensorCutter\":%s,"
      "\"sensorHoseA\":%s,\"sensorHoseB\":%s,\"sensorTray\":%s,"
      "\"cutterR\":%s,\"cutterL\":%s,\"cutters\":%s,\"grippers\":%s,"
      "\"holder\":%s,\"encoder\":%s,\"fgtray\":%s,\"blower\":%s,\"reset\":%s}\n",
      (unsigned)PLCA_PROTO_VER, type, (unsigned)mask, (unsigned)sequence,
      jsonBool(mask & BIT_GRIPPER), jsonBool(mask & BIT_HOLDER),
      jsonBool(mask & BIT_CUTTER), jsonBool(mask & BIT_HOSE_A),
      jsonBool(mask & BIT_HOSE_B), jsonBool(mask & BIT_ENCODER),
      jsonBool(stCutterR), jsonBool(stCutterL), jsonBool(stCutters),
      jsonBool(stGrippers), jsonBool(stHolder), jsonBool(stEncoder),
      jsonBool(stEncoder), jsonBool(stBlower), jsonBool(stReset));
  return w > 0 && (size_t)w < n;
}

static bool tcpTxStatus(const char* type)
{
  char buf[640];
  if (!buildStatusJson(buf, sizeof(buf), type)) return false;
  return tcpTx(buf);
}

static bool sendSnapshot(uint8_t bitmask)
{
  char buf[96];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"event\",\"field\":\"snapshot\",\"mask\":%u,\"seq\":%u}\n",
           (unsigned)bitmask, (unsigned)sequence);
  if (!tcpTx(buf))
    return false;
  sequence++;
  lastSnapshotTxMs = millis();
  return true;
}

static void sendAlert(uint8_t code)
{
  char buf[64];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"event\",\"field\":\"alert\",\"code\":%u}\n", (unsigned)code);
  tcpTx(buf);
}

static void plcTcpTxEvent(uint8_t byteCode, const char* name)
{
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"event\",\"actuator\":\"plc\",\"byte\":%u,\"name\":\"%s\"}",
           (unsigned)PLCA_PROTO_VER, (unsigned)byteCode, name);
  tcpTx(buf);
}

static void plcTcpTxValve(uint8_t byteCode, const char* name, bool on)
{
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"event\",\"actuator\":\"plc\",\"byte\":%u,"
           "\"name\":\"%s\",\"on\":%s}",
           (unsigned)PLCA_PROTO_VER, (unsigned)byteCode, name, on ? "true" : "false");
  tcpTx(buf);
}

static void sendAlertsForNewBits(uint8_t prev, uint8_t now)
{
  uint8_t risen = (uint8_t)(now & ~prev);
  if (!risen) return;
  if (risen & BIT_GRIPPER) { sendAlert(1); plcTcpTxEvent(PLC_TX_GRIPPER_ERR, "GripperE"); }  // falla gripper / aire
  if (risen & BIT_HOLDER)  { sendAlert(2); plcTcpTxEvent(PLC_TX_HOLDER_ERR, "HolderE"); }
  if (risen & BIT_CUTTER)  { sendAlert(3); plcTcpTxEvent(PLC_TX_CUTTER_ERR, "CutterE"); }
  if (risen & (BIT_HOSE_A | BIT_HOSE_B)) sendAlert(4);
  if (risen & BIT_ENCODER) { sendAlert(5); plcTcpTxEvent(PLC_TX_ENCODER_ERR, "EncoderE"); }  // aire / manguera / cilindro
  plcTcpPushStateIfChanged();
}

static void peerTxSensorBitEvents(uint8_t prev, uint8_t now)
{
  struct { uint8_t bit; const char* field; } map[] = {
    { BIT_GRIPPER, "sensorGripper" },
    { BIT_HOLDER,  "sensorHolder" },
    { BIT_CUTTER,  "sensorCutter" },
    { BIT_HOSE_A,  "sensorHoseA" },
    { BIT_HOSE_B,  "sensorHoseB" },
    { BIT_ENCODER, "sensorTray" },
  };
  for (unsigned i = 0; i < sizeof(map) / sizeof(map[0]); i++)
  {
    const bool was = (prev & map[i].bit) != 0;
    const bool nowOn = (now & map[i].bit) != 0;
    if (was == nowOn) continue;
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"event\",\"field\":\"%s\",\"value\":%s}\n",
             map[i].field, nowOn ? "true" : "false");
    tcpTx(buf);
  }
}

static void peerTxValveIfChanged(bool& last, bool now, uint8_t byteCode, const char* name)
{
  if (now == last) return;
  last = now;
  plcTcpTxValve(byteCode, name, now);
}

static void peerTxEvents()
{
  const uint8_t mask = currentStableMask();
  if (lastPeerMask != 0xFF && mask != lastPeerMask)
    peerTxSensorBitEvents(lastPeerMask, mask);
  if (mask != lastPeerMask)
  {
    lastPeerMask = mask;
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"event\",\"field\":\"sensorMask\",\"value\":%u}\n",
             (unsigned)mask);
    tcpTx(buf);
  }
  peerTxValveIfChanged(lastPeerCutterR, stCutterR, PLC_CMD_CUTTER_R, "CutterR");
  peerTxValveIfChanged(lastPeerCutterL, stCutterL, PLC_CMD_CUTTER_L, "CutterL");
  if (stCutters != lastPeerCutters)
  {
    lastPeerCutters = stCutters;
    char buf[72];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"event\",\"field\":\"cutters\",\"value\":%s}\n",
             stCutters ? "true" : "false");
    tcpTx(buf);
  }
  peerTxValveIfChanged(lastPeerGrippers, stGrippers, PLC_CMD_GRIPPER, "Gripper");
  peerTxValveIfChanged(lastPeerHolder, stHolder, PLC_CMD_HOLDER, "Holder");
  peerTxValveIfChanged(lastPeerEncoder, stEncoder, PLC_CMD_ENCODER, "Encoder");
  peerTxValveIfChanged(lastPeerBlower, stBlower, PLC_CMD_BLOWER, "Blower");
  if (stReset != lastPeerReset)
  {
    lastPeerReset = stReset;
    char buf[72];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"event\",\"field\":\"reset\",\"value\":%s}\n",
             stReset ? "true" : "false");
    tcpTx(buf);
  }
}

static void peerPushNow(bool fullStatus = true)
{
  peerTxEvents();
  if (fullStatus)
    tcpTxStatus("status");
  lastStatusPushMs = millis();
}

static void replyPollSnapshot()
{
  sendSnapshot(currentStableMask());
  tcpTxStatus("status");
  lastStatusPushMs = millis();
}

static void blowerService()
{
  if (!blowerTimedActive) return;
  if ((millis() - blowerStartMs) < blowerHoldMs) return;
  blowerStop();
  if (tcpLinkOk()) {
    peerTxEvents();
    tcpTxStatus("status");
    plcTcpPushStateIfChanged();
    lastStatusPushMs = millis();
  }
}

static void valveFinishJob()
{
  const uint8_t flags = vpFlags;
  const uint8_t ackByte = vpAckByte;
  vpPhase = VP_IDLE;
  vpN = 0;
  vpFlags = 0;
  if ((flags & JOB_ACK) && ackByte)
    plcSendAck(ackByte, true, "");
  if (flags & (JOB_ACK | JOB_STATUS))
    peerTxEvents();
  if (flags & JOB_STATUS) {
    tcpTxStatus("status");
    lastStatusPushMs = millis();
  }
  if (flags & (JOB_ACK | JOB_STATUS))
    plcTcpPushStateIfChanged();
}

static void valveStartNext()
{
  if (vpPhase != VP_IDLE || valveJobCount == 0) return;
  ValveJob& j = valveJobs[valveJobHead];
  valveJobHead = (valveJobHead + 1) % VALVE_JOB_Q;
  valveJobCount--;
  vpN = j.n;
  for (uint8_t i = 0; i < vpN; i++)
    vpPins[i] = j.pins[i];
  vpAckByte = j.ackByte;
  vpFlags = j.flags;
  const unsigned long now = millis();
  if (lastValvePulseEndMs != 0 && (now - lastValvePulseEndMs) < VALVE_PULSE_GAP_MS) {
    vpPhase = VP_GAP;
    vpPhaseMs = lastValvePulseEndMs;
    return;
  }
  for (uint8_t i = 0; i < vpN; i++)
    valveWritePin(vpPins[i], true);
  vpPhase = VP_HIGH;
  vpPhaseMs = now;
}

static void serviceValvePulse()
{
  const unsigned long now = millis();
  if (vpPhase == VP_GAP) {
    if (lastValvePulseEndMs == 0 || (now - lastValvePulseEndMs) >= VALVE_PULSE_GAP_MS) {
      for (uint8_t i = 0; i < vpN; i++)
        valveWritePin(vpPins[i], true);
      vpPhase = VP_HIGH;
      vpPhaseMs = now;
    }
  } else if (vpPhase == VP_HIGH) {
    if ((now - vpPhaseMs) >= VALVE_PULSE_MS) {
      for (uint8_t i = 0; i < vpN; i++)
        valveWritePin(vpPins[i], false);
      lastValvePulseEndMs = now;
      valveFinishJob();
    }
  }
  if (vpPhase == VP_IDLE)
    valveStartNext();
}

// --- TCP RX / WiFi ---
static const char* jFindKey(const char* j, const char* k)
{
  char pat[40];
  snprintf(pat, sizeof(pat), "\"%s\":", k);
  const char* p = strstr(j, pat);
  if (!p) return nullptr;
  return p + strlen(pat);
}

static int jInt(const char* j, const char* k, int d)
{
  const char* p = jFindKey(j, k);
  if (!p) return d;
  while (*p == ' ') p++;
  return atoi(p);
}

static float jNum(const char* j, const char* k)
{
  const char* p = jFindKey(j, k);
  if (!p) return -1.0f;
  while (*p == ' ') p++;
  return atof(p);
}

static unsigned long blowerParseDurationMs(const char* line)
{
  float sec = jNum(line, "durationSec");
  if (sec < 0) sec = jNum(line, "blowerSec");
  if (sec < 0) sec = jNum(line, "sec");
  if (sec > 0) {
    unsigned long ms = (unsigned long)(sec * 1000.0f + 0.5f);
    if (ms < BLOWER_MIN_MS) ms = BLOWER_MIN_MS;
    if (ms > BLOWER_MAX_MS) ms = BLOWER_MAX_MS;
    return ms;
  }
  const float msVal = jNum(line, "durationMs");
  if (msVal > 0) {
    unsigned long ms = (unsigned long)(msVal + 0.5f);
    if (ms < BLOWER_MIN_MS) ms = BLOWER_MIN_MS;
    if (ms > BLOWER_MAX_MS) ms = BLOWER_MAX_MS;
    return ms;
  }
  return BLOWER_DEFAULT_SEC * 1000UL;
}

static void jStr(const char* j, const char* k, char* out, size_t n)
{
  if (!out || n == 0) return;
  out[0] = 0;
  char pat[48];
  snprintf(pat, sizeof(pat), "\"%s\":\"", k);
  const char* p = strstr(j, pat);
  if (!p) return;
  p += strlen(pat);
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < n)
    out[i++] = *p++;
  out[i] = 0;
}

static bool jBool(const char* j, const char* k, bool defVal)
{
  const char* p = jFindKey(j, k);
  if (!p) return defVal;
  while (*p == ' ') p++;
  if (strncmp(p, "true", 4) == 0 || *p == '1') return true;
  if (strncmp(p, "false", 5) == 0 || *p == '0') return false;
  return defVal;
}

static bool plcTcpIsCmdByte(uint8_t b)
{
  return (b >= PLC_CMD_CUTTER_R && b <= PLC_CMD_RESET) || b == PLC_CMD_BLOWER
      || b == PLC_CMD_HOLDER_ENCODER;
}

static bool plcTcpParseOn(const char* line, uint8_t cmdByte)
{
  if (cmdByte == PLC_CMD_RESET) return true;
  char val[16];
  jStr(line, "value", val, sizeof(val));
  if (val[0])
    return parseOnOffStr(val);
  return jBool(line, "on", jBool(line, "value", true));
}

static bool plcTcpDoByte(uint8_t cmdByte, const char* line)
{
  const char* err = "";
  bool ok = false;
  bool deferred = false;
  const bool wantOn = plcTcpParseOn(line, cmdByte);

  switch (cmdByte) {
    case PLC_CMD_CUTTER_R:
      ok = valveArm(stCutterR, PIN_OUT_CUTTER_R, wantOn, cmdByte, JOB_ACK, deferred);
      break;
    case PLC_CMD_CUTTER_L:
      ok = valveArm(stCutterL, PIN_OUT_CUTTER_L, wantOn, cmdByte, JOB_ACK, deferred);
      break;
    case PLC_CMD_GRIPPER:
      ok = valveArm(stGrippers, PIN_OUT_GRIPPERS, wantOn, cmdByte, JOB_ACK, deferred);
      break;
    case PLC_CMD_HOLDER:
      ok = valveArm(stHolder, PIN_OUT_HOLDER, wantOn, cmdByte, JOB_ACK, deferred);
      break;
    case PLC_CMD_ENCODER:
      ok = valveArm(stEncoder, PIN_OUT_ENCODER, wantOn, cmdByte, JOB_ACK, deferred);
      break;
    case PLC_CMD_HOLDER_ENCODER: {
      const bool needH = (stHolder != wantOn);
      const bool needE = (stEncoder != wantOn);
      if (!needH && !needE) {
        ok = true;
        break;
      }
      uint8_t pins[2];
      uint8_t np = 0;
      if (needH) pins[np++] = PIN_OUT_HOLDER;
      if (needE) pins[np++] = PIN_OUT_ENCODER;
      if (!valveEnqueue(pins, np, cmdByte, JOB_ACK)) break;
      if (needH) stHolder = wantOn;
      if (needE) stEncoder = wantOn;
      deferred = true;
      ok = true;
      break;
    }
    case PLC_CMD_BLOWER:
      blowerSet(wantOn, wantOn ? blowerParseDurationMs(line) : 0);
      ok = true;
      break;
    case PLC_CMD_RESET:
      ok = plcDoHardwareReset(cmdByte, JOB_ACK, deferred);
      break;
    default:
      if (cmdByte >= PLC_TX_CUTTER_ERR && cmdByte <= PLC_TX_ENCODER_ERR)
        err = "byte error es solo TX (0x1F-0x22)";
      else
        err = "byte/cmd PLC desconocido";
      break;
  }

  if (!ok && !err[0])
    err = "PLC ocupado";
  if (ok)
    plcTcpStopPending = false;
  if (!ok) {
    plcSendAck(cmdByte, false, err);
    return false;
  }
  // El ACK del pulso sale al bajar el GPIO. Si no hubo pulso, se confirma ya.
  if (!deferred) {
    plcSendAck(cmdByte, true, "");
    peerTxEvents();
    plcTcpPushStateIfChanged();
  }
  return true;
}

static uint8_t plcTcpResolveCmdByte(const char* line)
{
  const int b = jInt(line, "byte", -1);
  if (b > 0) return (uint8_t)b;

  char cmd[24];
  jStr(line, "command", cmd, sizeof(cmd));
  if (nameEq(cmd, "CutterR")) return PLC_CMD_CUTTER_R;
  if (nameEq(cmd, "CutterL")) return PLC_CMD_CUTTER_L;
  if (nameEq(cmd, "Gripper")) return PLC_CMD_GRIPPER;
  if (nameEq(cmd, "Holder")) return PLC_CMD_HOLDER;
  if (nameEq(cmd, "Encoder")) return PLC_CMD_ENCODER;
  if (nameEq(cmd, "ResetPLC") || nameEq(cmd, "reset")) return PLC_CMD_RESET;
  if (nameEq(cmd, "Blower")) return PLC_CMD_BLOWER;
  return 0;
}

static void tcpHandleLine(const char* line)
{
  if (!line || !line[0]) return;
  if (!strstr(line, "\"type\":\"command\"")) return;

  const int rawByte = jInt(line, "byte", -1);
  if (rawByte > 0) {
    const uint8_t cmdByte = (uint8_t)rawByte;
    if (plcTcpIsCmdByte(cmdByte)) {
      plcTcpDoByte(cmdByte, line);
      return;
    }
    if (cmdByte >= PLC_TX_CUTTER_ERR && cmdByte <= PLC_TX_ENCODER_ERR) {
      plcSendAck(cmdByte, false, "byte error es solo TX (0x1F-0x22)");
      return;
    }
    if (cmdByte >= PLC_TX_INIT && cmdByte <= PLC_TX_RETURN) {
      const uint8_t st = plcTcpResolveStateByte();
      plcTcpTxState(st, plcTcpStateName(st));
      return;
    }
    plcSendAck(cmdByte, false, "byte/cmd PLC desconocido");
    return;
  }

  const uint8_t cmdByte = plcTcpResolveCmdByte(line);
  if (cmdByte && plcTcpIsCmdByte(cmdByte)) {
    plcTcpDoByte(cmdByte, line);
    return;
  }

  char cmd[24];
  char val[24];
  char out[24];
  jStr(line, "command", cmd, sizeof(cmd));
  jStr(line, "value", val, sizeof(val));
  jStr(line, "out", out, sizeof(out));

  if (nameEq(cmd, "setOut") || nameEq(cmd, "setPlc") || nameEq(cmd, "plcSet"))
  {
    const char* outName = out[0] ? out : val;
    const bool wantOn = out[0] ? parseOnOffStr(val) : (jInt(line, "value", 0) != 0);
    bool deferred = false;
    if (nameEq(outName, "BLOWER"))
      blowerSet(wantOn, wantOn ? blowerParseDurationMs(line) : 0);
    else if (!setPlcOutputByName(outName, wantOn, deferred))
      return;
    if (!deferred) {
      peerTxEvents();
      tcpTxStatus("status");
      plcTcpPushStateIfChanged();
      lastStatusPushMs = millis();
    }
    return;
  }
  if (nameEq(cmd, "allOff"))
  {
    const bool alreadyHome = !stCutterR && !stCutterL && !stGrippers
                             && !stEncoder && !stBlower && !stReset && !stHolder;
    bool queued = false;
    if (!alreadyHome) {
      queued = plcGoHomePulse();
      plcTcpStopPending = true;
      plcTcpTxState(PLC_TX_STOP, "StoprState");
    }
    if (!queued) {
      peerTxEvents();
      tcpTxStatus("status");
      lastStatusPushMs = millis();
    }
    return;
  }
  if (nameEq(cmd, "poll"))
  {
    // Solo bajo demanda / debug — no usar como heartbeat (regla C1).
    replyPollSnapshot();
    return;
  }
  if (nameEq(cmd, "ping"))
  {
    // Keepalive de enlace: ack mínimo, sin snapshot de sensores.
    tcpTx("{\"type\":\"pong\"}");
    return;
  }
}

static bool plcLineBlocks(const char* line)
{
  if (!line || !strstr(line, "\"type\":\"command\"")) return false;
  const int b = jInt(line, "byte", -1);
  if (b > 0 && plcTcpIsCmdByte((uint8_t)b)) return true;
  char cmd[24];
  jStr(line, "command", cmd, sizeof(cmd));
  return nameEq(cmd, "setOut") || nameEq(cmd, "setPlc") || nameEq(cmd, "plcSet")
      || nameEq(cmd, "allOff") || nameEq(cmd, "CutterR") || nameEq(cmd, "CutterL")
      || nameEq(cmd, "Gripper") || nameEq(cmd, "Holder") || nameEq(cmd, "Encoder")
      || nameEq(cmd, "ResetPLC") || nameEq(cmd, "reset");
}

static bool lineEnqueue(const char* line)
{
  if (!line || lineQn >= LINE_Q) return false;
  if (strlen(line) > 238) return false;
  char* dst = lineQ[(lineQHead + lineQn) % LINE_Q];
  strncpy(dst, line, 239);
  dst[239] = 0;
  lineQn++;
  return true;
}

static void tcpOnLine(const char* line)
{
  if (!line || !line[0]) return;
  // Ping/poll se responden ya. Válvulas esperan a que el pulso anterior termine
  // para no encimar KEEP ni congelar el ACK.
  if (plcLineBlocks(line)) {
    if (!lineEnqueue(line)) {
      Serial.println("[TCP] cola de comandos llena");
      const int b = jInt(line, "byte", -1);
      if (b > 0)
        plcSendAck((uint8_t)b, false, "PLC ocupado");
    }
    return;
  }
  tcpHandleLine(line);
}

static void plcDispatchHeld()
{
  if (vpPhase != VP_IDLE || valveJobCount > 0 || !lineQn) return;
  char line[240];
  memcpy(line, lineQ[lineQHead], sizeof(line));
  lineQHead = (lineQHead + 1) % LINE_Q;
  lineQn--;
  tcpHandleLine(line);
}

static void tcpRxDrain()
{
  while (tcpClient.available())
  {
    char c = (char)tcpClient.read();
    if (c == '\n' || c == '\r')
    {
      if (tcpRxLen)
      {
        tcpRxLine[tcpRxLen] = 0;
        tcpOnLine(tcpRxLine);
        tcpRxLen = 0;
      }
    }
    else if (tcpRxLen < sizeof(tcpRxLine) - 1)
      tcpRxLine[tcpRxLen++] = c;
  }
}

static void tcpOnClientAccepted()
{
  tcpClient.setNoDelay(true);
  tcpRxLen = 0;
  lastPeerMask = 0xFF;
  lastPeerCutterR = !stCutterR;
  lastPeerCutterL = !stCutterL;
  lastPeerCutters = !stCutters;
  lastPeerGrippers = !stGrippers;
  lastPeerHolder = !stHolder;
  lastPeerEncoder = !stEncoder;
  lastPeerBlower = !stBlower;
  lastPeerReset = !stReset;
  plcTcpLastStateByte = 0;
  plcTcpNeedInit = true;
  plcTcpStopPending = false;
  peerLinkOk = true;
  valveDropPending();
  ackReset();
  txHoldReset();
  char hello[80];
  snprintf(hello, sizeof(hello),
           "{\"ver\":%u,\"type\":\"hello\",\"role\":\"plca\"}\n",
           (unsigned)PLCA_PROTO_VER);
  tcpTx(hello);
  peerPushNow(true);
  Serial.printf("[TCP] On desde %s\n", tcpClient.remoteIP().toString().c_str());
  tcpWasConnected = true;
}

static bool tcpAcceptIncoming()
{
  if (!tcpServicesUp || !tcpServer.hasClient()) return false;
  WiFiClient incoming = tcpServer.available();
  if (!incoming) return false;

  // Un solo maestro. Zombies WiFi a menudo quedan con connected()==false
  // (o true) y bloquean el pool LWIP → HMI “a veces conecta”.
  if (tcpClient)
    tcpClient.stop();
  tcpClient = incoming;
  tcpOnClientAccepted();
  return true;
}

static bool arduinoOtaStarted = false;
static volatile bool arduinoOtaInProgress = false;

static void plcOtaSafeStop()
{
  blowerStop();
  valveAbortAll();
}

static void serviceArduinoOta(uint8_t passes = 4)
{
  if (!arduinoOtaStarted) return;
  for (uint8_t i = 0; i < passes; i++)
  {
    ArduinoOTA.handle();
    yield();
  }
}

static void ensureArduinoOta()
{
  if (arduinoOtaStarted || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]()
  {
    arduinoOtaInProgress = true;
    plcOtaSafeStop();
    Serial.println("[OTA] inicio (red local)");
  });
  ArduinoOTA.onEnd([]()
  {
    arduinoOtaInProgress = false;
    Serial.println("\n[OTA] listo (red local)");
  });
  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total)
  {
    if (total) Serial.printf("[OTA] %u%%\r", (prog * 100U) / total);
  });
  ArduinoOTA.onError([](ota_error_t err)
  {
    arduinoOtaInProgress = false;
    Serial.printf("[OTA] error %u\n", err);
  });
  ArduinoOTA.begin();
  arduinoOtaStarted = true;
  Serial.printf("[OTA] red local listo hostname=%s (IDE puerto de red)\n", OTA_HOSTNAME);
}

static void tcpEnsureServices()
{
  if (tcpServicesUp || WiFi.status() != WL_CONNECTED) return;
  IPAddress ip = WiFi.localIP();
  if (ip == IPAddress(0, 0, 0, 0)) return;
  tcpServer.end();
  delay(20);
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  tcpServicesUp = true;
  wifiReady = true;
  ensureArduinoOta();
  Serial.printf("[TCP] Servicios On %s:%u\n", ip.toString().c_str(), PLCA_TCP_PORT);
}

static void tcpStopServices()
{
  if (tcpClient.connected()) tcpClient.stop();
  peerLinkOk = false;
  tcpWasConnected = false;
  if (!tcpServicesUp) return;
  tcpServer.end();
  tcpServicesUp = false;
  wifiReady = false;
  Serial.println("[TCP] Servicios Off");
}

static unsigned long wifiDownSinceMs = 0;
static const unsigned long WIFI_DOWN_GRACE_MS = 2500;

static void wifiLogStatus()
{
  const bool on = WiFi.status() == WL_CONNECTED;
  if (on) {
    wifiDownSinceMs = 0;
    if (staWasConnected) return;
    staWasConnected = true;
    wifiConnectStartedMs = 0;
    Serial.printf("[WiFi] On %s\n", WiFi.localIP().toString().c_str());
    tcpEnsureServices();
    return;
  }
  // Un WL_CONNECTION_LOST de un instante no debe tumbar el :8766.
  // Antes eso reiniciaba el radio y la HMI latcheaba E066.
  if (!staWasConnected) return;
  if (wifiDownSinceMs == 0) {
    wifiDownSinceMs = millis();
    Serial.println("[WiFi] blip — esperando auto-reconexion");
    return;
  }
  if (millis() - wifiDownSinceMs < WIFI_DOWN_GRACE_MS) return;
  staWasConnected = false;
  wifiDownSinceMs = 0;
  tcpStopServices();
  Serial.println("[WiFi] Off");
}

static bool wifiNeedsRetry()
{
  wl_status_t s = WiFi.status();
  if (s == WL_CONNECTED) return false;
  if (wifiDownSinceMs && (millis() - wifiDownSinceMs) < WIFI_DOWN_GRACE_MS)
    return false;
  if (s == WL_NO_SSID_AVAIL || s == WL_CONNECT_FAILED || s == WL_CONNECTION_LOST) return true;
  return wifiConnectStartedMs && (millis() - wifiConnectStartedMs > 15000);
}

static void wifiRetry()
{
  tcpStopServices();
  wifiConnectStartedMs = millis();
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.config(STA_IP, STA_GW, STA_MASK, STA_GW);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("[WiFi] Reintentando STA...");
}

static void serviceTcp()
{
  wifiLogStatus();
  if (wifiNeedsRetry())
  {
    static unsigned long lastRetryMs = 0;
    if (millis() - lastRetryMs >= 8000)
    {
      lastRetryMs = millis();
      wifiRetry();
    }
  }
  tcpEnsureServices();

  if (tcpAcceptIncoming())
    return;

  if (!tcpLinkOk())
  {
    if (tcpWasConnected)
    {
      Serial.println("[TCP] Off");
      tcpWasConnected = false;
      peerLinkOk = false;
      tcpRxLen = 0;
      valveDropPending();
      ackReset();
      txHoldReset();
    }
    return;
  }

  tcpWasConnected = true;
  peerLinkOk = true;
  tcpRxDrain();
  plcTcpPollStateEvents();

  const unsigned long iv = STATUS_PUSH_MS;
  if (millis() - lastStatusPushMs >= iv)
  {
    peerTxEvents();
    tcpTxStatus("status");
    plcTcpPushStateIfChanged();
    lastStatusPushMs = millis();
  }
}

// --- Setup / Loop ---
void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("PLCA esclavo — sensores + válvulas — 10.10.32.51:8766");

  pinMode(PIN_CUTTER, INPUT_PULLUP);
  pinMode(PIN_GRIPPER, INPUT_PULLUP);
  pinMode(PIN_HOLDER, INPUT_PULLUP);
  // GPIO34: input-only ESP32, sin pull-up interno (INPUT_PULLUP no aplica).
  // Requiere pull-up externa en hardware; sin ella el pin puede flotar.
  pinMode(PIN_ENCODER, INPUT);

  pinMode(PIN_OUT_CUTTER_R, OUTPUT);
  pinMode(PIN_OUT_CUTTER_L, OUTPUT);
  pinMode(PIN_OUT_GRIPPERS, OUTPUT);
  pinMode(PIN_OUT_HOLDER, OUTPUT);
  pinMode(PIN_OUT_ENCODER, OUTPUT);
  pinMode(PIN_OUT_BLOWER, OUTPUT);
  pinMode(PIN_OUT_RESET, OUTPUT);
  valveIdleAll();
  // Reposo: todas OFF (sin pulso). Holder no se auto-activa; rutina/manual.
  stCutterR = stCutterL = stGrippers = stHolder = stEncoder = stBlower = stReset = false;
  syncCuttersFlag();
  Serial.printf("Valvulas: modo pulso %lums (on=pulso, off=pulso); boot=all OFF\n", VALVE_PULSE_MS);

  wifiConnectStartedMs = millis();
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  delay(100);
  if (!WiFi.config(STA_IP, STA_GW, STA_MASK, STA_GW))
    Serial.println("WiFi.config() fallo.");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Buscando router \"%s\"...\n", WIFI_SSID);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart < WIFI_CONNECT_TIMEOUT_MS))
  {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  wifiLogStatus();
  tcpEnsureServices();
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("WiFi pendiente — reintentos en loop");
  bootGraceStartMs = millis();
  Serial.printf("Sensor boot grace %lums desde ahora\n", BOOT_GRACE_MS);
  Serial.printf("Firmware v%s · OTA red local hostname=%s\n", FW_VERSION, OTA_HOSTNAME);
}

static void plcHeapWatch()
{
  static unsigned long lastMs = 0;
  static uint32_t minHeap = 0xFFFFFFFF;
  const uint32_t heap = ESP.getFreeHeap();
  if (heap < minHeap) minHeap = heap;
  if (millis() - lastMs < 60000) return;
  lastMs = millis();
  Serial.printf("[PLC] heap libre=%u min=%u\n", heap, minHeap);
}

void loop()
{
  if (arduinoOtaInProgress)
  {
    serviceArduinoOta(24);
    serviceTcp();
    yield();
    return;
  }

  tcpTxPump();
  serviceValvePulse();
  plcDispatchHeld();
  serviceValvePulse();
  tcpTxPump();
  plcHeapWatch();

  serviceTcp();
  if (tcpLinkOk())
    tcpRxDrain();
  serviceArduinoOta(4);
  blowerService();
  tcpTxPump();
  serviceValvePulse();
  plcDispatchHeld();
  serviceValvePulse();

  unsigned long now = millis();
  uint8_t raw = readSensorBitmask();
  uint8_t stable = lastBitmask;
  bool changed = updateDebouncedMask(raw, stable, now);

  if (!changed)
  {
    if (tcpLinkOk() && lastBitmask != 0xFF
        && (now - lastSnapshotTxMs) >= SNAPSHOT_HEARTBEAT_MS)
      sendSnapshot(currentStableMask());
    yield();
    return;
  }

  // Baseline (lastBitmask==0xFF): solo fijar máscara; no tratar bits presentes
  // como flanco rising (E050/E047–E049 fantasma al primer sample / HMI connect).
  const bool establishingBaseline = (lastBitmask == 0xFF);
  uint8_t previousMask = establishingBaseline ? 0 : lastBitmask;
  if (tcpLinkOk())
  {
    sendSnapshot(stable);
    if (!establishingBaseline && !plcInBootGrace())
      sendAlertsForNewBits(previousMask, stable);
    peerTxEvents();
    tcpTxStatus("status");
    plcTcpPushStateIfChanged();
    lastStatusPushMs = millis();
  }
  else
    Serial.println("TCP offline — sensores/válvulas solo locales");

  lastBitmask = stable;
  serviceTcp();
}
