// PLCA — sensores + válvulas por pulso. Ver Config.h (IP, TCP :8766, JSON+newline).

#include <WiFi.h>
#include <ArduinoOTA.h>
#include "../common/EspLink.h"
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
EspRxLine tcpRx = {tcpRxLine, sizeof(tcpRxLine), 0, false};
static EspSta staLink;

static bool lastPeerCutterR = false;
static bool lastPeerCutterL = false;
static bool lastPeerGrippers = false;
static bool lastPeerHolder = false;
static bool lastPeerEncoder = false;
static bool lastPeerBlower = false;
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
static void plcSendAck(uint8_t cmdByte, bool ok, const char* message, uint16_t reqId);
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
  uint16_t reqId;
  unsigned long enqueuedMs;
};

static ValveJob valveJobs[VALVE_JOB_Q];
static uint8_t valveJobHead = 0;
static uint8_t valveJobCount = 0;
static unsigned long lastValvePulseEndMs = 0;

static char lineQ[LINE_Q][240];
static unsigned long lineQMs[LINE_Q];
static uint8_t lineQHead = 0;
static uint8_t lineQn = 0;

enum : uint8_t { VP_IDLE = 0, VP_GAP = 1, VP_HIGH = 2 };
static uint8_t vpPhase = VP_IDLE;
static uint8_t vpPins[4];
static uint8_t vpN = 0;
static uint8_t vpAckByte = 0;
static uint8_t vpFlags = 0;
static uint16_t vpReqId = 0;
static unsigned long vpPhaseMs = 0;
static unsigned long vpEnqueuedMs = 0;
// id del comando que se está interpretando. El pulso lo copia a su job.
static uint16_t tcpReqId = 0;
// millis() del RX/dispatch del comando actual (latencia end-to-end ACK).
static unsigned long tcpReqMs = 0;

static void ackTraceDebt(const char* tag, uint8_t byteCode, uint16_t reqId,
                         unsigned long ageMs, const char* extra);

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
  j.reqId = tcpReqId;
  j.enqueuedMs = tcpReqMs ? tcpReqMs : millis();
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
      if (!valveEnqueue(pins, np, PLC_CMD_CUTTERS, JOB_ACK | JOB_STATUS)) return false;
      if (needR) stCutterR = state;
      if (needL) stCutterL = state;
      syncCuttersFlag();
      deferred = true;
#if PLC_ACK_TRACE
      ackTraceDebt("defer", PLC_CMD_CUTTERS, tcpReqId, 0, "setOut CUTTERS");
#endif
    } else {
#if PLC_ACK_TRACE
      ackTraceDebt("imm", PLC_CMD_CUTTERS, tcpReqId, 0, "setOut ya en estado");
#endif
      plcSendAck(PLC_CMD_CUTTERS, true, "", tcpReqId);
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

// Un write corto no es un enlace muerto. El sufijo queda en txHold
// y tcpTxPump lo reintenta. stop() solo si connected() ya es falso.
static char txHoldBuf[700];
static EspTcpHold txHold = {txHoldBuf, sizeof(txHoldBuf), 0};

// Un ACK por comando encolado, por pulso en curso y uno libre para el rechazo.
// No es un aumento ciego: es el tope de órdenes que el PLC ya puede aceptar.
static const uint8_t ACK_Q = LINE_Q + VALVE_JOB_Q + 2;
static char ackQ[ACK_Q][192];
static uint8_t ackQHead = 0;
static uint8_t ackQn = 0;
// El más nuevo que no cupo en la cola. tcpTxPump lo mete cuando se libera un hueco.
static char ackSpill[192];
static bool ackSpillOn = false;

#if PLC_ACK_TRACE
static void ackTraceDebt(const char* tag, uint8_t byteCode, uint16_t reqId,
                         unsigned long ageMs, const char* extra)
{
  Serial.printf(
      "[ACK] %s byte=0x%02X id=%u age=%lums lineQ=%u jobQ=%u ackQ=%u spill=%u "
      "vp=%u hold=%u%s%s\n",
      tag ? tag : "?",
      (unsigned)byteCode,
      (unsigned)reqId,
      (unsigned long)ageMs,
      (unsigned)lineQn,
      (unsigned)valveJobCount,
      (unsigned)ackQn,
      ackSpillOn ? 1u : 0u,
      (unsigned)vpPhase,
      (unsigned)txHold.len,
      (extra && extra[0]) ? " · " : "",
      (extra && extra[0]) ? extra : "");
}
#else
static void ackTraceDebt(const char*, uint8_t, uint16_t, unsigned long, const char*) {}
#endif

static void ackReset()
{
  ackQHead = 0;
  ackQn = 0;
  ackSpillOn = false;
}

static void tcpNoteDead()
{
  espTcpStop(tcpClient);
  peerLinkOk = false;
  espTcpHoldClear(txHold);
  ackReset();
}

static bool tcpTx(const char* m)
{
  if (!m || !m[0] || txHold.len || !tcpLinkOk()) return false;
  if (!espTcpHoldPut(txHold, m)) return false;
  if (espTcpHoldFlush(tcpClient, txHold)) return true;
  if (!tcpClient.connected()) tcpNoteDead();
  return false;
}

static bool ackCopyLine(char* dst, size_t cap, const char* line)
{
  if (!line || !line[0]) return false;
  size_t n = strlen(line);
  if (n + 2 >= cap) return false;
  memcpy(dst, line, n);
  if (line[n - 1] != '\n') dst[n++] = '\n';
  dst[n] = 0;
  return true;
}

static bool ackStore(const char* line)
{
  if (!line || !line[0] || ackQn >= ACK_Q) return false;
  char* dst = ackQ[(ackQHead + ackQn) % ACK_Q];
  if (!ackCopyLine(dst, sizeof(ackQ[0]), line)) return false;
  ackQn++;
  return true;
}

// El parcial vive solo en txHold. Otro TX no puede intercalarse.
// Si el hold está ocupado, la línea completa entra a la cola. El
// spill guarda el más nuevo si la cola ya está llena; no se descarta.
static void ackPush(const char* line)
{
  if (!line || !line[0] || !tcpLinkOk()) {
#if PLC_ACK_TRACE
    if (line && line[0])
      Serial.println("[ACK] push drop · TCP off");
#endif
    return;
  }
  if (!txHold.len && espTcpHoldPut(txHold, line)) {
    if (!espTcpHoldFlush(tcpClient, txHold) && !tcpClient.connected())
      tcpNoteDead();
#if PLC_ACK_TRACE
    Serial.printf("[ACK] tx path=hold%s\n",
                  txHold.len ? " (parcial)" : " (flush)");
#endif
    return;
  }
  if (ackStore(line)) {
#if PLC_ACK_TRACE
    Serial.printf("[ACK] tx path=queue ackQ=%u\n", (unsigned)ackQn);
#endif
    return;
  }
  if (!ackSpillOn && ackCopyLine(ackSpill, sizeof(ackSpill), line)) {
    ackSpillOn = true;
#if PLC_ACK_TRACE
    Serial.println("[ACK] tx path=spill");
#endif
    return;
  }
  Serial.println("[TCP] ACK no enviado");
#if PLC_ACK_TRACE
  Serial.printf("[ACK] DROP ackQ=%u hold=%u\n",
                (unsigned)ackQn, (unsigned)txHold.len);
#endif
}

static void tcpTxPump()
{
  if (!tcpLinkOk()) return;
  if (txHold.len) {
    if (!espTcpHoldFlush(tcpClient, txHold)) {
      if (!tcpClient.connected()) tcpNoteDead();
      return;
    }
  }
  while (ackQn) {
    char* slot = ackQ[ackQHead];
    size_t n = strlen(slot);
    if (!espTcpWriteAll(tcpClient, slot, &n)) {
      if (!tcpClient.connected()) tcpNoteDead();
      return;
    }
    ackQHead = (ackQHead + 1) % ACK_Q;
    ackQn--;
  }
  if (ackSpillOn && ackStore(ackSpill))
    ackSpillOn = false;
}

static uint8_t ackDebt()
{
  uint8_t n = (uint8_t)(lineQn + valveJobCount + ackQn + (ackSpillOn ? 1 : 0));
  if (vpPhase != VP_IDLE && (vpFlags & JOB_ACK) && vpAckByte)
    n++;
  return n;
}

static bool ackCanAcceptCommand()
{
  return (uint16_t)ackDebt() + 1 < ACK_Q;
}

static void plcSendAck(uint8_t cmdByte, bool ok, const char* message, uint16_t reqId)
{
  const unsigned long now = millis();
  const unsigned long ageMs = (tcpReqMs && now >= tcpReqMs) ? (now - tcpReqMs) : 0;
  char buf[192];
  if (reqId) {
    snprintf(buf, sizeof(buf),
             "{\"ver\":%u,\"type\":\"ack\",\"actuator\":\"plc\",\"byte\":%u,"
             "\"id\":%u,\"ok\":%s,\"message\":\"%s\"}\n",
             (unsigned)PLCA_PROTO_VER, (unsigned)cmdByte, (unsigned)reqId,
             ok ? "true" : "false", message ? message : "");
  } else {
    snprintf(buf, sizeof(buf),
             "{\"ver\":%u,\"type\":\"ack\",\"actuator\":\"plc\",\"byte\":%u,"
             "\"ok\":%s,\"message\":\"%s\"}\n",
             (unsigned)PLCA_PROTO_VER, (unsigned)cmdByte,
             ok ? "true" : "false", message ? message : "");
  }
  ackTraceDebt(ok ? "send_ok" : "send_ng", cmdByte, reqId, ageMs,
               (message && message[0]) ? message : "");
  if (ageMs >= ACK_SLOW_WARN_MS)
    ackTraceDebt("SLOW", cmdByte, reqId, ageMs, "cerca del watch HMI");
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

static bool plcTcpTxState(uint8_t byteCode, const char* name)
{
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"state\",\"actuator\":\"plc\",\"byte\":%u,\"name\":\"%s\"}",
           (unsigned)PLCA_PROTO_VER, (unsigned)byteCode, name);
  if (!tcpTx(buf)) return false;
  plcTcpLastStateByte = byteCode;
  return true;
}

static void plcTcpPushStateIfChanged()
{
  const uint8_t st = plcTcpResolveStateByte();
  if (st == plcTcpLastStateByte) return;
  if (!plcTcpTxState(st, plcTcpStateName(st))) return;
  if (st == PLC_TX_RETURN)
    plcTcpStopPending = false;
}

static void plcTcpPollStateEvents()
{
  if (!tcpLinkOk()) return;

  if (plcTcpNeedInit && plcTcpTxState(PLC_TX_INIT, "InitState"))
    plcTcpNeedInit = false;

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
      "{\"ver\":%u,\"type\":\"%s\",\"actuator\":\"plc\",\"role\":\"plc\",\"mask\":%u,\"seq\":%u,"
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

// on < 0: evento sin nivel. on 0/1: el mismo evento, con el nivel que cambió.
static bool plcTcpTxEvent(uint8_t byteCode, const char* name, int on = -1)
{
  char buf[192];
  if (on < 0) {
    snprintf(buf, sizeof(buf),
             "{\"ver\":%u,\"type\":\"event\",\"actuator\":\"plc\",\"byte\":%u,\"name\":\"%s\"}",
             (unsigned)PLCA_PROTO_VER, (unsigned)byteCode, name);
  } else {
    snprintf(buf, sizeof(buf),
             "{\"ver\":%u,\"type\":\"event\",\"actuator\":\"plc\",\"byte\":%u,"
             "\"name\":\"%s\",\"on\":%s}",
             (unsigned)PLCA_PROTO_VER, (unsigned)byteCode, name, on ? "true" : "false");
  }
  return tcpTx(buf);
}

static bool peerTxSensorBitEvents(uint8_t prev, uint8_t now)
{
  struct { uint8_t bit; uint8_t byteCode; const char* name; } map[] = {
    { BIT_GRIPPER, PLC_TX_GRIPPER_ERR, "GripperE" },
    { BIT_HOLDER,  PLC_TX_HOLDER_ERR,  "HolderE" },
    { BIT_CUTTER,  PLC_TX_CUTTER_ERR,  "CutterE" },
    { BIT_ENCODER, PLC_TX_ENCODER_ERR, "EncoderE" },
  };
  bool ok = true;
  for (unsigned i = 0; i < sizeof(map) / sizeof(map[0]); i++)
  {
    const bool was = (prev & map[i].bit) != 0;
    const bool nowOn = (now & map[i].bit) != 0;
    if (was == nowOn) continue;
    if (!plcTcpTxEvent(map[i].byteCode, map[i].name, nowOn ? 1 : 0))
      ok = false;
  }
  return ok;
}

static void peerTxValveIfChanged(bool& last, bool now, uint8_t byteCode, const char* name)
{
  if (now == last) return;
  if (!plcTcpTxEvent(byteCode, name, now ? 1 : 0)) return;
  last = now;
}

static void peerTxEvents()
{
  const uint8_t mask = currentStableMask();
  if (lastPeerMask != 0xFF && mask != lastPeerMask) {
    if (!peerTxSensorBitEvents(lastPeerMask, mask))
      return;
  }
  if (mask != lastPeerMask)
    lastPeerMask = mask;
  peerTxValveIfChanged(lastPeerCutterR, stCutterR, PLC_CMD_CUTTER_R, "CutterR");
  peerTxValveIfChanged(lastPeerCutterL, stCutterL, PLC_CMD_CUTTER_L, "CutterL");
  peerTxValveIfChanged(lastPeerGrippers, stGrippers, PLC_CMD_GRIPPER, "Gripper");
  peerTxValveIfChanged(lastPeerHolder, stHolder, PLC_CMD_HOLDER, "Holder");
  peerTxValveIfChanged(lastPeerEncoder, stEncoder, PLC_CMD_ENCODER, "Encoder");
  peerTxValveIfChanged(lastPeerBlower, stBlower, PLC_CMD_BLOWER, "Blower");
}

static void peerPushNow(bool fullStatus = true)
{
  peerTxEvents();
  if (fullStatus)
    tcpTxStatus("status");
}

static void replyStatus()
{
  tcpTxStatus("status");
}

static void blowerService()
{
  if (!blowerTimedActive) return;
  if ((millis() - blowerStartMs) < blowerHoldMs) return;
  blowerStop();
  if (tcpLinkOk()) {
    peerTxEvents();
    plcTcpPushStateIfChanged();
  }
}

static void valveFinishJob()
{
  const uint8_t flags = vpFlags;
  const uint8_t ackByte = vpAckByte;
  const uint16_t reqId = vpReqId;
  const unsigned long enqMs = vpEnqueuedMs;
  vpPhase = VP_IDLE;
  vpN = 0;
  vpFlags = 0;
  vpEnqueuedMs = 0;
  if ((flags & JOB_ACK) && ackByte) {
    // Latencia desde el RX del comando (no desde el inicio del HIGH).
    tcpReqMs = enqMs ? enqMs : tcpReqMs;
#if PLC_ACK_TRACE
    const unsigned long now = millis();
    const unsigned long pulseAge = (enqMs && now >= enqMs) ? (now - enqMs) : 0;
    ackTraceDebt("pulse_end", ackByte, reqId, pulseAge, "ACK al bajar GPIO");
#endif
    plcSendAck(ackByte, true, "", reqId);
    tcpReqMs = 0;
  }
  if (flags & (JOB_ACK | JOB_STATUS)) {
    peerTxEvents();
    plcTcpPushStateIfChanged();
  }
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
  vpReqId = j.reqId;
  vpEnqueuedMs = j.enqueuedMs;
  const unsigned long now = millis();
  if (lastValvePulseEndMs != 0 && (now - lastValvePulseEndMs) < VALVE_PULSE_GAP_MS) {
    vpPhase = VP_GAP;
    vpPhaseMs = lastValvePulseEndMs;
#if PLC_ACK_TRACE
    if ((j.flags & JOB_ACK) && j.ackByte) {
      const unsigned long waitMs = (j.enqueuedMs && now >= j.enqueuedMs)
                                       ? (now - j.enqueuedMs) : 0;
      ackTraceDebt("pulse_gap", j.ackByte, j.reqId, waitMs, "espera hueco KEEP");
    }
#endif
    return;
  }
  for (uint8_t i = 0; i < vpN; i++)
    valveWritePin(vpPins[i], true);
  vpPhase = VP_HIGH;
  vpPhaseMs = now;
#if PLC_ACK_TRACE
  if ((j.flags & JOB_ACK) && j.ackByte) {
    const unsigned long waitMs = (j.enqueuedMs && now >= j.enqueuedMs)
                                     ? (now - j.enqueuedMs) : 0;
    ackTraceDebt("pulse_hi", j.ackByte, j.reqId, waitMs, "KEEP HIGH");
  }
#endif
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

// Traduce el JSON a ms. El rango lo aplica blowerStart().
static unsigned long blowerParseDurationMs(const char* line)
{
  float sec = jNum(line, "durationSec");
  if (sec < 0) sec = jNum(line, "blowerSec");
  if (sec < 0) sec = jNum(line, "sec");
  if (sec > 0)
    return (unsigned long)(sec * 1000.0f + 0.5f);
  const float msVal = jNum(line, "durationMs");
  if (msVal > 0)
    return (unsigned long)(msVal + 0.5f);
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
#if PLC_ACK_TRACE
  {
    const unsigned long ageMs = (tcpReqMs && millis() >= tcpReqMs)
                                    ? (millis() - tcpReqMs) : 0;
    char note[40];
    snprintf(note, sizeof(note), "want=%s", wantOn ? "ON" : "OFF");
    ackTraceDebt("rx_cmd", cmdByte, tcpReqId, ageMs, note);
  }
#endif

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
#if PLC_ACK_TRACE
    ackTraceDebt("reject", cmdByte, tcpReqId, 0, err);
#endif
    plcSendAck(cmdByte, false, err, tcpReqId);
    return false;
  }
  // El ACK del pulso sale al bajar el GPIO. Si no hubo pulso, se confirma ya.
  if (!deferred) {
#if PLC_ACK_TRACE
    ackTraceDebt("imm", cmdByte, tcpReqId, 0, "sin pulso");
#endif
    plcSendAck(cmdByte, true, "", tcpReqId);
    peerTxEvents();
    plcTcpPushStateIfChanged();
  } else {
#if PLC_ACK_TRACE
    ackTraceDebt("defer", cmdByte, tcpReqId, 0, "ACK tras pulso");
#endif
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
  if (!tcpReqMs)
    tcpReqMs = millis();
  tcpReqId = (uint16_t)jInt(line, "id", 0);

  const int rawByte = jInt(line, "byte", -1);
  if (rawByte > 0) {
    const uint8_t cmdByte = (uint8_t)rawByte;
    if (plcTcpIsCmdByte(cmdByte)) {
      plcTcpDoByte(cmdByte, line);
      return;
    }
    if (cmdByte >= PLC_TX_CUTTER_ERR && cmdByte <= PLC_TX_ENCODER_ERR) {
      plcSendAck(cmdByte, false, "byte error es solo TX (0x1F-0x22)", tcpReqId);
      return;
    }
    if (cmdByte >= PLC_TX_INIT && cmdByte <= PLC_TX_RETURN) {
      const uint8_t st = plcTcpResolveStateByte();
      plcTcpTxState(st, plcTcpStateName(st));
      return;
    }
    plcSendAck(cmdByte, false, "byte/cmd PLC desconocido", tcpReqId);
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
      plcTcpPushStateIfChanged();
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
      plcTcpPushStateIfChanged();
    }
    return;
  }
  if (nameEq(cmd, "status") || nameEq(cmd, "poll"))
  {
    replyStatus();
    return;
  }
  if (nameEq(cmd, "ping"))
  {
    char pong[80];
    snprintf(pong, sizeof(pong),
             "{\"ver\":%u,\"type\":\"pong\",\"actuator\":\"plc\"}",
             (unsigned)PLCA_PROTO_VER);
    tcpTx(pong);
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

// Byte efectivo para trazas (setOut CUTTERS → 0x2B; sin byte → command).
static uint8_t plcTraceCmdByte(const char* line)
{
  const int b = jInt(line, "byte", -1);
  if (b > 0) return (uint8_t)b;
  char cmd[24];
  char out[24];
  char val[24];
  jStr(line, "command", cmd, sizeof(cmd));
  if (nameEq(cmd, "setOut") || nameEq(cmd, "setPlc") || nameEq(cmd, "plcSet")) {
    jStr(line, "out", out, sizeof(out));
    jStr(line, "value", val, sizeof(val));
    const char* outName = out[0] ? out : val;
    if (nameEq(outName, "CUTTER") || nameEq(outName, "CUTTERS"))
      return PLC_CMD_CUTTERS;
    if (nameEq(outName, "GRIPPER") || nameEq(outName, "GRIPPERS"))
      return PLC_CMD_GRIPPER;
    if (nameEq(outName, "HOLDER")) return PLC_CMD_HOLDER;
    if (nameEq(outName, "ENCODER") || nameEq(outName, "FGTRAY") || nameEq(outName, "TRAY"))
      return PLC_CMD_ENCODER;
    if (nameEq(outName, "BLOWER")) return PLC_CMD_BLOWER;
  }
  return plcTcpResolveCmdByte(line);
}

static bool lineEnqueue(const char* line)
{
  if (!line || lineQn >= LINE_Q) return false;
  if (strlen(line) > 238) return false;
  const uint8_t slot = (uint8_t)((lineQHead + lineQn) % LINE_Q);
  char* dst = lineQ[slot];
  strncpy(dst, line, 239);
  dst[239] = 0;
  lineQMs[slot] = millis();
  lineQn++;
  return true;
}

static void tcpOnLine(const char* line)
{
  if (!line || !line[0]) return;
  // Ping/poll se responden ya. Válvulas esperan a que el pulso anterior termine
  // para no encimar KEEP ni congelar el ACK.
  if (plcLineBlocks(line)) {
    const unsigned long rxMs = millis();
    const uint8_t tb = plcTraceCmdByte(line);
    const uint16_t rid = (uint16_t)jInt(line, "id", 0);
    if (!ackCanAcceptCommand() || !lineEnqueue(line)) {
      Serial.println("[TCP] cola de comandos llena");
      tcpReqMs = rxMs;
      tcpReqId = rid;
      if (tb) {
        ackTraceDebt("full", tb, rid, 0, "lineQ/jobQ saturada");
        // setOut CUTTERS confirma con 0x2B; el resto solo si trae byte de válvula.
        const int rawB = jInt(line, "byte", -1);
        if (rawB > 0)
          plcSendAck((uint8_t)rawB, false, "PLC ocupado", rid);
        else if (tb == PLC_CMD_CUTTERS)
          plcSendAck(PLC_CMD_CUTTERS, false, "PLC ocupado", rid);
      }
      tcpReqMs = 0;
      return;
    }
#if PLC_ACK_TRACE
    if (tb) {
      char note[48];
      snprintf(note, sizeof(note), "held debt=%u", (unsigned)ackDebt());
      ackTraceDebt("hold_q", tb, rid, 0, note);
    }
#endif
    return;
  }
  tcpReqMs = millis();
  tcpHandleLine(line);
  tcpReqMs = 0;
}

static void plcDispatchHeld()
{
  if (vpPhase != VP_IDLE || valveJobCount > 0 || !lineQn) return;
  char line[240];
  memcpy(line, lineQ[lineQHead], sizeof(line));
  tcpReqMs = lineQMs[lineQHead];
  if (!tcpReqMs)
    tcpReqMs = millis();
  lineQHead = (lineQHead + 1) % LINE_Q;
  lineQn--;
#if PLC_ACK_TRACE
  {
    const uint8_t tb = plcTraceCmdByte(line);
    const uint16_t rid = (uint16_t)jInt(line, "id", 0);
    const unsigned long ageMs = (millis() >= tcpReqMs) ? (millis() - tcpReqMs) : 0;
    if (tb)
      ackTraceDebt("dispatch", tb, rid, ageMs, "sale de lineQ");
  }
#endif
  tcpHandleLine(line);
  tcpReqMs = 0;
}

static void tcpRxDrain()
{
  while (tcpClient.available())
  {
    if (!espRxLinePush(tcpRx, (char)tcpClient.read())) continue;
    tcpOnLine(tcpRx.buf);
    tcpRx.len = 0;
  }
}

static void tcpOnClientAccepted()
{
  tcpClient.setNoDelay(true);
  espTcpKeepAlive(tcpClient);
  espRxLineReset(tcpRx);
  lastPeerMask = 0xFF;
  lastPeerCutterR = stCutterR;
  lastPeerCutterL = stCutterL;
  lastPeerGrippers = stGrippers;
  lastPeerHolder = stHolder;
  lastPeerEncoder = stEncoder;
  lastPeerBlower = stBlower;
  lastPeerMask = currentStableMask();
  plcTcpLastStateByte = 0;
  plcTcpNeedInit = true;
  plcTcpStopPending = false;
  peerLinkOk = true;
  valveDropPending();
  ackReset();
  espTcpHoldClear(txHold);
  char hello[80];
  snprintf(hello, sizeof(hello),
           "{\"ver\":%u,\"type\":\"hello\",\"role\":\"plc\"}\n",
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
    espTcpStop(tcpClient);
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
  tcpNoteDead();
  tcpWasConnected = false;
  espRxLineReset(tcpRx);
  if (!tcpServicesUp) return;
  tcpServer.end();
  tcpServicesUp = false;
  wifiReady = false;
  Serial.println("[TCP] Servicios Off");
}

static void serviceSta()
{
  const uint8_t ev = espStaPoll(staLink);
  if (ev & ESP_STA_UP) {
    Serial.printf("[WiFi] On %s\n", WiFi.localIP().toString().c_str());
    tcpEnsureServices();
  }
  if (ev & ESP_STA_BLIP)
    Serial.println("[WiFi] blip — esperando auto-reconexion");
  if (ev & ESP_STA_DOWN) {
    tcpStopServices();
    Serial.println("[WiFi] Off");
  }
  if (ev & ESP_STA_KICK) {
    tcpStopServices();
    if (!espStaKick(staLink, OTA_HOSTNAME, WIFI_SSID, WIFI_PASS,
                    STA_IP, STA_GW, STA_MASK, STA_GW))
      Serial.println("WiFi.config() fallo.");
    Serial.println("[WiFi] Reintentando STA...");
  }
}

static void serviceTcp()
{
  serviceSta();
  if (!staLink.wasUp) return;

  tcpEnsureServices();

  if (tcpAcceptIncoming())
    return;

  if (!tcpLinkOk())
  {
    if (tcpClient.available()) tcpRxDrain();
    if (tcpWasConnected)
    {
      Serial.println("[TCP] Off");
      tcpWasConnected = false;
      espRxLineReset(tcpRx);
      valveDropPending();
    }
    if (tcpClient) tcpNoteDead();
    return;
  }

  tcpWasConnected = true;
  peerLinkOk = true;
  tcpRxDrain();
  plcTcpPollStateEvents();
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

  espStaClear(staLink);
  Serial.printf("Buscando router \"%s\"...\n", WIFI_SSID);
  if (!espStaKick(staLink, OTA_HOSTNAME, WIFI_SSID, WIFI_PASS,
                  STA_IP, STA_GW, STA_MASK, STA_GW))
    Serial.println("WiFi.config() fallo.");
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("WiFi pendiente — reintentos en loop");
  bootGraceStartMs = millis();
  Serial.printf("Sensor boot grace %lums desde ahora\n", BOOT_GRACE_MS);
  Serial.printf("Firmware v%s · OTA red local hostname=%s\n", FW_VERSION, OTA_HOSTNAME);
#if PLC_ACK_TRACE
  Serial.println("[ACK] trace ON · tags: hold_q/dispatch/rx_cmd/defer/pulse_*/send_*/SLOW/full/DROP");
  Serial.printf("[ACK] warn si age>=%lums (HMI watch ~2.5s)\n",
                (unsigned long)ACK_SLOW_WARN_MS);
#endif
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

#if PLC_ACK_TRACE
// Un loop trabado retrasa tcpTxPump → el ACK no sale y la HMI vence el watch.
static void plcLoopLagWatch()
{
  static unsigned long lastMs = 0;
  const unsigned long now = millis();
  if (lastMs) {
    const unsigned long gap = now - lastMs;
    if (gap >= 80)
      Serial.printf("[ACK] loop_lag=%lums lineQ=%u jobQ=%u ackQ=%u vp=%u hold=%u\n",
                    (unsigned long)gap,
                    (unsigned)lineQn,
                    (unsigned)valveJobCount,
                    (unsigned)ackQn,
                    (unsigned)vpPhase,
                    (unsigned)txHold.len);
  }
  lastMs = now;
}
#else
static void plcLoopLagWatch() {}
#endif

void loop()
{
  if (arduinoOtaInProgress)
  {
    serviceArduinoOta(24);
    serviceTcp();
    yield();
    return;
  }

  plcLoopLagWatch();
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
    yield();
    return;
  }

  // Baseline (lastBitmask==0xFF): solo fijar máscara; no tratar bits presentes
  // como flanco rising (E050/E047–E049 fantasma al primer sample / HMI connect).
  const bool establishingBaseline = (lastBitmask == 0xFF);
  if (establishingBaseline)
    lastPeerMask = stable;
  lastBitmask = stable;
  if (tcpLinkOk())
  {
    peerTxEvents();
    plcTcpPushStateIfChanged();
  }
  else
    Serial.println("TCP offline — sensores/válvulas solo locales");
  serviceTcp();
}
