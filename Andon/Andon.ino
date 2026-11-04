// Andon — torreta + presión FRL.
// HMI manda ANDON_RX_* (0x40–0x49). Presión: pin local → torreta Error + TX 0x50.

#include "Config.h"
#include "AndonStates.h"
#include <WiFi.h>
#include "../common/EspLink.h"

static WiFiServer tcpServer(ANDON_TCP_PORT);
static WiFiClient tcpClient;
static char tcpRxLine[384];
static EspRxLine tcpRx = {tcpRxLine, sizeof(tcpRxLine), 0, false};
static const uint8_t ACK_Q = 4;
static char ackQ[ACK_Q][192];
static char ackSpill[192];
static bool ackSpillOn = false;
static bool tcpRxHeld = false;
static uint8_t ackQHead = 0;
static uint8_t ackQn = 0;
static bool andonStatusDirty = false;
static bool pressureEventPending = false;
static bool tcpServicesUp = false;
static bool tcpWasConnected = false;

static EspSta staLink;
static char tcpHoldBuf[512];
static EspTcpHold tcpHold = {tcpHoldBuf, sizeof(tcpHoldBuf), 0};

static uint8_t lastMachineByte = ANDON_RX_IDLE;
static bool pressureFaultLatched = false;
static bool buzzerMuted = false;
static bool manualOverride = false;
static bool stGreen = false;
static bool stYellow = false;
static bool stRed = false;
static bool stBuzzerWant = false;

// Fin de WO (0x47): secuencia temporal R→Y→G + buzzer (no bloqueante).
static bool finishSeqActive = false;
static uint8_t finishSeqPhase = 0;   // 0=R, 1=Y, 2=G por ciclo
static uint8_t finishSeqCycle = 0;
static uint32_t finishSeqPhaseMs = 0;

static bool tcpLinkOk();
static bool tcpTx(const String& m);
static void andonTxStatus();
static void andonFinishSeqStop(bool settleIdle);
static void andonFinishSeqStart();
static void andonApplyStaticTower(uint8_t byteCode);

static void andonWriteOut(uint8_t pin, bool on)
{
#if ANDON_ACTIVE_HIGH
  digitalWrite(pin, on ? HIGH : LOW);
#else
  digitalWrite(pin, on ? LOW : HIGH);
#endif
}

void andonSetGreen(bool on)
{
  stGreen = on;
  andonWriteOut(PIN_GREEN_LED, on);
}
void andonSetYellow(bool on)
{
  stYellow = on;
  andonWriteOut(PIN_YELLOW_LED, on);
}
void andonSetRed(bool on)
{
  stRed = on;
  andonWriteOut(PIN_RED_LED, on);
}
void andonSetBuzzer(bool on)
{
  stBuzzerWant = on;
  if (buzzerMuted) on = false;
  andonWriteOut(PIN_BUZZER, on);
}

void andonTowerAllOff()
{
  andonSetGreen(false);
  andonSetYellow(false);
  andonSetRed(false);
  andonSetBuzzer(false);
}

static bool andonTxStatusNow()
{
  if (!tcpLinkOk()) return false;
  char buf[280];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"status\",\"actuator\":\"andon\","
           "\"green\":%s,\"yellow\":%s,\"red\":%s,\"buzzer\":%s,"
           "\"manual\":%s,\"mute\":%s,\"byte\":%u,\"pressure\":%s}",
           (unsigned)ANDON_PROTO_VER,
           stGreen ? "true" : "false",
           stYellow ? "true" : "false",
           stRed ? "true" : "false",
           (stBuzzerWant && !buzzerMuted) ? "true" : "false",
           manualOverride ? "true" : "false",
           buzzerMuted ? "true" : "false",
           (unsigned)lastMachineByte,
           pressureFaultLatched ? "true" : "false");
  return tcpTx(String(buf));
}

static void andonTxStatus()
{
  if (andonTxStatusNow()) andonStatusDirty = false;
  else andonStatusDirty = true;
}

static bool andonIsNaByte(uint8_t byteCode)
{
  // Start / Reset → N/A (no cambian torreta)
  return byteCode == ANDON_RX_START || byteCode == ANDON_RX_RESET;
}

static void andonFinishSeqStop(bool settleIdle)
{
  finishSeqActive = false;
  finishSeqPhase = 0;
  finishSeqCycle = 0;
  finishSeqPhaseMs = 0;
  if (settleIdle) {
    lastMachineByte = ANDON_RX_IDLE;
    andonTowerAllOff();
    andonSetGreen(true);
    andonTxStatus();
  }
}

static void andonFinishSeqStart()
{
  finishSeqActive = true;
  finishSeqPhase = 0;
  finishSeqCycle = 0;
  finishSeqPhaseMs = millis();
  andonTowerAllOff();
  andonSetRed(true);
  andonSetBuzzer(true);
  andonTxStatus();
}

void andonServiceFinishSequence()
{
  if (!finishSeqActive || pressureFaultLatched || manualOverride) return;

  const uint32_t now = millis();
  if ((now - finishSeqPhaseMs) < ANDON_FINISH_STEP_MS) return;

  finishSeqPhaseMs = now;
  finishSeqPhase++;

  if (finishSeqPhase >= 3) {
    finishSeqPhase = 0;
    finishSeqCycle++;
    if (finishSeqCycle >= ANDON_FINISH_CYCLES) {
      andonFinishSeqStop(true);
      return;
    }
  }

  andonTowerAllOff();
  andonSetBuzzer(true);
  if (finishSeqPhase == 0) andonSetRed(true);
  else if (finishSeqPhase == 1) andonSetYellow(true);
  else andonSetGreen(true);
  andonTxStatus();
}

static void andonApplyStaticTower(uint8_t byteCode)
{
  andonTowerAllOff();
  switch (byteCode) {
    case ANDON_RX_INIT:
    case ANDON_RX_IDLE:
    case ANDON_RX_BUSY:
      andonSetGreen(true);
      break;
    case ANDON_RX_STOP:
      andonSetRed(true);
      break;
    case ANDON_RX_ERROR:
      andonSetRed(true);
      andonSetBuzzer(true);
      break;
    case ANDON_RX_PAUSE:
      andonSetYellow(true);
      break;
    case ANDON_RX_MATERIALIST:
      andonSetYellow(true);
      andonSetBuzzer(true);
      break;
    default:
      break;
  }
}

void andonApplyMachineByte(uint8_t byteCode)
{
  if (!andonIsMachineByte(byteCode)) return;
  if (andonIsNaByte(byteCode)) return;
  // T1: presión reporta 0x50 a Main; no se bloquea el RX de estado/manual.

  manualOverride = false;
  andonFinishSeqStop(false);
  lastMachineByte = byteCode;

  if (byteCode == ANDON_RX_FINISH) {
    andonFinishSeqStart();
    return;
  }

  andonApplyStaticTower(byteCode);
  andonTxStatus();
}

void andonSetBuzzerMute(bool mute)
{
  buzzerMuted = mute;
  if (mute) {
    andonWriteOut(PIN_BUZZER, false);
  } else if (!pressureFaultLatched) {
    if (manualOverride) {
      andonSetBuzzer(stBuzzerWant);
    } else if (finishSeqActive) {
      andonSetBuzzer(true);
    } else {
      andonApplyStaticTower(lastMachineByte);
    }
  } else {
    andonSetBuzzer(true);
  }
  andonTxStatus();
}

static void andonManualSetOut(const String& outName, bool wantOn)
{
  andonFinishSeqStop(false);
  manualOverride = true;
  String n = outName;
  n.trim();
  n.toLowerCase();
  if (n == "green" || n == "verde") andonSetGreen(wantOn);
  else if (n == "yellow" || n == "amarillo") andonSetYellow(wantOn);
  else if (n == "red" || n == "rojo") andonSetRed(wantOn);
  else if (n == "buzzer" || n == "beep") andonSetBuzzer(wantOn);
  andonTxStatus();
}

static void andonManualAllOff()
{
  andonFinishSeqStop(false);
  manualOverride = true;
  andonTowerAllOff();
  andonTxStatus();
}

static void andonResumeAuto()
{
  manualOverride = false;
  andonApplyMachineByte(lastMachineByte);
}

static bool andonTxPressureNow()
{
  if (!tcpLinkOk()) return false;
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"event\",\"actuator\":\"andon\","
           "\"byte\":%u,\"name\":\"PressureError\"}",
           (unsigned)ANDON_PROTO_VER, (unsigned)ANDON_ERR_PRESSURE);
  return tcpTx(String(buf));
}

void PressureError()
{
  pressureEventPending = !andonTxPressureNow();
}

bool andonPressureFaultRaw()
{
  // Presostato FRL: LOW = presión correcta; HIGH = falta de presión.
  return digitalRead(PIN_PRESSURE_FRL) == HIGH;
}

void andonServicePressure()
{
  static bool raw = false, stable = false;
  static uint32_t tChange = 0;
  const uint32_t now = millis();
  const bool r = andonPressureFaultRaw();

  if (r != raw) {
    raw = r;
    tChange = now;
  } else if ((now - tChange) >= ANDON_PRESSURE_DEBOUNCE_MS && r != stable) {
    stable = r;
    if (stable) {
      pressureFaultLatched = true;
      andonFinishSeqStop(false);
      andonTowerAllOff();
      andonSetRed(true);
      andonSetBuzzer(true);
      PressureError();
      andonTxStatus();
    } else {
      pressureFaultLatched = false;
      andonApplyMachineByte(lastMachineByte);
    }
  }
}

// --- TCP helpers ---
static bool tcpLinkOk()
{
  return tcpClient && tcpClient.connected();
}

static void ackReset()
{
  ackQHead = 0;
  ackQn = 0;
  ackSpillOn = false;
  tcpRxHeld = false;
}

static void tcpNoteDead()
{
  espTcpStop(tcpClient);
  espTcpHoldClear(tcpHold);
  ackReset();
  andonStatusDirty = true;
  if (pressureFaultLatched) pressureEventPending = true;
}

static bool tcpTx(const String& m)
{
  if (!m.length() || tcpHold.len || !tcpLinkOk()) return false;
  if (!espTcpHoldPut(tcpHold, m.c_str())) return false;
  if (espTcpHoldFlush(tcpClient, tcpHold)) return true;
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

static bool ackPark(const char* line)
{
  if (ackStore(line)) return true;
  if (ackSpillOn) return false;
  if (!ackCopyLine(ackSpill, sizeof(ackSpill), line)) return false;
  ackSpillOn = true;
  return true;
}

static bool ackHasRoom()
{
  return ackQn < ACK_Q || !ackSpillOn;
}

static void tcpTxPump()
{
  if (!tcpLinkOk()) return;
  if (tcpHold.len && !espTcpHoldFlush(tcpClient, tcpHold)) {
    if (!tcpClient.connected()) tcpNoteDead();
    return;
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
  if (pressureEventPending) {
    if (!andonTxPressureNow()) return;
    pressureEventPending = false;
  }
  if (andonStatusDirty) {
    if (andonTxStatusNow()) andonStatusDirty = false;
  }
}

static int jInt(const char* j, const char* k, int d)
{
  String n = String("\"") + k + "\":";
  int i = String(j).indexOf(n);
  return i < 0 ? d : String(j).substring(i + n.length()).toInt();
}

static String jStr(const char* j, const char* k)
{
  String n = String("\"") + k + "\":\"";
  String s(j);
  int i = s.indexOf(n);
  if (i < 0) return "";
  int a = i + n.length(), b = s.indexOf('"', a);
  return b < 0 ? "" : s.substring(a, b);
}

static bool jBool(const char* j, const char* k, bool d)
{
  String n = String("\"") + k + "\":";
  String s(j);
  int i = s.indexOf(n);
  if (i < 0) return d;
  String rest = s.substring(i + n.length());
  rest.trim();
  if (rest.startsWith("true") || rest.startsWith("1")) return true;
  if (rest.startsWith("false") || rest.startsWith("0")) return false;
  return d;
}

static void tcpAck(uint8_t byteCode, bool ok, const char* msg)
{
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"ver\":%u,\"type\":\"ack\",\"actuator\":\"andon\",\"byte\":%u,"
           "\"ok\":%s,\"message\":\"%s\"}",
           (unsigned)ANDON_PROTO_VER, (unsigned)byteCode,
           ok ? "true" : "false", msg ? msg : "");
  if (!tcpLinkOk()) return;
  if (!tcpHold.len && espTcpHoldPut(tcpHold, buf)) {
    if (!espTcpHoldFlush(tcpClient, tcpHold) && !tcpClient.connected())
      tcpNoteDead();
    return;
  }
  if (!ackPark(buf))
    Serial.println("[TCP] ACK no enviado");
}

static void tcpOnLine(const char* line)
{
  if (!line || !line[0]) return;
  if (!strstr(line, "\"type\":\"command\"")) return;

  const String cmd = jStr(line, "command");
  if (cmd == "ping") {
    char pong[80];
    snprintf(pong, sizeof(pong),
             "{\"ver\":%u,\"type\":\"pong\",\"actuator\":\"andon\"}",
             (unsigned)ANDON_PROTO_VER);
    tcpTx(pong);
    return;
  }
  if (cmd == "status" || cmd == "poll") {
    andonTxStatus();
    return;
  }
  if (cmd == "buzzerMute") {
    const bool mute = jBool(line, "mute", jBool(line, "on", false));
    andonSetBuzzerMute(mute);
    tcpAck(0, true, mute ? "buzzer muted" : "buzzer on");
    return;
  }
  if (cmd == "setOut" || cmd == "setAndon") {
    const String outName = jStr(line, "out");
    const bool wantOn = jBool(line, "on", true);
    if (!outName.length()) {
      tcpAck(0, false, "out requerido");
      return;
    }
    andonManualSetOut(outName, wantOn);
    tcpAck(0, true, "setOut");
    return;
  }
  if (cmd == "allOff") {
    andonManualAllOff();
    tcpAck(0, true, "allOff");
    return;
  }
  if (cmd == "resumeAuto" || cmd == "auto") {
    andonResumeAuto();
    tcpAck(0, true, "resumeAuto");
    return;
  }

  const int rawByte = jInt(line, "byte", -1);
  if (rawByte < 0) {
    tcpAck(0, false, "byte/cmd Andon desconocido");
    return;
  }
  const uint8_t byteCode = (uint8_t)rawByte;
  if (!andonIsMachineByte(byteCode)) {
    tcpAck(byteCode, false, "byte fuera de 0x40-0x49");
    return;
  }
  andonApplyMachineByte(byteCode);
  tcpAck(byteCode, true, andonRxName(byteCode));
}

static void tcpRxDrain()
{
  while (tcpClient.available() || tcpRxHeld) {
    if (!tcpRxHeld) {
      if (!espRxLinePush(tcpRx, (char)tcpClient.read())) continue;
      tcpRxHeld = true;
    }
    if (!ackHasRoom()) break;
    tcpOnLine(tcpRx.buf);
    tcpRx.len = 0;
    tcpRxHeld = false;
  }
}

static void tcpOnClientAccepted()
{
  tcpClient.setNoDelay(true);
  espTcpKeepAlive(tcpClient);
  espTcpHoldClear(tcpHold);
  espRxLineReset(tcpRx);
  tcpRxHeld = false;
  ackSpillOn = false;
  tcpTx(String("{\"ver\":") + ANDON_PROTO_VER + ",\"type\":\"hello\",\"role\":\"andon\"}");
  andonTxStatus();
  tcpWasConnected = true;
}

static bool tcpAcceptIncoming()
{
  if (!tcpServicesUp || !tcpServer.hasClient()) return false;
  WiFiClient incoming = tcpServer.available();
  if (!incoming) return false;
  if (tcpClient)
    espTcpStop(tcpClient);
  tcpClient = incoming;
  tcpOnClientAccepted();
  return true;
}

static void tcpEnsureServices()
{
  if (tcpServicesUp || WiFi.status() != WL_CONNECTED) return;
  const IPAddress ip = WiFi.localIP();
  if (ip == IPAddress(0, 0, 0, 0)) return;
  tcpServer.end();
  delay(20);
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  tcpServicesUp = true;
  Serial.printf("[TCP] Servicios On %s:%u\n",
                ip.toString().c_str(), (unsigned)ANDON_TCP_PORT);
}

static void tcpStopServices()
{
  tcpNoteDead();
  tcpWasConnected = false;
  espRxLineReset(tcpRx);
  if (!tcpServicesUp) return;
  tcpServer.end();
  tcpServicesUp = false;
  Serial.println("[TCP] Servicios Off");
}

static void serviceWifi()
{
  const uint8_t ev = espStaPoll(staLink);
  if (ev & ESP_STA_UP) {
    const IPAddress ip = WiFi.localIP();
    Serial.printf("WiFi OK  IP=%s  RSSI=%d dBm\n",
                  ip.toString().c_str(), WiFi.RSSI());
    if (ip != STA_IP) {
      Serial.printf("[WiFi] AVISO: IP real %s != fija %s\n",
                    ip.toString().c_str(), STA_IP.toString().c_str());
    }
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
    if (!espStaKick(staLink, "andon", WIFI_SSID, WIFI_PASS,
                    STA_IP, STA_GW, STA_MASK, STA_DNS))
      Serial.println("WiFi.config() fallo (IP estatica)");
    Serial.println("[WiFi] Reintentando STA...");
  }
}

static void serviceTcp()
{
  if (!staLink.wasUp) return;

  tcpEnsureServices();
  tcpTxPump();

  if (tcpAcceptIncoming())
    return;

  if (!tcpLinkOk()) {
    if (tcpClient.available()) tcpRxDrain();
    if (tcpWasConnected) {
      Serial.println("[TCP] Off");
      tcpWasConnected = false;
      espRxLineReset(tcpRx);
    }
    if (tcpClient) tcpNoteDead();
    return;
  }

  tcpWasConnected = true;
  tcpRxDrain();
  tcpTxPump();
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_YELLOW_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_PRESSURE_FRL, INPUT_PULLUP);

  pinMode(PIN_ETH_CS, OUTPUT);
  pinMode(PIN_ETH_CLK, OUTPUT);
  pinMode(PIN_ETH_RST, OUTPUT);
  pinMode(PIN_ETH_MOSI, OUTPUT);
  pinMode(PIN_ETH_INT, INPUT);

  andonTowerAllOff();
  andonApplyMachineByte(ANDON_RX_INIT);

  espStaClear(staLink);
  Serial.printf("Conectando a WiFi '%s' -> %s...\n",
                WIFI_SSID, STA_IP.toString().c_str());
  if (!espStaKick(staLink, "andon", WIFI_SSID, WIFI_PASS,
                  STA_IP, STA_GW, STA_MASK, STA_DNS))
    Serial.println("WiFi.config() fallo (IP estatica)");
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("WiFi pendiente — reconexion no bloqueante en loop...");
  tcpEnsureServices();

  Serial.printf("Andon TCP en %s:%u\n",
                STA_IP.toString().c_str(), (unsigned)ANDON_TCP_PORT);
}

void loop()
{
  serviceWifi();
  serviceTcp();
  andonServiceFinishSequence();
  andonServicePressure();
}
