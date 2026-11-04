// PF_V2 Pre-Feeder L/R — loop: comandos, machineTick, red; motores en outputTask.

#include "config.h"
#include "params.h"
#include "machine.h"
#include "page.h"
#include <ArduinoOTA.h>
#include <HardwareSerial.h>
#include <WebServer.h>
#include <WiFi.h>
#include "../common/EspLink.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static constexpr size_t CMD_Q_LEN = 16;
static NetCommand cmdRing[CMD_Q_LEN];
static size_t cmdHead = 0, cmdTail = 0;
static SemaphoreHandle_t cmdMux = nullptr;

static void commandsInit() { cmdMux = xSemaphoreCreateMutex(); }

bool machinePopCommand(NetCommand& out) {
  if (!cmdMux) return false;
  bool got = false;
  if (xSemaphoreTake(cmdMux, portMAX_DELAY)) {
    if (cmdTail != cmdHead) {
      out = cmdRing[cmdTail];
      cmdTail = (cmdTail + 1) % CMD_Q_LEN;
      got = true;
    }
    xSemaphoreGive(cmdMux);
  }
  return got;
}

static bool commandsPush(const NetCommand& cmd) {
  if (!cmdMux) return false;
  bool ok = false;
  if (xSemaphoreTake(cmdMux, portMAX_DELAY)) {
    size_t next = (cmdHead + 1) % CMD_Q_LEN;
    if (next != cmdTail) {
      cmdRing[cmdHead] = cmd;
      cmdHead = next;
      ok = true;
    }
    xSemaphoreGive(cmdMux);
  }
  return ok;
}

static SensorSnapshot gSnap;
static uint32_t fullHighMs = 0, fullLowMs = 0;
static bool fullStable = false;
static uint32_t midHighMs = 0, midLowMs = 0;
static bool midStable = false;
static uint32_t highOnMs = 0, highOffMs = 0;
static bool highStable = false;
static uint32_t holgHighMs = 0, holgLowMs = 0;
static bool holguraStableOn = false;

static void sensorsInitPins() {
  pinMode(PIN_BUF_FULL, INPUT_PULLUP);
  pinMode(PIN_BUF_MID, INPUT);
  pinMode(PIN_BUF_HIGH, INPUT);
  pinMode(PIN_BUF_MAX, INPUT_PULLUP);
  pinMode(PIN_CYLINDER, INPUT_PULLDOWN);
  pinMode(PIN_HOSE, INPUT_PULLUP);
  pinMode(PIN_HOLGURA, INPUT_PULLUP);
}

static bool holguraSensorRawOn() {
  return digitalRead(PIN_HOLGURA) == HOLGURA_ON_LEVEL;
}

static bool holguraOkStable() { return holguraStableOn; }

// Misma espera que GPIO19: 50 ms ON, 100 ms OFF.
static void debounceLevel(bool raw, uint32_t dt, uint32_t& onMs, uint32_t& offMs, bool& stable) {
  if (raw) {
    onMs += dt;
    offMs = 0;
    if (onMs >= BUF_FULL_ON_MS) stable = true;
  } else {
    offMs += dt;
    onMs = 0;
    if (offMs >= BUF_FULL_OFF_MS) stable = false;
  }
}

// Lee GPIO de buffer, cilindro, manguera; filtra estabilidad GPIO19 (full)
static void sensorsUpdate(uint32_t nowMs) {
  static uint32_t lastMs = 0;
  uint32_t dt = lastMs ? (nowMs - lastMs) : 0;
  lastMs = nowMs;

  gSnap.bufFullRaw = digitalRead(PIN_BUF_FULL) == HIGH;
  gSnap.bufMidRaw = digitalRead(PIN_BUF_MID) == HIGH;
  gSnap.bufHighRaw = digitalRead(PIN_BUF_HIGH) == HIGH;
  gSnap.bufMaxRaw = digitalRead(PIN_BUF_MAX) == HIGH;
  debounceLevel(gSnap.bufMidRaw, dt, midHighMs, midLowMs, midStable);
  debounceLevel(gSnap.bufHighRaw, dt, highOnMs, highOffMs, highStable);
  gSnap.bufMidStable = midStable;
  gSnap.bufHighStable = highStable;
  gSnap.cylinderOpen = digitalRead(PIN_CYLINDER) == HIGH;
  gSnap.hoseAbsent = digitalRead(PIN_HOSE) == HIGH;

  if (holguraSensorRawOn()) {
    holgHighMs += dt;
    holgLowMs = 0;
    if (holgHighMs >= HOLGURA_ON_STABLE_MS) holguraStableOn = true;
  } else {
    holgLowMs += dt;
    holgHighMs = 0;
    if (holgLowMs >= HOLGURA_OFF_STABLE_MS) holguraStableOn = false;
  }
  gSnap.holguraOn = holguraStableOn;

  if (gSnap.bufFullRaw) {
    fullHighMs += dt;
    fullLowMs = 0;
    if (fullHighMs >= BUF_FULL_ON_MS) fullStable = true;  // espera ON estable → buffer lleno
  } else {
    fullLowMs += dt;
    fullHighMs = 0;
    if (fullLowMs >= BUF_FULL_OFF_MS) fullStable = false;  // espera OFF estable → no lleno
  }
  gSnap.bufFullStable = fullStable;
}

static StepperRMT* dereeler = nullptr;
static StepperRMT* feeder = nullptr;
static SemaphoreHandle_t intentMux = nullptr;
static OutputIntent desired;
static OutputIntent applied;
static bool appliedValid = false;
static Params* P = nullptr;
static bool servoLedcOk = false;

static uint32_t servoUsToDuty(uint16_t us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  return (uint32_t)us * ((1UL << SERVO_LEDC_BITS) - 1) / 20000UL;
}

static bool servoWriteUs(uint16_t us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  if (!servoLedcOk) return false;
  return ledcWrite(PIN_SERVO, servoUsToDuty(us));
}

static void servoNeutral() { servoWriteUs(SERVO_NEUTRAL_US); }

static void servoMotionUs(int16_t extraDelta) {
  if (!P) return;
  int v = (int)P->servoActiveUs + (int)extraDelta;
  v = constrain(v, (int)SERVO_MIN_US, (int)SERVO_MAX_US);
  servoWriteUs((uint16_t)v);
}

static void dereelerDirCw() { digitalWrite(PIN_DEREELER_DIR, DEREELER_CW_LEVEL); }
static void dereelerDirCcw() { digitalWrite(PIN_DEREELER_DIR, DEREELER_CCW_LEVEL); }

// RPM en caché: no Brake/setSpeed cada 8 ms.
static float commandedFeederRpm = 0.0f;
static float commandedDereelerRpm = 0.0f;
static bool dereelerCmdReverse = false;

static void motorBrakeIfRunning(StepperRMT* m, float& commanded) {
  if (!m) return;
  if (commanded > 0.01f) {
    m->Brake();
    commanded = 0.0f;
  }
}

static bool outputIntentEqual(const OutputIntent& a, const OutputIntent& b) {
  return a.mode == b.mode && fabsf(a.feederRpm - b.feederRpm) <= 0.25f
         && fabsf(a.dereelerRpm - b.dereelerRpm) <= 0.25f
         && a.dereelerReverse == b.dereelerReverse && a.servoRun == b.servoRun
         && a.servoExtraUs == b.servoExtraUs;
}

static void motorApplyRpm(StepperRMT* m, float& commanded, float targetRpm) {
  if (!m) return;
  if (targetRpm < RPM_MIN) {
    motorBrakeIfRunning(m, commanded);
    return;
  }
  targetRpm = constrain(targetRpm, RPM_MIN, RPM_MAX);
  if (fabsf(commanded - targetRpm) <= 0.5f) return;
  if (m->setSpeed(targetRpm, MOTOR_ACCEL)) commanded = targetRpm;
  // Si setSpeed falla: no Brake — reintentar en el siguiente ciclo sin reiniciar perfil.
}

static void motorFeederTarget(float rpm) {
  motorApplyRpm(feeder, commandedFeederRpm, rpm > 0.5f ? rpm : 0.0f);
}

static void motorDereelerTarget(float rpm, bool reverse) {
  if (!dereeler) return;
  if (rpm < 0.5f) {
    motorBrakeIfRunning(dereeler, commandedDereelerRpm);
    return;
  }
  rpm = constrain(rpm, RPM_MIN, RPM_MAX);
  if (reverse != dereelerCmdReverse) {
    if (commandedDereelerRpm > 0.01f) {
      dereeler->Brake();
      commandedDereelerRpm = 0.0f;
    }
    if (reverse)
      dereelerDirCcw();
    else
      dereelerDirCw();
    dereelerCmdReverse = reverse;
  }
  motorApplyRpm(dereeler, commandedDereelerRpm, rpm);
}

static void motorOutputsAllStop() {
  motorBrakeIfRunning(feeder, commandedFeederRpm);
  motorBrakeIfRunning(dereeler, commandedDereelerRpm);
  appliedValid = false;
}

// Ejecuta intención en hardware: mueve feeder / DeReeler / PWM servo
static void applyIntent(const OutputIntent& in) {
  switch (in.mode) {
    case OUT_AUTO: {
      motorFeederTarget(in.feederRpm);
      if (in.dereelerRpm > 0.5f)
        motorDereelerTarget(in.dereelerRpm, in.dereelerReverse);
      else
        motorDereelerTarget(0.0f, false);
      if (in.servoRun) servoMotionUs(in.servoExtraUs);
      else servoNeutral();
      break;
    }
    case OUT_REFILL_MANUAL:  // Materialista: refill manual por canal
      motorFeederTarget(in.feederRpm);
      motorDereelerTarget(in.dereelerRpm > 0.5f ? in.dereelerRpm : 0.0f, false);
      if (in.servoRun) servoMotionUs(0);
      else servoNeutral();
      break;
    default:  // OUT_IDLE: para todo
      motorOutputsAllStop();
      servoNeutral();
      break;
  }
}

// Tarea FreeRTOS: aplica applyIntent en bucle (no bloquea loop principal)
static void outputTask(void*) {
  for (;;) {
    OutputIntent copy;
    if (xSemaphoreTake(intentMux, portMAX_DELAY)) {
      copy = desired;
      xSemaphoreGive(intentMux);
    }
    if (!appliedValid || !outputIntentEqual(copy, applied)) {
      applyIntent(copy);
      applied = copy;
      appliedValid = true;
    }
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

static void outputsInit() {
  pinMode(PIN_SERVO, OUTPUT);
  digitalWrite(PIN_SERVO, LOW);
  servoLedcOk = ledcAttachChannel(PIN_SERVO, 50, SERVO_LEDC_BITS, SERVO_LEDC_CH);
  servoNeutral();
  pinMode(PIN_DEREELER_DIR, OUTPUT);
  dereelerDirCw();
  StepperRMT::Pins p1 = { PIN_DEREELER_STEP, -1, -1, -1, -1, -1, -1 };
  dereeler = new StepperRMT(DRIVER_DM556T, p1);
  dereeler->begin();
  dereeler->setDefault(MOTOR_MICROSTEP, MOTOR_ACCEL, MOTOR_DECEL);
  dereeler->Brake();
  StepperRMT::Pins p2 = { PIN_FEEDER_STEP, -1, -1, -1, -1, -1, -1 };
  feeder = new StepperRMT(DRIVER_DM556T, p2);
  feeder->begin();
  feeder->setDefault(MOTOR_MICROSTEP, MOTOR_ACCEL, MOTOR_DECEL);
  feeder->Brake();
  intentMux = xSemaphoreCreateMutex();
}

static void outputsSetIntent(const OutputIntent& intent) {
  if (!intentMux) return;
  if (xSemaphoreTake(intentMux, portMAX_DELAY)) {
    desired = intent;
    xSemaphoreGive(intentMux);
  }
}

static SystemFault fault = FAULT_NONE;
// Sube en cada Reset ya aplicado. El Master ignora un status anterior (mismo epoch).
static uint16_t faultEpoch = 1;
static bool peerStatusForce = false;
static bool autoOn = false;
static bool idleMode = false;
static bool inProcess = false;
static bool benchInProcess = false;
// In process OFF pedido: no cortar todavía. Idle real = full ON y holgura ON.
static bool releaseToIdlePending = false;

enum FeederMode : uint8_t {
  FEED_NONE = 0,
  FEED_TFEED,
  FEED_HOLGURA,
};

enum HolguraPhase : uint8_t {
  HOLG_IDLE = 0,
  HOLG_WAIT_ON,
  HOLG_SETTLE,
  HOLG_TIMED_FEED,
};

static bool feederDemand = false;
static FeederMode feederMode = FEED_NONE;
static HolguraPhase holguraPhase = HOLG_IDLE;
static uint32_t feederMinEndMs = 0;
static uint32_t holguraSettleEndMs = 0;
static uint32_t holguraTimedEndMs = 0;
static bool holguraLastOn = false;
static bool holguraSawOn = false;
static bool holguraPostCycleOffWatch = false;
static uint32_t holguraOffSinceMs = 0;
static uint32_t holguraStallSinceMs = 0;
static uint16_t holguraActiveTriggerId = 0;
static bool pendingTrigger = false;
static float pendingTriggerSec = 0.0f;
static uint16_t pendingTriggerId = 0;
static uint16_t doneTriggerIds[8] = {};
static uint8_t doneTriggerCount = 0;

static uint32_t bufferEmptySinceMs = 0;

static bool refillMat = false, refillDr = false, refillSv = false, refillFd = false;
static uint32_t refillPulseUntil[4] = {};
static uint32_t midBoostUntil = 0, highBoostUntil = 0;
static bool lastMid = false, lastHigh = false;
static bool highReverseActive = false;
static uint32_t highReverseUntilMs = 0;
static bool highReverseCanArm = true;
static bool midPassedBeforeHigh = false;

static bool triggerIdDone(uint16_t id) {
  if (!id) return false;
  for (uint8_t i = 0; i < doneTriggerCount; i++)
    if (doneTriggerIds[i] == id) return true;
  return false;
}

static void triggerIdRemember(uint16_t id) {
  if (!id || triggerIdDone(id)) return;
  if (doneTriggerCount < 8) doneTriggerIds[doneTriggerCount++] = id;
}

// Producción armada: auto ON + inProcess + no Materialista + sin falla
static bool motionArmed() {
  return autoOn && !idleMode && inProcess && fault == FAULT_NONE;
}

static bool refillActive() { return refillMat || refillDr || refillSv || refillFd; }

// Buffer sin GPIO19 estable: DeReeler/servo pueden correr.
static bool bufferWantsFill() {
  return motionArmed() && !gSnap.bufFullStable;
}

static void resetHolguraFeedState() {
  holguraPhase = HOLG_IDLE;
  holguraSettleEndMs = 0;
  holguraTimedEndMs = 0;
  holguraLastOn = false;
  holguraSawOn = false;
  holguraPostCycleOffWatch = false;
  holguraOffSinceMs = 0;
  holguraStallSinceMs = 0;
  holguraActiveTriggerId = 0;
}

static void holguraOnRisingEdge(uint32_t nowMs) {
  holguraSawOn = true;
  holguraPhase = HOLG_SETTLE;
  holguraSettleEndMs = nowMs + (P ? (uint32_t)P->holguraSettleMs : (uint32_t)HOLGURA_SETTLE_MS_DEFAULT);
  holguraTimedEndMs = 0;
  holguraOffSinceMs = 0;
}

static void enterFault(SystemFault f);

static void stopFeederDemand() {
  feederDemand = false;
  feederMode = FEED_NONE;
  feederMinEndMs = 0;
  resetHolguraFeedState();
  pendingTrigger = false;
}

static void beginHolguraFeed(uint32_t nowMs) {
  (void)nowMs;
  if (holguraOkStable()) return;
  holguraPostCycleOffWatch = false;
  // No reiniciar holguraOffSinceMs: rearmar el feeder no borra el OFF sostenido.
  feederMode = FEED_HOLGURA;
  holguraSawOn = false;
  holguraSettleEndMs = 0;
  holguraTimedEndMs = 0;
  holguraLastOn = false;
  holguraPhase = HOLG_WAIT_ON;
  feederDemand = true;
}

// Ciclo holgura terminó: flanco ON → settle → n s de alimentación.
static void stopHolguraFeedCycle(uint32_t nowMs) {
  triggerIdRemember(holguraActiveTriggerId);
  holguraActiveTriggerId = 0;
  feederDemand = false;
  feederMode = FEED_NONE;
  holguraPhase = HOLG_IDLE;
  holguraTimedEndMs = 0;
  holguraLastOn = holguraOkStable();
  if (holguraSawOn) {
    holguraPostCycleOffWatch = true;
    holguraSawOn = false;
  }
  if (!holguraOkStable())
    holguraOffSinceMs = nowMs;
  else
    holguraOffSinceMs = 0;
}

// Asentamiento, n s tras flanco ON y Tfeed ignoran el sensor a propósito.
static bool holguraOffFaultPaused() {
  if (feederMode == FEED_TFEED && feederDemand) return true;
  if (feederMode == FEED_HOLGURA &&
      (holguraPhase == HOLG_SETTLE || holguraPhase == HOLG_TIMED_FEED))
    return true;
  return false;
}

// Auto ON (también esperando in process): OFF estable > N s → fallo.
// Sigue contando en HOLG_WAIT_ON aunque el feeder esté girando.
static void serviceHolguraOffFault(uint32_t nowMs) {
  if (!P || fault != FAULT_NONE || !autoOn || idleMode) {
    holguraOffSinceMs = 0;
    return;
  }
  if (holguraOkStable() || holguraOffFaultPaused()) {
    holguraOffSinceMs = 0;
    return;
  }
  if (!holguraOffSinceMs) holguraOffSinceMs = nowMs;
  if ((nowMs - holguraOffSinceMs) >= (uint32_t)(P->holguraOffFaultSec * 1000.0f))
    enterFault(FAULT_HOLGURA);
}

// Orden aceptada o RPM real de la rampa. Cualquiera de las dos cuenta como en marcha.
static bool feederStepperMoving() {
  if (commandedFeederRpm > 0.5f) return true;
  return feeder && fabsf(feeder->currentRPM()) > 0.5f;
}

// Armado + holgura OFF estable + feeder quieto de verdad > T s → mismo fallo de holgura.
// No corre fuera de producción: ahí el motor está quieto a propósito y vale el timer de OFF.
static void serviceHolguraStallFault(uint32_t nowMs) {
  if (!P || fault != FAULT_NONE || !motionArmed() || holguraOkStable() || feederStepperMoving()) {
    holguraStallSinceMs = 0;
    return;
  }
  if (!holguraStallSinceMs) holguraStallSinceMs = nowMs;
  if ((nowMs - holguraStallSinceMs) >= (uint32_t)(P->holguraStallFaultSec * 1000.0f))
    enterFault(FAULT_HOLGURA);
}

static void serviceHolguraFeed(uint32_t nowMs) {
  if (feederMode != FEED_HOLGURA || !P) return;
  const bool on = holguraOkStable();
  const bool rising = on && !holguraLastOn;
  holguraLastOn = on;

  if (holguraPhase == HOLG_WAIT_ON) {
    if (rising || (on && !holguraSawOn)) holguraOnRisingEdge(nowMs);
    // Alimentar solo con holgura OFF hasta flanco ON (GPIO 22).
    feederDemand = !on;
    return;
  }

  if (holguraPhase == HOLG_SETTLE) {
    feederDemand = true;
    if (holguraSettleEndMs && (int32_t)(nowMs - holguraSettleEndMs) >= 0) {
      holguraPhase = HOLG_TIMED_FEED;
      holguraTimedEndMs = nowMs + (uint32_t)(P->holguraOnFeedSec * 1000.0f);
    }
    return;
  }

  if (holguraPhase == HOLG_TIMED_FEED) {
    feederDemand = true;
    if (holguraTimedEndMs && (int32_t)(nowMs - holguraTimedEndMs) >= 0)
      stopHolguraFeedCycle(nowMs);
    return;
  }

  holguraPhase = HOLG_WAIT_ON;
  feederDemand = !on;
}

// Idle solo con buffer full estable, holgura ON y el ciclo de asentamiento cerrado.
// Cortar en el flanco (settle / n s / Tfeed) deja el feeder a medias.
static bool settledForIdle() {
  if (!gSnap.bufFullStable || !holguraOkStable()) return false;
  if (feederDemand || feederMode != FEED_NONE) return false;
  return holguraPhase == HOLG_IDLE;
}

// Nivel ya filtrado: al reanudar, un sensor que sigue ON no es flanco nuevo.
static void syncBoostEdgeBaseline() {
  lastMid = gSnap.bufMidStable;
  lastHigh = gSnap.bufHighStable;
}

static void stopAutoOutputs() {
  releaseToIdlePending = false;
  stopFeederDemand();
  midBoostUntil = highBoostUntil = 0;
  highReverseActive = false;
  highReverseUntilMs = 0;
  highReverseCanArm = true;
  midPassedBeforeHigh = false;
  syncBoostEdgeBaseline();
  bufferEmptySinceMs = 0;
  OutputIntent o;
  outputsSetIntent(o);
  motorOutputsAllStop();
}

static void enterFault(SystemFault f) {
  fault = f;
  autoOn = false;
  stopAutoOutputs();
}

static int16_t boostServoDelta() {
  if (!P) return 0;
  if (highBoostUntil && (int32_t)(millis() - highBoostUntil) < 0) return P->highServoDeltaUs;
  if (midBoostUntil && (int32_t)(millis() - midBoostUntil) < 0) return P->midServoDeltaUs;
  return 0;
}

static float boostDereelerRpm(float base) {
  if (!P) return base;
  if (highBoostUntil && (int32_t)(millis() - highBoostUntil) < 0) return P->highBoostRpm;
  if (midBoostUntil && (int32_t)(millis() - midBoostUntil) < 0) return P->midBoostRpm;
  return base;
}

// Traduce estado lógico → OutputIntent (la tarea outputTask mueve motores)
static void syncOutputs() {
  OutputIntent o;
  if (refillActive() && idleMode) {
    o.mode = OUT_REFILL_MANUAL;
    o.feederRpm = (refillFd || refillMat) ? P->feederRpm : 0.0f;
    o.dereelerRpm = (refillDr || refillMat) ? P->dereelerRpm : 0.0f;
    o.servoRun = refillSv || refillMat;
    outputsSetIntent(o);
    return;
  }
  if (!motionArmed()) {
    outputsSetIntent(o);  // para salidas auto
    return;
  }

  o.mode = OUT_AUTO;
  o.feederRpm = feederDemand ? P->feederRpm : 0.0f;  // feeder independiente del buffer

  const bool bufOn = bufferWantsFill();
  if (bufOn) {  // mueve DeReeler + servo mientras buffer no lleno estable
    o.dereelerRpm = boostDereelerRpm(P->dereelerRpm);
    o.dereelerReverse = highReverseActive;
    o.servoRun = true;
    o.servoExtraUs = boostServoDelta();
  } else {  // buffer lleno estable → para DeReeler y servo
    o.dereelerRpm = 0.0f;
    o.servoRun = false;
  }
  outputsSetIntent(o);
}

// In process OFF: seguir sensores hasta full ON y holgura ON; ahí sí Idle.
static void requestProcessIdle() {
  benchInProcess = false;
  if (!inProcess) {
    stopAutoOutputs();
    return;
  }
  releaseToIdlePending = true;
}

static void serviceIdleRelease() {
  if (!releaseToIdlePending || idleMode || fault != FAULT_NONE) return;
  if (!settledForIdle()) return;
  inProcess = false;
  benchInProcess = false;
  stopAutoOutputs();
}

// Tfeed por tiempo (trigger HMI con holgura ya ON).
static void startTfeedDemand(uint32_t nowMs, float sec, uint16_t trigId) {
  if (sec < 0.05f) sec = P->triggerFeedS;
  triggerIdRemember(trigId);
  resetHolguraFeedState();
  feederMode = FEED_TFEED;
  feederDemand = true;
  feederMinEndMs = nowMs + (uint32_t)(sec * 1000.0f);
  pendingTrigger = false;
}

static void serviceTfeedDemand(uint32_t nowMs) {
  if (feederMode != FEED_TFEED || !feederDemand) return;
  if ((int32_t)(nowMs - feederMinEndMs) >= 0)
    stopFeederDemand();
}

static void serviceFeederDemand(uint32_t nowMs) {
  if (feederMode == FEED_TFEED)
    serviceTfeedDemand(nowMs);
  else if (feederMode == FEED_HOLGURA)
    serviceHolguraFeed(nowMs);
}

// Buffer mid/high estables: boost en flanco. HIGH invierte DeReeler solo si hubo MID antes.
// Secuencia: MID↑ (latch midPassedBeforeHigh), luego HIGH↑ → CCW durante high_reverse_s
// o hasta buffer full estable. MID se evalúa antes que HIGH en el mismo ciclo.
// No rearmar otro pulso invertido hasta que HIGH haya estado OFF.
static void serviceBoostEdges(uint32_t nowMs) {
  if (!P) return;
  if (!bufferWantsFill()) {
    highReverseActive = false;
    highReverseUntilMs = 0;
    highReverseCanArm = true;
    midPassedBeforeHigh = false;
    syncBoostEdgeBaseline();
    return;
  }
  if (!gSnap.bufHighStable)
    highReverseCanArm = true;
  if (gSnap.bufMidStable && !lastMid) {
    midBoostUntil = nowMs + (uint32_t)(P->midBoostSec * 1000.0f);
    midPassedBeforeHigh = true;
  }
  if (gSnap.bufHighStable && !lastHigh) {
    highBoostUntil = nowMs + (uint32_t)(P->highBoostSec * 1000.0f);
    if (midPassedBeforeHigh && highReverseCanArm && !highReverseActive) {
      highReverseActive = true;
      highReverseUntilMs = nowMs + (uint32_t)(P->highReverseSec * 1000.0f);
      midPassedBeforeHigh = false;
      highReverseCanArm = false;
    }
  }
  if (highReverseActive) {
    const bool timerDone = highReverseUntilMs
                           && (int32_t)(nowMs - highReverseUntilMs) >= 0;
    if (gSnap.bufFullStable || timerDone) {
      highReverseActive = false;
      highReverseUntilMs = 0;
    }
  }
  syncBoostEdgeBaseline();
}

// Espera buffer_refill_fault_s sin ver GPIO19 estable lleno → falla timeout
static void serviceBufferTimeout(uint32_t nowMs) {
  if (!bufferWantsFill()) {
    bufferEmptySinceMs = 0;
    return;
  }
  if (!bufferEmptySinceMs) bufferEmptySinceMs = nowMs;
  if ((nowMs - bufferEmptySinceMs) > (uint32_t)(P->bufferFillTimeoutS * 1000.0f))
    enterFault(FAULT_BUFFER_TIMEOUT);
}

// Max / manguera / cilindro (auto; Materialista excluye según idle).
static void serviceProtections() {
  if (!motionArmed() && !refillActive()) return;
  if (gSnap.bufMaxRaw) enterFault(FAULT_ENDSTOP);
  if (!idleMode) {
    if (gSnap.hoseAbsent) enterFault(FAULT_HOSE);
    if (gSnap.cylinderOpen) enterFault(FAULT_CYLINDER);
  }
}

void machineInit(Params* params) {
  P = params;
  commandsInit();
  sensorsInitPins();
  outputsInit();
  xTaskCreatePinnedToCore(outputTask, "outputs", 4096, nullptr, 2, nullptr, 1);
  fault = FAULT_NONE;
}

// Comandos encolados desde web/TCP (no mueven GPIO directo)
void machineHandleCommand(const NetCommand& cmd) {
  switch (cmd.type) {
    case NC_START:
      // Materialista: solo refill manual; no armar auto desde TCP/HTML local.
      if (fault == FAULT_NONE && !idleMode) autoOn = true;
      break;
    case NC_STOP:
      fault = FAULT_OPERATOR;
      autoOn = false;
      stopAutoOutputs();
      break;
    case NC_RESET:
      fault = FAULT_NONE;
      autoOn = false;
      idleMode = false;
      inProcess = false;
      benchInProcess = false;
      stopAutoOutputs();
      // machineTick (mismo loop) puede reentrar si la causa sigue. El status
      // forzado sale después, con este epoch y el fault ya definitivo.
      faultEpoch = faultEpoch == 65535 ? 1 : (uint16_t)(faultEpoch + 1);
      peerStatusForce = true;
      break;
    case NC_TRIGGER:  // encola Tfeed; machineTick → startFeederDemand
      if (fault != FAULT_NONE || !autoOn) break;
      if (cmd.id && triggerIdDone((uint16_t)cmd.id)) break;
      pendingTrigger = true;
      pendingTriggerId = (uint16_t)cmd.id;
      pendingTriggerSec = cmd.arg.length() ? cmd.arg.toFloat() : 0.0f;
      break;
    case NC_IN_PROCESS:  // OFF no corta: Idle = full ON + holgura ON
      if (cmd.arg == "1" || cmd.arg == "true" || cmd.arg == "on") {
        inProcess = true;
        releaseToIdlePending = false;
        benchInProcess = false;
      } else
        requestProcessIdle();
      break;
    case NC_BENCH_IN_PROCESS:  // HTML local: simular En proceso (banco)
      benchInProcess = (cmd.arg == "1" || cmd.arg == "true" || cmd.arg == "on");
      if (benchInProcess) {
        idleMode = false;
        inProcess = true;
        releaseToIdlePending = false;
        if (fault == FAULT_NONE) autoOn = true;
      } else {
        inProcess = false;
        autoOn = false;
        stopAutoOutputs();
      }
      break;
    case NC_IDLE_MODE:  // Materialista: solo refill manual
      idleMode = (cmd.arg == "1" || cmd.arg == "true" || cmd.arg == "on");
      if (idleMode) {
        autoOn = false;
        inProcess = false;
        benchInProcess = false;
        stopAutoOutputs();
      }
      break;
    case NC_SET_TRIGGER_FEED:
      P->triggerFeedS = cmd.arg.toFloat();
      paramsValidate(*P);
      paramsSave(*P);
      break;
    case NC_SET_HMI_TFEED:
      P->hmiTfeedEnabled = (cmd.arg == "1" || cmd.arg == "true" || cmd.arg == "on");
      paramsValidate(*P);
      paramsSave(*P);
      break;
    case NC_SET_REFILL_PULSE:
      P->refillPulseMs = (uint32_t)(cmd.arg.toFloat() * 1000.0f);
      paramsValidate(*P);
      paramsSave(*P);
      break;
    case NC_REFILL:
    case NC_REFILL_HOLD: {
      if (!idleMode) break;
      const bool hold = cmd.type == NC_REFILL_HOLD;
      const bool on = cmd.refillOn;
      if (cmd.refillChannel == "material") {
        refillMat = on;
        if (!hold && on) refillPulseUntil[0] = millis() + P->refillPulseMs;
        else if (hold || !on) refillPulseUntil[0] = 0;
      } else if (cmd.refillChannel == "dereeler") {
        refillDr = on;
        if (!hold && on) refillPulseUntil[1] = millis() + P->refillPulseMs;
        else if (hold || !on) refillPulseUntil[1] = 0;
      } else if (cmd.refillChannel == "servo") {
        refillSv = on;
        if (!hold && on) refillPulseUntil[2] = millis() + P->refillPulseMs;
        else if (hold || !on) refillPulseUntil[2] = 0;
      } else if (cmd.refillChannel == "feeder") {
        refillFd = on;
        if (!hold && on) refillPulseUntil[3] = millis() + P->refillPulseMs;
        else if (hold || !on) refillPulseUntil[3] = 0;
      }
      syncOutputs();
      break;
    }
    case NC_HALT:
      inProcess = false;
      refillMat = refillDr = refillSv = refillFd = false;
      stopAutoOutputs();
      break;
    default:
      break;
  }
}

// Un paso del ciclo de máquina (~cada loop + delay 2 ms)
void machineTick(uint32_t nowMs) {
  sensorsUpdate(nowMs);  // lee GPIO + estabilidad buffer

  for (int i = 0; i < 4; i++) {  // espera refill_pulse_s → apaga canal Materialista
    if (refillPulseUntil[i] && (int32_t)(nowMs - refillPulseUntil[i]) >= 0) {
      refillPulseUntil[i] = 0;
      if (i == 0) refillMat = false;
      if (i == 1) refillDr = false;
      if (i == 2) refillSv = false;
      if (i == 3) refillFd = false;
    }
  }

  serviceProtections();

  if (fault == FAULT_NONE && motionArmed()) {
    if (holguraPostCycleOffWatch && holguraOkStable()) {
      holguraPostCycleOffWatch = false;
      holguraOffSinceMs = 0;
    }
    if (pendingTrigger && !feederDemand) {
      float sec = pendingTriggerSec > 0.05f ? pendingTriggerSec : P->triggerFeedS;
      const uint16_t tid = pendingTriggerId;
      pendingTrigger = false;
      pendingTriggerId = 0;
      pendingTriggerSec = 0.0f;
      if (P->hmiTfeedEnabled && holguraOkStable())
        startTfeedDemand(nowMs, sec, tid);
      else if (!holguraOkStable()) {
        holguraActiveTriggerId = tid;
        beginHolguraFeed(nowMs);
      } else if (tid)
        triggerIdRemember(tid);  // holgura ya ON: no arrancar ciclo OFF→ON
    // Holgura OFF siempre rearma feeder aunque postCycleOffWatch esté activo (ciclo
    // terminó con holgura OFF tras haber visto ON): omitirlo dejaba solo buffer/DeReeler.
    } else if (!feederDemand && !holguraOkStable())
      beginHolguraFeed(nowMs);
    serviceFeederDemand(nowMs);
    serviceBoostEdges(nowMs);     // impulsos mid/high + inversión temporizada
    serviceBufferTimeout(nowMs);  // espera llenado buffer
  } else {
    syncBoostEdgeBaseline();  // sin proceso: seguir el nivel, no inventar flancos
  }

  serviceIdleRelease();  // Idle diferido: full ON + holgura ON y ciclo cerrado
  serviceHolguraOffFault(nowMs);  // Auto ON + holgura OFF > N s, con o sin in process
  serviceHolguraStallFault(nowMs);  // Armado + OFF + feeder quieto > T s
  syncOutputs();  // actualiza intención → outputTask mueve hardware
}

const SensorSnapshot& machineSensors() { return gSnap; }

MachineStatus machineStatus() {
  MachineStatus s;
  s.fault = fault;
  s.autoEnabled = autoOn;
  s.idleMode = idleMode;
  s.inProcess = inProcess;
  s.benchInProcess = benchInProcess;
  s.feederActive = feederDemand && motionArmed();
  s.bufferActive = bufferWantsFill();
  s.triggerActive = s.feederActive;
  if (feederDemand) {
    uint32_t endMs = feederMode == FEED_TFEED ? feederMinEndMs : holguraTimedEndMs;
    if (endMs) {
      int32_t left = (int32_t)(endMs - millis());
      s.feederMsLeft = left > 0 ? (uint32_t)left : 0;
    }
  }
  return s;
}

uint8_t machineFaultWireCode() {
  if (fault == FAULT_NONE) return 0;
  uint8_t id = 0;
  switch (fault) {
    case FAULT_ENDSTOP: id = 1; break;
    case FAULT_TENSION: id = 2; break;
    case FAULT_CYLINDER: id = 3; break;
    case FAULT_HOSE: id = 4; break;
    case FAULT_BUFFER_TIMEOUT: id = 5; break;
    case FAULT_HOLGURA: id = 6; break;
    default: return 0;
  }
  return PF_FAULT_WIRE_BASE + id;
}

bool machinePushCommand(const NetCommand& cmd) { return commandsPush(cmd); }

// --- Wi-Fi, HTTP, TCP peer, OTA ---

static WebServer server(80);
static WiFiServer peerServer(PEER_TCP_PORT);
static WiFiClient peerClient;
static uint32_t peerLastStatusMs = 0;
static bool peerLinkOk = false;
static bool peerUp = false;
static EspSta staLink;
static char peerHoldBuf[1500];
static EspTcpHold peerHold = {peerHoldBuf, sizeof(peerHoldBuf), 0};

static String jStr(const char* j, const char* k) {
  String n = String("\"") + k + "\":\"";
  String s(j);
  int i = s.indexOf(n);
  if (i < 0) return "";
  int a = i + n.length(), b = s.indexOf('"', a);
  return b < 0 ? "" : s.substring(a, b);
}

static int jInt(const char* j, const char* k, int d) {
  String n = String("\"") + k + "\":";
  int i = String(j).indexOf(n);
  return i < 0 ? d : String(j).substring(i + n.length()).toInt();
}

static void peerNoteDead() {
  espTcpStop(peerClient);
  espTcpHoldClear(peerHold);
  peerLinkOk = false;
}

static void peerTx(const String& m) {
  if (!peerClient.connected()) {
    if (peerClient) peerNoteDead();
    return;
  }
  if (peerHold.len && !espTcpHoldFlush(peerClient, peerHold)) {
    if (!peerClient.connected()) peerNoteDead();
    return;
  }
  if (!m.length() || !espTcpHoldPut(peerHold, m.c_str())) return;
  if (!espTcpHoldFlush(peerClient, peerHold) && !peerClient.connected())
    peerNoteDead();
}

static void dispatchPeerJson(const String& line) {
  if (line.indexOf("\"type\":\"command\"") < 0) return;
  String cmd = jStr(line.c_str(), "command");
  String val = jStr(line.c_str(), "value");
  int id = jInt(line.c_str(), "id", 0);
  if (cmd == "ping") {
    peerTx("{\"ver\":1,\"type\":\"pong\"}");
    return;
  }
  NetCommand nc;
  if (cmd == "start") nc.type = NC_START;
  else if (cmd == "stop") nc.type = NC_STOP;
  else if (cmd == "reset") nc.type = NC_RESET;
  else if (cmd == "trigger") {
    nc.type = NC_TRIGGER;
    nc.arg = val;
    nc.id = id;
  } else if (cmd == "setInProcess" || cmd == "inProcess") {
    nc.type = NC_IN_PROCESS;
    nc.arg = val;
  } else if (cmd == "setIdleMode" || cmd == "idleMode" || cmd == "materialistaCall"
             || cmd == "setMaterialistaCall" || cmd == "setTestMode" || cmd == "testMode") {
    nc.type = NC_IDLE_MODE;
    nc.arg = val;
  } else if (cmd == "setTriggerFeed" || cmd == "setTriggerCfg") {
    nc.type = NC_SET_TRIGGER_FEED;
    nc.arg = val;
  } else if (cmd == "setHmiTfeedEnabled" || cmd == "setTfeedEnabled") {
    nc.type = NC_SET_HMI_TFEED;
    nc.arg = val;
  } else if (cmd == "setRefillPulseS" || cmd == "refillPulseS") {
    nc.type = NC_SET_REFILL_PULSE;
    nc.arg = val;
  } else if (cmd == "halt" || cmd == "motionStop" || cmd == "hardStop") nc.type = NC_HALT;
  else if (cmd.startsWith("refill")) {
    nc.type = cmd.startsWith("refillHold") ? NC_REFILL_HOLD : NC_REFILL;
    nc.refillOn = (val == "1" || val == "true" || val == "on");
    if (cmd.indexOf("Dereeler") >= 0) nc.refillChannel = "dereeler";
    else if (cmd.indexOf("Servo") >= 0) nc.refillChannel = "servo";
    else if (cmd.indexOf("Feeder") >= 0) nc.refillChannel = "feeder";
    else nc.refillChannel = "material";
  } else return;
  machinePushCommand(nc);
}

static String buildPeerStatus(const MachineStatus& c, const SensorSnapshot& s) {
  String j;
  j.reserve(2000);
  j += "{\"ver\":1,\"type\":\"status\",\"role\":\"";
  j += PF_ROLE;
  j += "\",\"side\":\"";
  j += PF_SIDE_TAG;
  j += "\",\"idleMode\":";
  j += c.idleMode ? "true" : "false";
  j += ",\"inProcess\":";
  j += c.inProcess ? "true" : "false";
  j += ",\"machineState\":\"";
  if (c.idleMode) j += "materialist";
  else if (c.feederActive || c.bufferActive) j += "in_process";
  else if (c.inProcess) j += "production";
  else j += "idle";
  j += "\",\"sensorsArmed\":";
  j += (c.autoEnabled && c.inProcess && !c.idleMode) ? "true" : "false";
  j += ",\"home\":";
  j += s.bufFullStable ? "true" : "false";
  j += ",\"mid\":";
  j += s.bufMidRaw ? "true" : "false";
  j += ",\"high\":";
  j += s.bufHighRaw ? "true" : "false";
  j += ",\"endstop\":";
  j += s.bufMaxRaw ? "true" : "false";
  j += ",\"cylinderOpen\":";
  j += s.cylinderOpen ? "true" : "false";
  j += ",\"hoseAbsent\":";
  j += s.hoseAbsent ? "true" : "false";
  j += ",\"holgura\":";
  j += s.holguraOn ? "true" : "false";
  j += ",\"error\":";
  j += c.fault != FAULT_NONE ? "true" : "false";
  j += ",\"errorCode\":";
  j += String(machineFaultWireCode());
  j += ",\"faultEpoch\":";
  j += String(faultEpoch);
  j += ",\"autoEnabled\":";
  j += c.autoEnabled ? "true" : "false";
  j += ",\"feederActive\":";
  j += c.feederActive ? "true" : "false";
  j += ",\"bufferActive\":";
  j += c.bufferActive ? "true" : "false";
  j += ",\"triggerActive\":";
  j += c.triggerActive ? "true" : "false";
  j += ",\"triggerMsLeft\":";
  j += String(c.feederMsLeft);
  j += ",\"triggerFeedS\":";
  j += String(P->triggerFeedS, 2);
  j += ",\"autoRpm\":";
  j += String(P->dereelerRpm, 1);
  j += ",\"motor2Rpm\":";
  j += String(P->feederRpm, 1);
  j += ",\"refillMaterial\":";
  j += refillMat ? "true" : "false";
  j += ",\"refillDereeler\":";
  j += refillDr ? "true" : "false";
  j += ",\"refillServo\":";
  j += refillSv ? "true" : "false";
  j += ",\"refillFeeder\":";
  j += refillFd ? "true" : "false";
  j += "}";
  return j;
}

static void ensurePeerServer() {
  const uint8_t ev = espStaPoll(staLink);
  if (ev & ESP_STA_UP)
    Serial.printf("[PF] WiFi On %s\n", WiFi.localIP().toString().c_str());
  if (ev & ESP_STA_BLIP)
    Serial.println("[PF] WiFi blip — esperando auto-reconexion");
  if (ev & ESP_STA_DOWN) {
    peerNoteDead();
    if (peerUp) {
      peerServer.end();
      peerUp = false;
    }
    Serial.println("[PF] WiFi Off");
  }
  if (ev & ESP_STA_KICK) {
    peerNoteDead();
    if (peerUp) {
      peerServer.end();
      peerUp = false;
    }
    const IPAddress ip(PF_STA_IP_0, PF_STA_IP_1, PF_STA_IP_2, PF_STA_IP_3);
    const IPAddress gw(STA_GW_0, STA_GW_1, STA_GW_2, STA_GW_3);
    const IPAddress mask(STA_MASK_0, STA_MASK_1, STA_MASK_2, STA_MASK_3);
    if (!espStaKick(staLink, PF_OTA_HOST, WIFI_SSID, WIFI_PASS, ip, gw, mask, gw))
      Serial.println("[PF] WiFi.config() fallo");
    Serial.println("[PF] Reintentando STA...");
  }
  if (!staLink.wasUp || WiFi.status() != WL_CONNECTED) return;
  if (!peerUp) {
    peerServer.begin();
    peerServer.setNoDelay(true);
    peerUp = true;
  }
  if (peerServer.hasClient()) {
    WiFiClient incoming = peerServer.available();
    if (incoming) {
      peerNoteDead();
      peerClient = incoming;
      peerClient.setNoDelay(true);
      espTcpKeepAlive(peerClient);
      espTcpHoldClear(peerHold);
      peerTx(String("{\"ver\":1,\"type\":\"hello\",\"role\":\"") + PF_ROLE + "\",\"side\":\""
             + PF_SIDE_TAG + "\"}");
    }
  }
  if (peerHold.len && peerClient.connected() &&
      !espTcpHoldFlush(peerClient, peerHold) && !peerClient.connected())
    peerNoteDead();
  peerLinkOk = peerClient.connected();
}

static void appendJsonFloat(String& j, float v, uint8_t dec = 1) {
  char b[20];
  snprintf(b, sizeof(b), dec == 2 ? "%.2f" : "%.1f", v);
  j += b;
}

static void sendStatusJson(bool paramsSaved = false) {
  const SensorSnapshot& s = machineSensors();
  MachineStatus c = machineStatus();
  String json;
  json.reserve(4096);
  json += "{\"side\":\"";
  json += PF_SIDE_TAG;
  json += "\",\"home\":{\"raw\":";
  json += s.bufFullRaw ? "true" : "false";
  json += ",\"stable\":";
  json += s.bufFullStable ? "true" : "false";
  json += "},\"auto\":{\"enabled\":";
  json += c.autoEnabled ? "true" : "false";
  json += ",\"rpm\":";
  appendJsonFloat(json, P->dereelerRpm);
  json += ",\"servo_pwm_us\":";
  json += String(P->servoActiveUs);
  json += ",\"buffer_refill_fault_s\":";
  appendJsonFloat(json, P->bufferFillTimeoutS);
  json += ",\"mid_boost_rpm\":";
  appendJsonFloat(json, P->midBoostRpm);
  json += ",\"mid_boost_s\":";
  appendJsonFloat(json, P->midBoostSec);
  json += ",\"mid_servo_delta_us\":";
  json += String((int)P->midServoDeltaUs);
  json += ",\"high_boost_rpm\":";
  appendJsonFloat(json, P->highBoostRpm);
  json += ",\"high_boost_s\":";
  appendJsonFloat(json, P->highBoostSec);
  json += ",\"high_reverse_s\":";
  appendJsonFloat(json, P->highReverseSec);
  json += ",\"high_servo_delta_us\":";
  json += String((int)P->highServoDeltaUs);
  json += "},\"trigger2\":{\"trigger_feed_s\":";
  appendJsonFloat(json, P->triggerFeedS, 2);
  json += ",\"m2_rpm\":";
  appendJsonFloat(json, P->feederRpm);
  json += ",\"trigger_active\":";
  json += c.triggerActive ? "true" : "false";
  json += ",\"hmi_tfeed_enabled\":";
  json += P->hmiTfeedEnabled ? "true" : "false";
  json += ",\"holgura_settle_ms\":";
  json += String((unsigned)P->holguraSettleMs);
  json += ",\"holgura_on_s\":";
  appendJsonFloat(json, P->holguraOnFeedSec, 2);
  json += ",\"holgura_off_fault_s\":";
  appendJsonFloat(json, P->holguraOffFaultSec, 2);
  json += ",\"holgura_stall_fault_s\":";
  appendJsonFloat(json, P->holguraStallFaultSec, 2);
  json += ",\"holgura_on\":";
  json += s.holguraOn ? "true" : "false";
  json += "},\"refill\":{\"pulse_s\":";
  appendJsonFloat(json, P->refillPulseMs / 1000.0f);
  json += "},\"actuators\":{\"feeder\":";
  json += c.feederActive ? "true" : "false";
  json += ",\"buffer\":";
  json += c.bufferActive ? "true" : "false";
  json += ",\"feeder_ms_left\":";
  json += String(c.feederMsLeft);
  json += "},\"sensors\":{";
  json += "\"mid\":";
  json += s.bufMidRaw ? "true" : "false";
  json += ",\"high\":";
  json += s.bufHighRaw ? "true" : "false";
  json += ",\"max\":";
  json += s.bufMaxRaw ? "true" : "false";
  json += ",\"cylinder\":";
  json += s.cylinderOpen ? "true" : "false";
  json += ",\"hose\":";
  json += s.hoseAbsent ? "true" : "false";
  json += ",\"holgura\":";
  json += s.holguraOn ? "true" : "false";
  json += "},\"fault\":";
  json += String((unsigned)c.fault);
  json += ",\"idleMode\":";
  json += c.idleMode ? "true" : "false";
  json += ",\"inProcess\":";
  json += c.inProcess ? "true" : "false";
  json += ",\"benchInProcess\":";
  json += c.benchInProcess ? "true" : "false";
  json += ",\"peer\":{\"link\":";
  json += peerLinkOk ? "true" : "false";
  json += "}";
  json += ",\"manualRefill\":{\"material\":";
  json += refillMat ? "true" : "false";
  json += ",\"dereeler\":";
  json += refillDr ? "true" : "false";
  json += ",\"servo\":";
  json += refillSv ? "true" : "false";
  json += ",\"feeder\":";
  json += refillFd ? "true" : "false";
  json += "}";
  if (paramsSaved) json += ",\"save\":{\"ok\":true}";
  json += "}";
  server.send(200, "application/json", json);
}

static String httpQueryArgs() {
  String q;
  for (int i = 0; i < server.args(); i++) {
    if (q.length()) q += "&";
    q += server.argName(i) + "=" + server.arg(i);
  }
  return q;
}

static void handleConfig() {
  const bool saved = paramsApplyFromQuery(httpQueryArgs(), *P, true);
  sendStatusJson(saved);
}

static void handleAuto() {
  if (server.hasArg("enable")) {
    const bool on = server.arg("enable") == "1";
    if (on) {
      const MachineStatus st = machineStatus();
      if (!st.idleMode && !st.inProcess) {
        server.send(409, "application/json",
                    "{\"ok\":false,\"error\":\"materialist_or_in_process\"}");
        return;
      }
    }
    NetCommand nc;
    nc.type = on ? NC_START : NC_STOP;
    machinePushCommand(nc);
  }
  if (server.hasArg("reset")) {
    NetCommand nc;
    nc.type = NC_RESET;
    machinePushCommand(nc);
  }
  if (server.hasArg("bench")) {
    NetCommand nc;
    nc.type = NC_BENCH_IN_PROCESS;
    nc.arg = (server.arg("bench") == "1") ? "1" : "0";
    machinePushCommand(nc);
  }
  if (server.hasArg("trigger")) {
    NetCommand nc;
    nc.type = NC_TRIGGER;
    nc.arg = server.hasArg("sec") ? server.arg("sec") : "";
    nc.id = 0;
    machinePushCommand(nc);
  }
  String q;
  for (int i = 0; i < server.args(); i++) {
    const String& n = server.argName(i);
    if (n == "enable" || n == "reset" || n == "bench" || n == "trigger" || n == "sec")
      continue;
    if (q.length()) q += "&";
    q += n + "=" + server.arg(i);
  }
  bool saved = false;
  if (q.length()) saved = paramsApplyFromQuery(q, *P, true);
  sendStatusJson(saved);
}

static void handleRefill() {
  if (!machineStatus().idleMode) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"idle_required\"}");
    return;
  }
  const bool holdMode = server.hasArg("hold") && server.arg("hold") == "1";
  NetCommand nc;
  nc.type = holdMode ? NC_REFILL_HOLD : NC_REFILL;
  if (server.hasArg("pulse_s") || server.hasArg("refill_pulse_s")) {
    const String& raw = server.hasArg("pulse_s") ? server.arg("pulse_s") : server.arg("refill_pulse_s");
    paramsApplyFromQuery(String("refill_pulse_s=") + raw, *P, true);
    sendStatusJson(true);
    return;
  }
  if (server.hasArg("material")) {
    nc.refillChannel = "material";
    nc.refillOn = server.arg("material") == "1";
  } else if (server.hasArg("dereeler")) {
    nc.refillChannel = "dereeler";
    nc.refillOn = server.arg("dereeler") == "1";
  } else if (server.hasArg("servo")) {
    nc.refillChannel = "servo";
    nc.refillOn = server.arg("servo") == "1";
  } else if (server.hasArg("feeder")) {
    nc.refillChannel = "feeder";
    nc.refillOn = server.arg("feeder") == "1";
  }
  machinePushCommand(nc);
  server.send(200, "application/json", "{\"ok\":true}");
}

void pfNetworkInit(Params* params) {
  P = params;
  espStaClear(staLink);
  const IPAddress ip(PF_STA_IP_0, PF_STA_IP_1, PF_STA_IP_2, PF_STA_IP_3);
  const IPAddress gw(STA_GW_0, STA_GW_1, STA_GW_2, STA_GW_3);
  const IPAddress mask(STA_MASK_0, STA_MASK_1, STA_MASK_2, STA_MASK_3);
  if (!espStaKick(staLink, PF_OTA_HOST, WIFI_SSID, WIFI_PASS, ip, gw, mask, gw))
    Serial.println("[PF] WiFi.config() fallo");

  server.on("/", []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/html", FPSTR(PF_PAGE_HTML));
  });
  server.on("/api/status", []() { sendStatusJson(); });
  server.on("/api/config", handleConfig);
  server.on("/api/auto", handleAuto);
  server.on("/api/refill", handleRefill);
  server.on("/ping", []() { server.send(200, "text/plain", "pong"); });
  server.begin();

  ArduinoOTA.setHostname(PF_OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    NetCommand h;
    h.type = NC_HALT;
    machinePushCommand(h);
  });
  ArduinoOTA.begin();
}

void pfNetworkTick(uint32_t nowMs) {
  server.handleClient();
  ArduinoOTA.handle();
  ensurePeerServer();
  MachineStatus c = machineStatus();
  const SensorSnapshot& s = machineSensors();
  if (peerClient) {
    static String line;
    while (peerClient.available()) {
      char ch = (char)peerClient.read();
      if (ch == '\n') {
        if (line.length()) dispatchPeerJson(line);
        line = "";
      } else if (ch != '\r' && line.length() < 900)
        line += ch;
    }
    if (!peerClient.connected()) {
      line = "";
      peerNoteDead();
    } else {
      uint32_t iv = (c.feederActive || c.bufferActive) ? PEER_STATUS_FAST_MS : PEER_STATUS_MS;
      if (peerStatusForce || nowMs - peerLastStatusMs >= iv) {
        peerTx(buildPeerStatus(c, s));
        peerLastStatusMs = nowMs;
        peerStatusForce = false;
      }
    }
  }
}

static Params gParams;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\nPF_V2 %s  %d.%d.%d.%d  fw %s\n", PF_SIDE_TAG,
                PF_STA_IP_0, PF_STA_IP_1, PF_STA_IP_2, PF_STA_IP_3, FW_VERSION);
  paramsLoad(gParams);
  machineInit(&gParams);
  pfNetworkInit(&gParams);
}

void loop() {
  const uint32_t now = millis();
  NetCommand cmd;
  while (machinePopCommand(cmd))  // drena cola comandos (web / peer)
    machineHandleCommand(cmd);
  machineTick(now);      // ciclo planta
  pfNetworkTick(now);    // red
  delay(2);              // cede CPU (~500 Hz máx. loop lógica)
}
