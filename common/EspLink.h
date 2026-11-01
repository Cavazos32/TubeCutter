#pragma once

// Enlace STA + TCP igual en todos los ESP (PLC, Motion, Andon, Master, PreFeeder).
// Keepalive: mismos tiempos que HMI/core/tcp_link.py en Linux (3 s / 1 s / 3).
// Un write corto no cierra el socket: el sufijo se reintenta en la vuelta siguiente.
// EAGAIN no es caída. El fd se cierra cuando connected() ya es falso.

#include <WiFi.h>
#include <lwip/sockets.h>
#include <string.h>

static constexpr int ESP_TCP_KEEPIDLE_S = 3;
static constexpr int ESP_TCP_KEEPINTVL_S = 1;
static constexpr int ESP_TCP_KEEPCNT = 3;

static constexpr uint32_t ESP_WIFI_DOWN_GRACE_MS = 2500;
static constexpr uint32_t ESP_WIFI_RETRY_MS = 5000;
static constexpr uint32_t ESP_WIFI_ATTEMPT_MS = 30000;

struct EspSta {
  bool wasUp;
  uint32_t downSinceMs;
  uint32_t connectStartedMs;
  uint32_t lastKickMs;
};

enum EspStaEvent : uint8_t {
  ESP_STA_NONE = 0,
  ESP_STA_UP = 1,
  ESP_STA_DOWN = 2,
  ESP_STA_KICK = 4,
  ESP_STA_BLIP = 8,
};

inline void espStaClear(EspSta& s)
{
  s.wasUp = false;
  s.downSinceMs = 0;
  s.connectStartedMs = 0;
  s.lastKickMs = 0;
}

inline uint8_t espStaPoll(EspSta& s)
{
  const uint32_t now = millis();
  const wl_status_t st = WiFi.status();
  uint8_t ev = ESP_STA_NONE;

  if (st == WL_CONNECTED) {
    s.downSinceMs = 0;
    if (!s.wasUp) {
      s.wasUp = true;
      s.connectStartedMs = 0;
      ev |= ESP_STA_UP;
    }
    return ev;
  }

  if (s.wasUp) {
    if (s.downSinceMs == 0) {
      s.downSinceMs = now;
      return ESP_STA_BLIP;
    }
    if ((uint32_t)(now - s.downSinceMs) < ESP_WIFI_DOWN_GRACE_MS)
      return ESP_STA_NONE;
    s.wasUp = false;
    s.downSinceMs = 0;
    s.connectStartedMs = 0;
    s.lastKickMs = 0;
    ev |= ESP_STA_DOWN;
  }

  if (s.lastKickMs != 0 && (uint32_t)(now - s.lastKickMs) < ESP_WIFI_RETRY_MS)
    return ev;

  const bool attemptExpired =
      s.connectStartedMs != 0 &&
      (uint32_t)(now - s.connectStartedMs) >= ESP_WIFI_ATTEMPT_MS;
  const bool hardFail =
      st == WL_NO_SSID_AVAIL || st == WL_CONNECT_FAILED || st == WL_CONNECTION_LOST;
  const bool idle = s.connectStartedMs == 0;
  if (idle || attemptExpired || hardFail)
    ev |= ESP_STA_KICK;
  return ev;
}

inline bool espStaKick(EspSta& s, const char* hostname,
                       const char* ssid, const char* pass,
                       const IPAddress& ip, const IPAddress& gw,
                       const IPAddress& mask, const IPAddress& dns)
{
  WiFi.persistent(false);
  WiFi.disconnect(false);
  WiFi.mode(WIFI_STA);
  if (hostname && hostname[0]) WiFi.setHostname(hostname);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  const bool cfgOk = WiFi.config(ip, gw, mask, dns);
  const uint32_t now = millis();
  s.connectStartedMs = now;
  s.lastKickMs = now;
  WiFi.begin(ssid, pass);
  return cfgOk;
}

inline void espTcpKeepAlive(WiFiClient& client)
{
  const int fd = client.fd();
  if (fd < 0) return;
  const int on = 1;
  setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
#if defined(TCP_KEEPIDLE)
  const int idle = ESP_TCP_KEEPIDLE_S;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
#elif defined(TCP_KEEPALIVE)
  const int idle = ESP_TCP_KEEPIDLE_S;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPALIVE, &idle, sizeof(idle));
#endif
#ifdef TCP_KEEPINTVL
  const int intvl = ESP_TCP_KEEPINTVL_S;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
#endif
#ifdef TCP_KEEPCNT
  const int cnt = ESP_TCP_KEEPCNT;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
}

inline void espTcpStop(WiFiClient& client)
{
  if (client) client.stop();
}

// true cuando no queda nada por enviar.
// Si el stack escribe un prefijo, buf/len quedan en el sufijo.
inline bool espTcpWriteAll(WiFiClient& client, char* buf, size_t* len)
{
  if (!buf || !len || *len == 0) return true;
  if (!client.connected()) return false;
  const size_t n = *len;
  const int room = client.availableForWrite();
  if (room > 0 && (size_t)room < n) return false;
  const size_t sent = client.write(reinterpret_cast<const uint8_t*>(buf), n);
  if (sent == 0) return false;
  if (sent >= n) {
    *len = 0;
    buf[0] = 0;
    return true;
  }
  memmove(buf, buf + sent, n - sent);
  *len = n - sent;
  buf[*len] = 0;
  return false;
}

struct EspTcpHold {
  char* buf;
  size_t cap;
  size_t len;
};

inline void espTcpHoldClear(EspTcpHold& h)
{
  h.len = 0;
  if (h.buf && h.cap) h.buf[0] = 0;
}

inline bool espTcpHoldPut(EspTcpHold& h, const char* msg)
{
  if (!h.buf || h.cap < 2 || !msg || h.len) return false;
  size_t n = strlen(msg);
  const bool nl = n && msg[n - 1] == '\n';
  if (n + (nl ? 1u : 2u) > h.cap) return false;
  memcpy(h.buf, msg, n);
  if (!nl) h.buf[n++] = '\n';
  h.buf[n] = 0;
  h.len = n;
  return true;
}

inline bool espTcpHoldFlush(WiFiClient& client, EspTcpHold& h)
{
  if (!h.len) return true;
  return espTcpWriteAll(client, h.buf, &h.len);
}
