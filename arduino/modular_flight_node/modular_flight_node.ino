#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

// ============================================================================
// 1. 模块配置区（烧录不同模块时只需修改 MY_MODULE_ID）
// ============================================================================
#define MY_MODULE_ID           1     // 当前模块编号: 1 ~ MAX_MODULES
#define EXPECTED_MODULE_COUNT  2     // 参与合推力融合的模块总数
#define MAX_MODULES            8

// 串口引脚定义（与硬件接线保持一致）
#define FC_RX_PIN              D7
#define FC_TX_PIN              D6
#define DEBUG_BAUD             115200
#define FC_BAUD                115200

// ESP-NOW 通信配置（飞行中纯模块间直连，无需路由器与电脑）
#define ESPNOW_CHANNEL         1
#define FUSION_LOOP_PERIOD_MS  20    // 50Hz 读取飞控并广播 v_i
#define MODULE_TIMEOUT_MS      150   // 超过 150ms 未收到某模块数据则判为超时
#define THRUST_LPF_ALPHA       0.25f // 合推力一阶低通滤波系数 (0~1, 越小越平滑)
#define LOG_PRINT_PERIOD_MS    100   // 10Hz 打印串口调试信息

// MSPv1 协议常量
static const uint8_t MSP_API_VERSION = 1;
static const uint8_t MSP_ATTITUDE    = 108;
static const uint8_t MSP_WP          = 118;
static const uint8_t MSP_SET_WP      = 209;

// ============================================================================
// 2. 手动混控参数表（单位与 bfmini/src/main/flight/pid.h 严格一致）
//    - LiftCenter[3]:   mm [xo, yo, zo] (NED)
//    - RotationAxis[3]: 单位向量 * 1000
//    - t0[3]:           theta=0 时初始推力方向 * 1000
//    - T_eq:            平衡点推力 (mN)
//    - theta_eq:        平衡点倾角 (mrad)
//    - prop_rot_dir:    螺旋桨转向 (CCW = 1, CW = -1)
//    - tau_scaler[3]:   三轴力矩比例系数 [X, Y, Z]
// ============================================================================
struct __attribute__((packed)) ServoRotorMixerConfig {
  int16_t liftCenter[3];
  int16_t rotationAxis[3];
  int16_t t0[3];
  int16_t tEq;
  int16_t thetaEq;
  int8_t  propRotDir;
  uint16_t tauScaler[3];
};

static ServoRotorMixerConfig kModuleMixerConfigs[MAX_MODULES] = {
  // Module 1
  { {0,  320, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0,  1, {60, 40, 150} },
  // Module 2
  { {0, -320, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0, -1, {60, 40, 150} },
  // Module 3 ~ 8 (备用占位)
  { {0,    0, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0,  1, {60, 40, 150} },
  { {0,    0, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0, -1, {60, 40, 150} },
  { {0,    0, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0,  1, {60, 40, 150} },
  { {0,    0, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0, -1, {60, 40, 150} },
  { {0,    0, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0,  1, {60, 40, 150} },
  { {0,    0, -40}, {0, -1000, 0}, {0, 0, -1000}, 7000, 0, -1, {60, 40, 150} },
};

// ============================================================================
// 3. ESP-NOW 广播数据包定义 (18 bytes)
// ============================================================================
struct __attribute__((packed)) ThrustVecPacket {
  uint8_t  magic;      // 固定 0xA5
  uint8_t  module_id;  // 1 ~ MAX_MODULES
  uint16_t seq;        // 发送序号
  int32_t  vx_mN;      // 水平惯性系推力 X (mN)
  int32_t  vy_mN;      // 水平惯性系推力 Y (mN)
  int32_t  vz_mN;      // 水平惯性系推力 Z (mN)
  uint8_t  crc8;       // 前 16 字节 CRC8 校验
};

struct ModuleThrustState {
  bool     valid;
  uint32_t lastSeenMs;
  uint16_t lastSeq;
  int32_t  vx_mN;
  int32_t  vy_mN;
  int32_t  vz_mN;
};

static ModuleThrustState moduleStates[MAX_MODULES];
static const uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint16_t txSeq = 0;
static bool mixerAppliedToFc = false;
static bool espNowReady = false;

// 本地最近一次读取的飞控状态
static int32_t localVx_mN = 0, localVy_mN = 0, localVz_mN = 0;
static int32_t localThrust_mN = 0, localTheta_mrad = 0;
static float   localRollDeg = 0.0f, localPitchDeg = 0.0f;

// 融合后的合推力与等效整机姿态
static float totalVx_mN = 0.0f, totalVy_mN = 0.0f, totalVz_mN = 0.0f;
static bool  lpfInitialized = false;
static float effRollDeg = 0.0f, effPitchDeg = 0.0f;
static uint8_t activeModuleCount = 0;

// ============================================================================
// 4. 工具与 MSPv1 通信层
// ============================================================================
static uint8_t calcCrc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline int16_t readInt16LE(const uint8_t *buf, size_t offset) {
  return (int16_t)((uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8));
}

static inline int32_t readInt32LE(const uint8_t *buf, size_t offset) {
  return (int32_t)((uint32_t)buf[offset] |
                   ((uint32_t)buf[offset + 1] << 8) |
                   ((uint32_t)buf[offset + 2] << 16) |
                   ((uint32_t)buf[offset + 3] << 24));
}

static void sendMspV1(uint8_t cmd, const uint8_t *payload, uint8_t size) {
  uint8_t checksum = size ^ cmd;
  Serial1.write('$');
  Serial1.write('M');
  Serial1.write('<');
  Serial1.write(size);
  Serial1.write(cmd);
  for (uint8_t i = 0; i < size; i++) {
    Serial1.write(payload[i]);
    checksum ^= payload[i];
  }
  Serial1.write(checksum);
}

static bool readExactByte(uint8_t &b, uint32_t deadlineMs) {
  while ((int32_t)(millis() - deadlineMs) < 0) {
    if (Serial1.available() > 0) {
      b = (uint8_t)Serial1.read();
      return true;
    }
  }
  return false;
}

static bool waitForMspReply(uint8_t expectedCmd, uint8_t *payloadOut, uint8_t maxLen, uint8_t &sizeOut, uint32_t timeoutMs) {
  const uint32_t deadlineMs = millis() + timeoutMs;
  while ((int32_t)(millis() - deadlineMs) < 0) {
    uint8_t b = 0;
    if (!readExactByte(b, deadlineMs)) return false;
    if (b != '$') continue;

    uint8_t h1 = 0, h2 = 0, sz = 0, cmd = 0;
    if (!readExactByte(h1, deadlineMs) || !readExactByte(h2, deadlineMs) ||
        !readExactByte(sz, deadlineMs) || !readExactByte(cmd, deadlineMs)) {
      return false;
    }
    if (h1 != 'M') continue;

    uint8_t checksum = sz ^ cmd;
    uint8_t buf[64] = {0};
    if (sz > sizeof(buf)) {
      for (uint8_t i = 0; i < sz + 1; i++) {
        if (!readExactByte(b, deadlineMs)) return false;
      }
      continue;
    }
    for (uint8_t i = 0; i < sz; i++) {
      if (!readExactByte(buf[i], deadlineMs)) return false;
      checksum ^= buf[i];
    }
    uint8_t rxCsum = 0;
    if (!readExactByte(rxCsum, deadlineMs)) return false;
    if (rxCsum != checksum || cmd != expectedCmd) continue;
    if (h2 == '!') return false;
    if (h2 != '>') continue;

    const uint8_t copyLen = (sz < maxLen) ? sz : maxLen;
    memcpy(payloadOut, buf, copyLen);
    sizeOut = sz;
    return true;
  }
  return false;
}

static bool probeFcLink() {
  uint8_t payload[16] = {0};
  uint8_t sz = 0;
  sendMspV1(MSP_API_VERSION, nullptr, 0);
  if (waitForMspReply(MSP_API_VERSION, payload, sizeof(payload), sz, 500) && sz >= 3) {
    Serial.printf("[MSP] FC connected, API %u.%u\n", payload[1], payload[2]);
    return true;
  }
  return false;
}

static bool writeMixerConfigToFc(const ServoRotorMixerConfig &cfg) {
  static_assert(sizeof(ServoRotorMixerConfig) == 29, "ServoRotorMixerConfig must be 29 bytes");
  sendMspV1(MSP_SET_WP, (const uint8_t *)&cfg, sizeof(cfg));

  uint8_t ack[8] = {0};
  uint8_t sz = 0;
  if (waitForMspReply(MSP_SET_WP, ack, sizeof(ack), sz, 500) && sz >= 1 && ack[0] == 1) {
    Serial.println("[MSP] MSP_SET_WP mixer config applied OK");
    return true;
  }
  Serial.println("[MSP] MSP_SET_WP apply failed");
  return false;
}

static bool readLocalThrustVectorFromFc() {
  uint8_t payload[32] = {0};
  uint8_t sz = 0;
  sendMspV1(MSP_WP, nullptr, 0);
  if (!waitForMspReply(MSP_WP, payload, sizeof(payload), sz, 15) || sz < 20) {
    return false;
  }

  localVx_mN      = readInt32LE(payload, 0);
  localVy_mN      = readInt32LE(payload, 4);
  localVz_mN      = readInt32LE(payload, 8);
  localThrust_mN  = readInt32LE(payload, 12);
  localTheta_mrad = readInt32LE(payload, 16);
  return true;
}

static bool readLocalAttitudeFromFc() {
  uint8_t payload[16] = {0};
  uint8_t sz = 0;
  sendMspV1(MSP_ATTITUDE, nullptr, 0);
  if (!waitForMspReply(MSP_ATTITUDE, payload, sizeof(payload), sz, 15) || sz < 6) {
    return false;
  }
  localRollDeg  = readInt16LE(payload, 0) * 0.1f;
  localPitchDeg = readInt16LE(payload, 2) * 0.1f;
  return true;
}

// ============================================================================
// 5. ESP-NOW 模块间通信层
// ============================================================================
static void handleIncomingThrustPacket(const uint8_t *data, int len) {
  if (len != sizeof(ThrustVecPacket)) return;

  ThrustVecPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.magic != 0xA5) return;
  if (pkt.module_id < 1 || pkt.module_id > MAX_MODULES) return;
  if (pkt.module_id == MY_MODULE_ID) return; // 忽略自己的广播回环
  if (calcCrc8((const uint8_t *)&pkt, sizeof(pkt) - 1) != pkt.crc8) return;

  ModuleThrustState &st = moduleStates[pkt.module_id - 1];
  st.valid      = true;
  st.lastSeenMs = millis();
  st.lastSeq    = pkt.seq;
  st.vx_mN      = pkt.vx_mN;
  st.vy_mN      = pkt.vy_mN;
  st.vz_mN      = pkt.vz_mN;
}

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  handleIncomingThrustPacket(data, len);
}
#else
static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  handleIncomingThrustPacket(data, len);
}
#endif

static bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return false;
  }

  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, kBroadcastMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESPNOW] add broadcast peer failed");
    return false;
  }

  Serial.printf("[ESPNOW] ready on channel %d, MAC=%s\n", ESPNOW_CHANNEL, WiFi.macAddress().c_str());
  return true;
}

static void broadcastLocalThrustVector() {
  if (!espNowReady) return;

  ThrustVecPacket pkt = {};
  pkt.magic     = 0xA5;
  pkt.module_id = MY_MODULE_ID;
  pkt.seq       = ++txSeq;
  pkt.vx_mN     = localVx_mN;
  pkt.vy_mN     = localVy_mN;
  pkt.vz_mN     = localVz_mN;
  pkt.crc8      = calcCrc8((const uint8_t *)&pkt, sizeof(pkt) - 1);

  esp_now_send(kBroadcastMac, (const uint8_t *)&pkt, sizeof(pkt));
}

// ============================================================================
// 6. 合推力求和与等效 roll / pitch 解算
// ============================================================================
static void updateTotalThrustAndAttitude() {
  const uint32_t now = millis();
  int64_t sumVx = 0, sumVy = 0, sumVz = 0;
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_MODULES; i++) {
    ModuleThrustState &st = moduleStates[i];
    if (!st.valid) continue;
    if ((now - st.lastSeenMs) > MODULE_TIMEOUT_MS) {
      st.valid = false;
      continue;
    }
    sumVx += st.vx_mN;
    sumVy += st.vy_mN;
    sumVz += st.vz_mN;
    count++;
  }

  activeModuleCount = count;
  if (count == 0) return;

  if (!lpfInitialized) {
    totalVx_mN = (float)sumVx;
    totalVy_mN = (float)sumVy;
    totalVz_mN = (float)sumVz;
    lpfInitialized = true;
  } else {
    totalVx_mN += THRUST_LPF_ALPHA * ((float)sumVx - totalVx_mN);
    totalVy_mN += THRUST_LPF_ALPHA * ((float)sumVy - totalVy_mN);
    totalVz_mN += THRUST_LPF_ALPHA * ((float)sumVz - totalVz_mN);
  }

  // 合推力向上 (NED 下 Vz < 0)，整机等效 +z 轴为 -V / ||V||
  const float norm = sqrtf(totalVx_mN * totalVx_mN + totalVy_mN * totalVy_mN + totalVz_mN * totalVz_mN);
  if (norm < 100.0f) return; // 合推力过小（小于 0.1N）不解算姿态

  const float pitchRad = atan2f(-totalVx_mN, -totalVz_mN);
  const float rollRad  = atan2f(totalVy_mN, sqrtf(totalVx_mN * totalVx_mN + totalVz_mN * totalVz_mN));

  effPitchDeg = pitchRad * 180.0f / (float)M_PI;
  effRollDeg  = rollRad  * 180.0f / (float)M_PI;
}

// ============================================================================
// 7. 飞前串口调试命令（用于临时微调 T_eq / theta_eq 或重写飞控参数）
// ============================================================================
static void handleSerialCommands() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() == 0) continue;

      if (line.equalsIgnoreCase("WRITE")) {
        mixerAppliedToFc = writeMixerConfigToFc(kModuleMixerConfigs[MY_MODULE_ID - 1]);
      } else if (line.startsWith("SETEQ ")) {
        long tEq = 0, thetaEq = 0;
        if (sscanf(line.c_str(), "SETEQ %ld %ld", &tEq, &thetaEq) == 2) {
          kModuleMixerConfigs[MY_MODULE_ID - 1].tEq = (int16_t)tEq;
          kModuleMixerConfigs[MY_MODULE_ID - 1].thetaEq = (int16_t)thetaEq;
          mixerAppliedToFc = writeMixerConfigToFc(kModuleMixerConfigs[MY_MODULE_ID - 1]);
        }
      } else if (line.equalsIgnoreCase("STATUS")) {
        Serial.printf("[STAT] mod=%d/%d active=%u fc_applied=%d effR=%.2f effP=%.2f\n",
                      MY_MODULE_ID, EXPECTED_MODULE_COUNT, activeModuleCount,
                      mixerAppliedToFc ? 1 : 0, effRollDeg, effPitchDeg);
      }
      line = "";
    } else {
      line += c;
    }
  }
}

// ============================================================================
// 8. Arduino 主入口
// ============================================================================
void setup() {
  Serial.begin(DEBUG_BAUD);
  Serial1.begin(FC_BAUD, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  memset(moduleStates, 0, sizeof(moduleStates));

  delay(300);
  Serial.printf("\n[BOOT] modular_flight_node id=%d expected_N=%d\n", MY_MODULE_ID, EXPECTED_MODULE_COUNT);
  Serial.println("[CMD] WRITE | SETEQ <T_eq_mN> <theta_mrad> | STATUS");

  // 飞前阶段：探测飞控并写入手动混控参数
  if (probeFcLink()) {
    mixerAppliedToFc = writeMixerConfigToFc(kModuleMixerConfigs[MY_MODULE_ID - 1]);
  } else {
    Serial.println("[WARN] FC not responding on boot, will retry MSP_SET_WP in loop");
  }

  // 启动 ESP-NOW 模块间直连广播
  espNowReady = initEspNow();
}

void loop() {
  static uint32_t lastFusionMs = 0;
  static uint32_t lastLogMs = 0;
  static uint32_t lastRetryApplyMs = 0;
  const uint32_t now = millis();

  handleSerialCommands();

  // 若开机时飞控未就绪，每 1 秒重试一次手动参数下发
  if (!mixerAppliedToFc && (now - lastRetryApplyMs) >= 1000) {
    lastRetryApplyMs = now;
    if (probeFcLink()) {
      mixerAppliedToFc = writeMixerConfigToFc(kModuleMixerConfigs[MY_MODULE_ID - 1]);
    }
  }

  // 50Hz 主循环：读取本机 v_i -> ESP-NOW 广播 -> 融合合推力 V -> 解算等效 roll/pitch
  if ((now - lastFusionMs) >= FUSION_LOOP_PERIOD_MS) {
    lastFusionMs = now;

    if (readLocalThrustVectorFromFc()) {
      ModuleThrustState &self = moduleStates[MY_MODULE_ID - 1];
      self.valid      = true;
      self.lastSeenMs = now;
      self.vx_mN      = localVx_mN;
      self.vy_mN      = localVy_mN;
      self.vz_mN      = localVz_mN;

      broadcastLocalThrustVector();
    }

    readLocalAttitudeFromFc();
    updateTotalThrustAndAttitude();
  }

  // 10Hz 打印调试对比日志
  if ((now - lastLogMs) >= LOG_PRINT_PERIOD_MS) {
    lastLogMs = now;
    Serial.printf("N=%u/%u | vi=(%ld,%ld,%ld) T=%ld th=%ld | V=(%.0f,%.0f,%.0f) | locRP=(%.2f,%.2f) effRP=(%.2f,%.2f)\n",
                  activeModuleCount, EXPECTED_MODULE_COUNT,
                  (long)localVx_mN, (long)localVy_mN, (long)localVz_mN,
                  (long)localThrust_mN, (long)localTheta_mrad,
                  totalVx_mN, totalVy_mN, totalVz_mN,
                  localRollDeg, localPitchDeg, effRollDeg, effPitchDeg);
  }
}
