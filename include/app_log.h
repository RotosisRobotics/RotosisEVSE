#pragma once
#include <Arduino.h>

// 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG
#ifndef APP_LOG_LEVEL
#define APP_LOG_LEVEL 2
#endif

#define LOGE(fmt, ...) do { if (APP_LOG_LEVEL >= 0) Serial.printf("[E] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOGW(fmt, ...) do { if (APP_LOG_LEVEL >= 1) Serial.printf("[W] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOGI(fmt, ...) do { if (APP_LOG_LEVEL >= 2) Serial.printf("[I] " fmt "\n", ##__VA_ARGS__); } while(0)
#define LOGD(fmt, ...) do { if (APP_LOG_LEVEL >= 3) Serial.printf("[D] " fmt "\n", ##__VA_ARGS__); } while(0)
