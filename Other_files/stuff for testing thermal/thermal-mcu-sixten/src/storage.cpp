#include "storage.h"
#include "config.h"
#include <Arduino.h>
#include <Preferences.h>

void loadConfiguration() {
  Preferences prefs;
  // Open in read-only mode (true)
  prefs.begin("thermal", true);

  Serial.println("Loading configurations from NVS...");

  for (int i = 0; i < 8; i++) {
    char key[16];
    snprintf(key, sizeof(key), "cfg_%d", i);

    if (prefs.isKey(key)) {
      size_t bytesRead =
          prefs.getBytes(key, &channelConfigs[i], sizeof(ChannelConfig));
      if (bytesRead != sizeof(ChannelConfig)) {
        Serial.printf("Warning: Size mismatch for Channel %d config. Resetting "
                      "to defaults.\n",
                      i);
        // Fallback to default if read corrupted size
        channelConfigs[i].isCooler = false;
        channelConfigs[i].pwmPeriodMs = 20000; // 20s
        channelConfigs[i].kp = 2.0f;
        channelConfigs[i].ki = 0.1f;
        channelConfigs[i].kd = 1.0f;
        channelConfigs[i].hystDelta = 50;        // 0.50 C
        channelConfigs[i].commTimeoutMs = 30000; // 30s
      } else {
        Serial.printf("Channel %d config loaded successfully.\n", i);
      }
    } else {
      Serial.printf("Channel %d config not found. Writing default values.\n",
                    i);
      // Default settings
      channelConfigs[i].isCooler = false;
      channelConfigs[i].pwmPeriodMs = 20000; // 20s
      channelConfigs[i].kp = 2.0f;
      channelConfigs[i].ki = 0.1f;
      channelConfigs[i].kd = 1.0f;
      channelConfigs[i].hystDelta = 50;        // 0.50 C
      channelConfigs[i].commTimeoutMs = 30000; // 30s
    }
  }
  prefs.end();
}

void saveConfiguration() {
  Preferences prefs;
  // Open in read-write mode (false)
  prefs.begin("thermal", false);

  Serial.println("Saving configurations to NVS...");

  for (int i = 0; i < 8; i++) {
    char key[16];
    snprintf(key, sizeof(key), "cfg_%d", i);

    size_t bytesWritten =
        prefs.putBytes(key, &channelConfigs[i], sizeof(ChannelConfig));
    if (bytesWritten != sizeof(ChannelConfig)) {
      Serial.printf("Error: Failed to write config for Channel %d.\n", i);
    } else {
      Serial.printf("Channel %d config saved successfully.\n", i);
    }
  }
  prefs.end();
}
