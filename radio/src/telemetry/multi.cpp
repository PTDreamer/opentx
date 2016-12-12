/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "opentx.h"
#include "multi.h"

MultiModuleStatus multiModuleStatus;

enum MultiPacketTypes {
  MultiStatus = 1,
  FrSkySportTelemtry,
  FrSkyHubTelemetry,
  SpektrumTelemetry,
  DSMBindPacket,
  FlyskyIBusTelemetry,
};

enum MultiBufferState {
  ReceivingMultiProtocol,
  FrskySportTelemetryFallback,
  FrskyHubTelemetryFallback,
  SpektrumTelemetryFallback,
  FlyskyTelemetryFallback
};

MultiBufferState guessProtocol()
{
  if (g_model.moduleData[EXTERNAL_MODULE].getMultiProtocol(false) == MM_RF_PROTO_DSM2)
    return SpektrumTelemetryFallback;
  else if (g_model.moduleData[EXTERNAL_MODULE].getMultiProtocol(false) == MM_RF_PROTO_FS_AFHDS2A)
    return FlyskyTelemetryFallback;
  else if ((g_model.moduleData[EXTERNAL_MODULE].getMultiProtocol(false) == MM_RF_PROTO_FRSKY) &&
           (g_model.moduleData[EXTERNAL_MODULE].subType == MM_RF_FRSKY_SUBTYPE_D16 || g_model.moduleData[EXTERNAL_MODULE].subType == MM_RF_FRSKY_SUBTYPE_D16_8CH))
    return FrskySportTelemetryFallback;
  else
    return FrskyHubTelemetryFallback;
}

static void processMultiStatusPacket(const uint8_t *data)
{
  multiModuleStatus.flags = data[0];
  multiModuleStatus.major = data[1];
  multiModuleStatus.minor = data[2];
  multiModuleStatus.patchlevel = data[3] << 8 | data[4];
}

static void processMultiTelemetryPaket(const uint8_t *packet)
{
  uint8_t len = packet[3];
  const uint8_t *data = packet + 4;

  // Switch type
  switch (packet[2]) {
    case MultiStatus:
      if (len >= 5)
        processMultiStatusPacket(data);
      break;
    case DSMBindPacket:
      if (len >= 10)
        processDSMBindPacket(data);
      break;
    case SpektrumTelemetry:
      // processSpektrumPacket expects data[0] to be the telemetry indicator 0xAA but does not check it,
      // just send one byte of our header instead
      if (len >= 17)
        processSpektrumPacket(data - 1);
      break;
    case FlyskyIBusTelemetry:
      if (len >= 28)
        processFlySkyPacket(data);
      break;
    case FrSkyHubTelemetry:
      if (len >= 4)
        frskyDProcessPacket(data);
      break;
    case FrSkySportTelemtry:
      if (len >= 4)
        sportProcessTelemetryPacket(data);
      break;
    default:
      // ignore unknown types
      break;
  }
}

void MultiModuleStatus::getStatusString(char *statusText)
{

  if (major == 0 && minor == 0 && patchlevel == 0) {
    strcpy(statusText, STR_MODULE_NO_TELEMETRY);
    return;
  }

  if (!protocolValid()) {
    strcpy(statusText, STR_PROTOCOL_INVALID);
    return;
  } else if (!serialMode()) {
    strcpy(statusText, STR_MODULE_NO_SERIAL_MODE);
    return;
  } else if (!inputDetected()) {
    strcpy(statusText, STR_MODULE_NO_INPUT);
    return;
  }
  sprintf(statusText, "V%d.%d.%d ", major, minor, patchlevel);
  if (isBinding())
    strcat(statusText, STR_MODULE_BINDING);
}

static uint8_t multiTelemetryBufferState;

void processMultiTelemetryData(const uint8_t data)
{
  if (telemetryRxBufferCount == 0) {
    if (data == 'M') {
      multiTelemetryBufferState = ReceivingMultiProtocol;
      return;
    } else if (data == 0x55 || data == 0x7e) {
      multiTelemetryBufferState = guessProtocol();
    } else {
      TRACE("[MP] invalid start byte 0x%02X", data);
      return;
    }
  }

  switch (multiTelemetryBufferState) {
    // FIX IS_FRSKY_SPORT_PROTOCOL
    case FlyskyTelemetryFallback:
      processFlySkyTelemetryData(data);
      return;
    case FrskyHubTelemetryFallback:
    case FrskySportTelemetryFallback:
      processFrskyTelemetryData(data);
      return;
    case SpektrumTelemetryFallback:
      processSpektrumTelemetryData(data);
      return;
  }

  if (telemetryRxBufferCount < TELEMETRY_RX_PACKET_SIZE) {
    telemetryRxBuffer[telemetryRxBufferCount++] = data;
  } else {
    TRACE("[MP] array size %d error", telemetryRxBufferCount);
    telemetryRxBufferCount = 0;
  }


  if (telemetryRxBufferCount == 1 && data != 'P') {
    TRACE("[MP] invalid second byte 0x%02X", data);
    telemetryRxBufferCount = 0;
    return;
  }

  uint8_t len = telemetryRxBuffer[3];
  // Length field does not count the header
  if (telemetryRxBufferCount >= 4 && len == telemetryRxBufferCount - 4) {
    // debug print the content of the packet
#if 0
    debugPrintf("[MP] Packet type %02X len 0x%02X: ",
                telemetryRxBuffer[2], telemetryRxBuffer[3]);
    for (int i=1; i<(telemetryRxBufferCount+3)/4; i++) {
      debugPrintf("[%02X%02X %02X%02X] ", telemetryRxBuffer[i*4], telemetryRxBuffer[i*4 + 1],
                  telemetryRxBuffer[i*4 + 2], telemetryRxBuffer[i*4 + 3]);
    }
    debugPrintf("\r\n");
#endif
    // Packet is complete, process it
    processMultiTelemetryPaket(telemetryRxBuffer);
    telemetryRxBufferCount = 0;
  }
}
