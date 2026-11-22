/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include "lib/core/CHIPError.h"
#include "lib/support/CodeUtils.h"

#include <setup_payload/OnboardingCodesUtil.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include <app-common/zap-generated/attributes/Accessors.h>

#include <cmath>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

namespace {
   // Set the measurement interval.
   constexpr size_t kMeasurementsIntervalMs = 300000; // 5 Minutes

   // Endpoint and attribute constants for Temperature Measurement
   constexpr uint8_t kTemperatureMeasurementEndpointId = 1;
   constexpr int16_t kTemperatureMeasurementAttributeMaxValue = 0x7fff;
   constexpr int16_t kTemperatureMeasurementAttributeMinValue = 0x954d;
   constexpr int16_t kTemperatureMeasurementAttributeInvalidValue = 0x8000;
   constexpr uint8_t kTemperatureDecimalPlaces = 2;
   static_assert(kTemperatureDecimalPlaces <= 2);

   // Endpoint and attribute constants for Relative Humidity Measurement
   constexpr uint8_t kHumidityMeasurementEndpointId = 2;
   constexpr uint16_t kHumidityMeasurementAttributeMaxValue = 10000; // 100.00%
   constexpr uint16_t kHumidityMeasurementAttributeMinValue = 0; // 0.00%
   constexpr uint16_t kHumidityMeasurementAttributeInvalidValue = 0xffff; // Invalid value
   constexpr uint8_t kHumidityDecimalPlaces = 2;
   static_assert(kHumidityDecimalPlaces <= 2);

   k_timer sMeasurementsTimer;
   bool sIsMeasurementTimerStarted = false;

   const device *sht4x_dev = DEVICE_DT_GET_ONE(sensirion_sht4x);


} //namespace

void AppTask::MatterEventHandler(const ChipDeviceEvent *event, intptr_t arg)
{
   // On initial commissioning, start the timer.
   // We use a flag to ensure this only happens once per boot.
   if (event->Type == DeviceEventType::kCommissioningComplete && !sIsMeasurementTimerStarted) {
      LOG_INF("Commissioning complete, starting measurements.");

      // Take an immediate reading by calling UpdateClustersState directly.
      // This is posted to the event queue to ensure it runs in the correct context.
      Nrf::PostTask([] { Instance().UpdateClustersState(); });

      // Start the periodic timer. K_NO_WAIT is NOT used here because we just
      // triggered the first reading manually. The timer will fire after the first interval.
      k_timer_start(&sMeasurementsTimer, K_MSEC(kMeasurementsIntervalMs), K_MSEC(kMeasurementsIntervalMs));
      sIsMeasurementTimerStarted = true;
   }
}

void AppTask::UpdateTemperatureClusterState()
{
   struct sensor_value sTemperature;
   Protocols::InteractionModel::Status status;
   int result = sensor_channel_get(sht4x_dev, SENSOR_CHAN_AMBIENT_TEMP, &sTemperature);

   if (result == 0) {
      const int64_t tempInMicroC =
         static_cast<int64_t>(sTemperature.val1) * 1000000 + sTemperature.val2;
      constexpr int64_t roundingQuantumMicroC =
         kTemperatureDecimalPlaces == 0 ? 1000000 :
         kTemperatureDecimalPlaces == 1 ? 100000 : 10000;
      const int64_t roundedTempInMicroC = tempInMicroC >= 0
         ? ((tempInMicroC + roundingQuantumMicroC / 2) / roundingQuantumMicroC) * roundingQuantumMicroC
         : ((tempInMicroC - roundingQuantumMicroC / 2) / roundingQuantumMicroC) * roundingQuantumMicroC;

      // Matter's MeasuredValue is expressed in 1/100ths of a degree Celsius.
      int32_t newValue = static_cast<int32_t>(roundedTempInMicroC / 10000);

      if (newValue > kTemperatureMeasurementAttributeMaxValue || newValue < kTemperatureMeasurementAttributeMinValue) {
         newValue = kTemperatureMeasurementAttributeInvalidValue;
      }
      const int16_t matterValue = static_cast<int16_t>(newValue);
      LOG_DBG("New temperature measurement: %d.%06d *C, rounded attribute value: %d", sTemperature.val1,
         sTemperature.val2, matterValue);

      status = Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(kTemperatureMeasurementEndpointId, matterValue);
      if (status != Protocols::InteractionModel::Status::Success) {
         LOG_ERR("Updating temperature measurement failed: %x", to_underlying(status));
      }
   } else {
      LOG_ERR("Getting temperature measurement data from SHT4x failed with: %d", result);
   }
}

void AppTask::UpdateHumidityClusterState()
{
   struct sensor_value sHumidity;
   Protocols::InteractionModel::Status status;
   int result = sensor_channel_get(sht4x_dev, SENSOR_CHAN_HUMIDITY, &sHumidity);

   if (result == 0) {
      const int64_t humidityInMicroPercent =
         static_cast<int64_t>(sHumidity.val1) * 1000000 + sHumidity.val2;

      constexpr int64_t kMicroPerHundredthPercent = 10000;
      constexpr int64_t kRoundingQuantumMicroPercent =
         kHumidityDecimalPlaces == 0 ? 1000000 :
         kHumidityDecimalPlaces == 1 ? 100000 : kMicroPerHundredthPercent;
      const int64_t roundedHumidityMicroPercent = humidityInMicroPercent >= 0
         ? ((humidityInMicroPercent + kRoundingQuantumMicroPercent / 2) / kRoundingQuantumMicroPercent) *
              kRoundingQuantumMicroPercent
         : ((humidityInMicroPercent - kRoundingQuantumMicroPercent / 2) / kRoundingQuantumMicroPercent) *
              kRoundingQuantumMicroPercent;
      const int64_t roundedHumidityHundredths = roundedHumidityMicroPercent / kMicroPerHundredthPercent;

      // The SHT4x datasheet clamps relative humidity to 0-100%.
      int64_t clampedHumidityHundredths = roundedHumidityHundredths;
      if (clampedHumidityHundredths < kHumidityMeasurementAttributeMinValue) {
         clampedHumidityHundredths = kHumidityMeasurementAttributeMinValue;
      } else if (clampedHumidityHundredths > kHumidityMeasurementAttributeMaxValue) {
         clampedHumidityHundredths = kHumidityMeasurementAttributeMaxValue;
      }
      const uint16_t matterValue = static_cast<uint16_t>(clampedHumidityHundredths);

      LOG_DBG("New humidity measurement: %d.%06d %%RH, Matter attribute value: %u", sHumidity.val1, sHumidity.val2,
         static_cast<unsigned int>(matterValue));

      status = Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(kHumidityMeasurementEndpointId, matterValue);
      if (status != Protocols::InteractionModel::Status::Success) {
         LOG_ERR("Updating humidity measurement failed: %x", to_underlying(status));
      }
   } else {
      LOG_ERR("Getting humidity measurement data from SHT4x failed with: %d", result);
   }
}


void AppTask::MeasurementsTimerHandler()
{
   Instance().UpdateClustersState();
}

void AppTask::UpdateClustersState()
{
   // Fetch a new sample from the sensor. This updates all channels.
   const int result_bme = sensor_sample_fetch(sht4x_dev);

   if (result_bme == 0) {
      // Update both clusters with the new data.
      UpdateTemperatureClusterState();
      UpdateHumidityClusterState();
   } else {
      LOG_ERR("Fetching data from SHT4X sensor failed with: %d", result_bme);
   }
}

CHIP_ERROR AppTask::Init()
{
   /* Initialize Matter stack */
   ReturnErrorOnFailure(Nrf::Matter::PrepareServer());

   if (!Nrf::GetBoard().Init()) {
      LOG_ERR("User interface initialization failed.");
      return CHIP_ERROR_INCORRECT_STATE;
   }

   /* Register Matter event handler that controls the connectivity status LED based on the captured Matter network
    * state. */
   ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));

   ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(MatterEventHandler, 0));

   if (!device_is_ready(sht4x_dev)) {
      LOG_ERR("SHT4X sensor device not ready");
      return chip::System::MapErrorZephyr(-ENODEV);
   }

   k_timer_init(&sMeasurementsTimer, [](k_timer *) { Nrf::PostTask([] { MeasurementsTimerHandler(); }); }, nullptr);

   ReturnErrorOnFailure(Nrf::Matter::StartServer());

   // On reboot, if the device is already commissioned, start the timer directly.
   if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0 && !sIsMeasurementTimerStarted) {
      LOG_INF("Device is already commissioned on boot, starting measurements.");
      // Use K_NO_WAIT to get an immediate reading on reboot.
      k_timer_start(&sMeasurementsTimer, K_NO_WAIT, K_MSEC(kMeasurementsIntervalMs));
      sIsMeasurementTimerStarted = true;
   }

   return CHIP_NO_ERROR;
}

CHIP_ERROR AppTask::StartApp()
{
   ReturnErrorOnFailure(Init());

   while (true) {
      Nrf::DispatchNextTask();
   }

   return CHIP_NO_ERROR;
}
