/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(app, CONFIG_CHIP_APP_LOG_LEVEL);

int main() {
   // const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
   // if (!device_is_ready(uart)) {
   //    return EXIT_FAILURE;
   // }

   // constexpr char message[] = "Hello World\r\n";
   // for (size_t i = 0; i < sizeof(message) - 1; ++i) {
   //    uart_poll_out(uart, message[i]);
   // }

   CHIP_ERROR err = AppTask::Instance().StartApp();

   LOG_ERR("Exited with code %" CHIP_ERROR_FORMAT, err.Format());
   return err == CHIP_NO_ERROR ? EXIT_SUCCESS : EXIT_FAILURE;
}
