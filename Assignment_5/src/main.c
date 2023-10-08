/*
 * ================================================================
 * File: main.c
 * Author: Pontus Svensson
 * Email: psn19003@student.mdu.se
 * Date: 2023-09-24
 * Description:
 *
 * License: This code is distributed under the MIT License. visit
 * https://opensource.org/licenses/MIT for more information.
 * ================================================================
 */

//=============================================================================
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "tm4c129_functions.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "utils/uartstdio.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "inc/hw_memmap.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "portable.h"

/*================================================================*/
void vWorker(uint32_t runTime) {
  TickType_t tickIN = xTaskGetTickCount();
  TickType_t xRunTime = pdMS_TO_TICKS(runTime);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  while (1) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TickType_t currentTickTime = xTaskGetTickCount();
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Keep working until the specified amount of systemticks
    // has occured
    if (currentTickTime - tickIN >= xRunTime) {
      break;
    }
  }
}

void vTaskContextSwitchTimer(void *pvParameters) {
  const uint32_t work_time_in_ms = 1000;
  TickType_t tickIN = xTaskGetTickCount();
  TickType_t xRunTime = pdMS_TO_TICKS(work_time_in_ms);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
  }
}
int main(void) {
  uint32_t systemClock;
  uint32_t i = 0;
  uint32_t j = 0;
  BaseType_t xTaskReturn[40];

  systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   120000000);
  ConfigureUART();
  UARTClearScreen();

  for (i = 0; i < 40; i++) {
    xTaskReturn[i] =
        xTaskCreate(vTaskContextSwitchTimer, "Worker", 64, NULL, 1, NULL);
    if (xTaskReturn[i] == pdPASS) {
      j++;
    }
  }
  UARTprintf("%d Tasks created\n", j);
  vTaskStartScheduler();
}
