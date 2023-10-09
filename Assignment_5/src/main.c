/*
 * ================================================================
 * File: main.c
 * Author: Pontus Svensson
 * Email: psn19003@student.mdu.se
 * Date: 2023-10-09
 * Description: This program measures the context switch time with an increasing
 * amount of tasks. The left column indicates number of tasks and the right
 * column shows the context switching time in TICKs.
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

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define QUEUE_SIZE 100
#define NUMBER_OF_TASKS 100

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct xContext {
  uint32_t current_tasks;
  uint32_t context_switching_time;
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
QueueHandle_t control_queue;

//=============================================================================
void vWorker(uint32_t runTime) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

//=============================================================================
// Task measuring the context switching time
//=============================================================================
void vTaskContextSwitchTimer(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct xContext contextInfo;
  contextInfo.current_tasks = *(uint32_t *)pvParameters;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  const uint32_t work_time_in_ms = 10;
  TickType_t tickIN = xTaskGetTickCount();
  TickType_t tickOUT;
  TickType_t tickExecutionTime;
  TickType_t xRunTime = pdMS_TO_TICKS(work_time_in_ms);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    tickIN = xTaskGetTickCount();
    // Force a context switch
    vTaskDelay(xRunTime);
    tickOUT = xTaskGetTickCount();
    // Dont take the delay into account, we are only interested in the context
    // switch time.
    contextInfo.context_switching_time = (tickOUT - tickIN) - xRunTime;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (xQueueSend(control_queue, &contextInfo, portMAX_DELAY) == pdPASS) {
      // If context switch completed, do some busy work for 100 seconds
      vWorker(100000);
      tickIN = 0;
      tickOUT = 0;
      contextInfo.context_switching_time = 0;
    }
  }
}

//=============================================================================
// Task the receives the contextInfo from the generic tasks and prints the
// current amount of tasks and the corresponding context switch time.
//=============================================================================
void vControlTask(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct xContext contextInfo;
  struct xContext oldContexInfo;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TaskHandle_t GenericTaskHandle[NUMBER_OF_TASKS];
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  BaseType_t xTaskReturn[NUMBER_OF_TASKS];
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  uint32_t i = 0;
  uint32_t j = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  const uint32_t work_time_in_ms = 500;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  const TickType_t xRunTime = pdMS_TO_TICKS(work_time_in_ms);
  TickType_t tickIN = xTaskGetTickCount();

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (i < NUMBER_OF_TASKS) {
      xTaskReturn[i] = xTaskCreate(vTaskContextSwitchTimer, "Generic task", 64,
                                   (void *)&i, 1, &GenericTaskHandle[i]);
      i++;
    }

    // Let the tasks do some work
    vTaskDelayUntil(&tickIN, xRunTime);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // We only print if new contextInfo from the generic task has been sent.
    if (xQueueReceive(control_queue, &contextInfo, portMAX_DELAY) == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // When the number of tasks is not increasing anymore, just print the
      // value of number of tasks created
      UARTprintf("%d, %d\n", i, contextInfo.context_switching_time);
    }
  }
}

//=============================================================================
int main(void) {
  uint32_t systemClock;

  systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   120000000);
  ConfigureUART();
  UARTClearScreen();
  control_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct xContext));
  if (xTaskCreate(vControlTask, "Control Task", 256, NULL, 2, NULL) == pdPASS) {
    UARTprintf("Control task succesfully created\n");
  }

  vTaskStartScheduler();
}
