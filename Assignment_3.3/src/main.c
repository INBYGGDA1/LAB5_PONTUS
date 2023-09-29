/*
 * ================================================================
 * File: main.c
 * Author: Pontus Svensson, Carl Larsson
 * Email: psn19003@student.mdu.se, cln20001@student.mdu.se
 * Date: 2023-09-24
 * Description:
 *
 * License: This code is distributed under the MIT License. visit
 * https://opensource.org/licenses/MIT for more information.
 * ================================================================
 */

/*================================================================*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* TIWARE */
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#include "drivers/buttons.h"
#include "drivers/pinout.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "utils/uartstdio.h"

/* FreeRTOS */
#include "../inc/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portable.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif

/*================================================================*/
SemaphoreHandle_t s = NULL;
/*================================================================*/
void vWorker(uint32_t runTime) {
  TickType_t tickIN = xTaskGetTickCount();
  TickType_t xRunTime = pdMS_TO_TICKS(runTime);
  while (1) {
    TickType_t currentTickTime = xTaskGetTickCount();
    if (currentTickTime - tickIN >=
        xRunTime) { // Keep working until the specified amount of systemticks
                    // has occured

      break;
    }
  }
  // UARTprintf("WORKER TICK TIME: %d\n", xRunTime);
}
/*================================================================*/
void vTaskCHighPrio(void *pvParameters) {
  // Task C Periodic with 6 seconds
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(6000); // 6 second delay
  uint32_t xExecStart, xExecEnd, xExecTime, xExecSemPre, xExecSemPost,
      xExecSemTime;

  while (1) {

    vTaskDelayUntil(&xLastWakeTime, xDelay); // Delay for 6 s
    UARTprintf("Task C: STARTING\n");
    xExecStart =
        xTaskGetTickCount(); // Measure the time when Task C starts executing

    xExecSemPre =
        xTaskGetTickCount(); // Measure the time before the semaphore is taken

    if (xSemaphoreTake(s,
                       (TickType_t)portMAX_DELAY) ==
        pdTRUE) { // try to take the semaphore
      xExecSemPost =
          xTaskGetTickCount(); // Measure the time it took to take the semaphore

      xExecSemTime =
          floor(((xExecSemPost - xExecSemPre) * 1000.0) /
                configTICK_RATE_HZ); // Calculate the total time it took to take
                                     // the semaphore in MS

      UARTprintf("Task C: WORKING: SEMTAKE [%d] MS\n", xExecSemTime);
      vWorker(3000); // work for 3 seconds
      xExecEnd = xTaskGetTickCount();
      xExecTime =
          floor(((xExecEnd - xExecStart) * 1000.0) /
                configTICK_RATE_HZ); // Calculate the total execution time in MS

      if (xExecTime >
          6000) { // If the total execution time exceeds the periodicity of the
                  // task, it has missed the DEADLINE
        UARTprintf("Task C: COMPLETED: MISSED DEADLINE BY: [%d] MS: SEMGIVE\n",
                   xExecTime - 6000);
      } else { // Task COMPLETED before DEADLINE

        UARTprintf("Task C: COMPLETED [%d] MS: SEMGIVE\n", xExecTime);
      }
      while (xSemaphoreGive(s) != pdTRUE) { // Give the semaphore
      }

    } else {

      UARTprintf("Task C: Failed taking semaphore: Getting blocked!\n");
    }
  }
}
/*================================================================*/
void vTaskBMiddlePrio(void *pvParameters) {
  // Task B Periodic 13s
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(13000); // Delay for 13 seconds
  uint32_t xExecStart, xExecEnd, xExecTime;
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xDelay);
    UARTprintf("Task B: STARTED\n");
    xExecStart =
        xTaskGetTickCount(); // Get the starting time for the task execution
    UARTprintf("Task B: WORKING\n");
    vWorker(4000); // work for 4 seconds
    xExecEnd =
        xTaskGetTickCount(); // Measure the time it took to complete the work
    xExecTime = floor(((xExecEnd - xExecStart) * 1000.0) /
                      configTICK_RATE_HZ); // Calculate the execution time to ms

    if (xExecTime > 13000) { // If the task took longer than 13 seonds the
                             // DEADLINE was missed
      UARTprintf("Task B: COMPLETED: MISSED DEADLINE BY: [%d] MS",
                 xExecTime - 13000);
    } else { // Task completed before DEADLINE

      UARTprintf("Task B: COMPLETED [%d] MS\n", xExecTime);
    }
  }
}
/*================================================================*/
void vTaskALowPrio(void *pvParameters) {
  // Task A periodicity 11 Seconds
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(11000); // Delay for 11 seconds

  uint32_t xExecStart, xExecEnd, xExecTime, xExecSemPre, xExecSemPost,
      xExecSemTime;
  while (1) {

    vTaskDelayUntil(&xLastWakeTime, xDelay);
    UARTprintf("Task A: STARTING\n");
    xExecStart =
        xTaskGetTickCount(); // Get the time when task A started executing
    xExecSemPre = xTaskGetTickCount(); // Get the time for when the semaphore is
                                       // trying to get taken
    if ((xSemaphoreTake(s, (TickType_t)portMAX_DELAY)) == pdTRUE) {
      xExecSemPost =
          xTaskGetTickCount(); // get the time after the semaphore was taken
      xExecSemTime = floor(((xExecSemPost - xExecSemPre) * 1000.0) /
                           configTICK_RATE_HZ); // Calculate the time it took to
                                                // take the semaphore

      UARTprintf("Task A: SEMTAKE [%d] MS\n", xExecSemTime);
      UARTprintf("Task A: WORKING\n");
      vWorker(4000);                  // Do some work for 4 seconds
      xExecEnd = xTaskGetTickCount(); // Get the time after the task has
                                      // finished working
      xExecTime =
          floor(((xExecEnd - xExecStart) * 1000.0) /
                configTICK_RATE_HZ); // Calculate the time it took for the task
                                     // to finish executing in ms
      if (xExecTime > 11000) { // If the task took longer than 11 seconds we
                               // have missed the DEADLINE
        UARTprintf("Task A: COMPLETED: MISSED DEADLINE BY: [%d] MS: SEMGIVE\n",
                   xExecTime - 11000);
      } else { // Otherwise the task was completed before DEADLINE

        UARTprintf("Task A: COMPLETED [%d] MS: SEMGIVE\n", xExecTime);
      }
      while (xSemaphoreGive(s) != pdTRUE) { // Give back the semaphore
      }
    } else {
      UARTprintf("Task A: Unable to take sem\n");
    }
  }
}

/*================================================================*/
/*            Helper function configure uart                      */
/*================================================================*/
void ConfigureUART() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
}
int main(void) {
  uint32_t systemClock;

  systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   120000000);
  ConfigureUART();
  UARTprintf("\033[2J"); // Clear the screen

  BaseType_t xTaskAReturn =
      xTaskCreate(vTaskALowPrio, "Task A", 200, (void *)NULL, 1, NULL);
  BaseType_t xTaskBReturn =
      xTaskCreate(vTaskBMiddlePrio, "Task B", 200, (void *)NULL, 2, NULL);
  BaseType_t xTaskCReturn =
      xTaskCreate(vTaskCHighPrio, "Task C", 200, (void *)NULL, 3, NULL);
  if ((xTaskAReturn == pdPASS) && (xTaskBReturn == pdPASS) &&
      (xTaskCReturn == pdPASS)) {
    UARTprintf("Tasks created successfully\n");
  }
  s = xSemaphoreCreateBinary();
  if (s == NULL) {
    UARTprintf("Unable to create semaphore\n");
  }
  while (xSemaphoreGive(s) !=
         pdTRUE) { // The binary semaphore is initialized to an empty state, and
                   // needs to be given before it can be used
  }
  vTaskStartScheduler();
}
