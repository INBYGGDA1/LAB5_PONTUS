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
#include "FreeRTOSConfig.h"
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
void vWorker(uint32_t runTime) { // Worker function
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

    vTaskDelayUntil(&xLastWakeTime, xDelay); // Delay for 6 seconds
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
      while (
          xSemaphoreGive(s) !=
          pdTRUE) { // Give back the semaphore allowing other tasks to take it
      }

      xExecEnd =
          xTaskGetTickCount(); // Measure the time it took to execute the task
      xExecTime = floor(
          ((xExecEnd - xExecStart) * 1000.0) /
          configTICK_RATE_HZ); // Calculate the total execution time of task C
      UARTprintf("Task C: COMPLETED [%d] MS: Giving semaphore\n", xExecTime);
    } else {

      UARTprintf("Task C: Failed taking semaphore: Getting blocked!\n");
    }
  }
}
/*================================================================*/
void vTaskBMiddlePrio(void *pvParameters) {
  // Periodic 13s
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(13000); // Delay for 13 seconds
  uint32_t xExecStart, xExecEnd, xExecTime;
  while (1) {
    vTaskDelayUntil(&xLastWakeTime,
                    xDelay); // Wait for 13 seconds before continuing
    UARTprintf("Task B: STARTED\n");
    xExecStart =
        xTaskGetTickCount(); // Measure the time when the task start executing
    UARTprintf("Task B: WORKING\n");
    vWorker(4000); // work for 4 seconds
    xExecEnd =
        xTaskGetTickCount(); // Measure the time it took to COMPLETE the work
    xExecTime = floor(
        ((xExecEnd - xExecStart) * 1000.0) /
        configTICK_RATE_HZ); // Calculate the total execution time for task B
    UARTprintf("Task B: COMPLETED [%d] MS\n", xExecTime);
  }
}
/*================================================================*/
void vTaskALowPrio(void *pvParameters) {
  // Period: 11 seconds,
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(11000); // Delay for 11 seconds
  uint32_t xExecStart, xExecEnd, xExecTime, xExecSemPre, xExecSemPost,
      xExecSemTime;
  while (1) {

    vTaskDelayUntil(&xLastWakeTime,
                    xDelay); // Yield for 11 seconds before continuing
    UARTprintf("Task A: STARTING\n");
    xExecStart =
        xTaskGetTickCount(); // Measure the time when the task started executing
    xExecSemPre =
        xTaskGetTickCount(); // Measure the time before the semaphore was taken
    if ((xSemaphoreTake(s, (TickType_t)portMAX_DELAY)) ==
        pdTRUE) { // Take the semaphore, preventing any other task to use this
                  // semaphore
      xExecSemPost =
          xTaskGetTickCount(); // Take the time after taking the semaphore
      xExecSemTime = floor(((xExecSemPost - xExecSemPre) * 1000.0) /
                           configTICK_RATE_HZ); // Calculate the run time in MS

      UARTprintf("Task A: SEMTAKE [%d] MS\n", xExecSemTime);
      UARTprintf("Task A: WORKING\n");
      vWorker(4000);                  // Do some work for 4 seconds
      xExecEnd = xTaskGetTickCount(); // Measure the time after the working was
                                      // COMPLETED
      xExecTime =
          floor(((xExecEnd - xExecStart) * 1000.0) /
                configTICK_RATE_HZ); // Calculate the execution time in ms
      UARTprintf("Task A: COMPLETED [%d] MS\n", xExecTime);
      UARTprintf("Task A: SEMGIVE\n");
      while (xSemaphoreGive(s) != pdTRUE) { // Return the semaphore giving other
                                            // tasks the option to take it
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
  UARTprintf("\033[2J");

  BaseType_t xTaskAReturn =
      xTaskCreate(vTaskALowPrio, "Task A", 200, (void *)NULL, 1,
                  NULL); // Create task A with priority 1
  BaseType_t xTaskBReturn =
      xTaskCreate(vTaskBMiddlePrio, "Task B", 200, (void *)NULL, 2,
                  NULL); // Task B with priority 2
  BaseType_t xTaskCReturn =
      xTaskCreate(vTaskCHighPrio, "Task C", 200, (void *)NULL, 3,
                  NULL); // Task C with priority 3
  if ((xTaskAReturn == pdPASS) && (xTaskBReturn == pdPASS) &&
      (xTaskCReturn == pdPASS)) {
    UARTprintf("Tasks created successfully\n");
  }
  s = xSemaphoreCreateBinary();
  if (s == NULL) {
    UARTprintf("Unable to create semaphore\n");
  }
  while (xSemaphoreGive(s) !=
         pdTRUE) { // The binary semaphore is initialized in an empty state and
                   // needs to be given first
  }
  vTaskStartScheduler();
}
