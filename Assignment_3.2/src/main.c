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
    if (currentTickTime - tickIN >= xRunTime) {
      break;
    }
  }
  // UARTprintf("WORKER TICK TIME: %d\n", xRunTime);
}
/*================================================================*/
void vTaskCHighPrio(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 second delay
  uint32_t workingFlag = 0;
  uint32_t printFlag = 0;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xDelay); // Delay for 10 ticks
    if (workingFlag == 0) {

      if (printFlag != 1) {
        UARTprintf("Task C: Taking semaphore\n");
      }
      if (xSemaphoreTake(s,
                         (TickType_t)0) ==
          pdTRUE) { // try to take the semaphore
        UARTprintf("Task C: WORKING\n");
        vWorker(5000); // work for 5 seconds
        UARTprintf("Task C: COMPLETED: Giving semaphore\n");
        workingFlag = 1;

      } else if (printFlag != 1) {

        UARTprintf("Task C: Failed taking semaphore: Getting blocked!\n");
        printFlag = 1;
      }
    }
  }
}
/*================================================================*/
void vTaskBMiddlePrio(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(2000); // Delay for 2 seconds
  uint32_t workingFlag = 0;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xDelay);
    if (workingFlag == 0) {
      UARTprintf("Task B: WORKING\n");
      vWorker(5000); // work for 5 seconds
      UARTprintf("Task B: COMPLETED\n");
      workingFlag = 1;
    }
  }
}
/*================================================================*/
void vTaskALowPrio(void *pvParameters) {

  uint32_t workingFlag = 0;
  BaseType_t xReturn;
  while (1) {

    if (workingFlag == 0) {

      UARTprintf("Task A: taking semaphore\n");
      if ((xReturn = xSemaphoreTake(s, (TickType_t)0)) == pdTRUE) {

        UARTprintf("Task A: WORKING\n");
        vWorker(5000); // Do some work for 5 seconds

        UARTprintf("Task A: COMPLETED, Giving semaphore\n");
        xSemaphoreGive(s);
        workingFlag = 1;
      } else {
        UARTprintf("Task A: Unable to take sem\n");
      }
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
      xTaskCreate(vTaskALowPrio, "Task A", 64, (void *)NULL, 1, NULL);
  BaseType_t xTaskBReturn =
      xTaskCreate(vTaskBMiddlePrio, "Task B", 64, (void *)NULL, 2, NULL);
  BaseType_t xTaskCReturn =
      xTaskCreate(vTaskCHighPrio, "Task C", 64, (void *)NULL, 3, NULL);
  if ((xTaskAReturn == pdPASS) && (xTaskBReturn == pdPASS) &&
      (xTaskCReturn == pdPASS)) {
    UARTprintf("Tasks created successfully\n");
  }
  s = xSemaphoreCreateMutex();
  if (s == NULL) {
    UARTprintf("Unable to create semaphore\n");
  }
  vTaskStartScheduler();
}
