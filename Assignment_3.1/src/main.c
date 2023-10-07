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
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "portable.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*================================================================*/
struct xParam {
  uint32_t xLed;
  uint32_t xButton;
  uint32_t xLedStatus;
};

/*================================================================*/
volatile uint8_t led_array[] = {CLP_D1, CLP_D2, CLP_D3, CLP_D4};
volatile uint8_t button_array[] = {LEFT_BUTTON, RIGHT_BUTTON};
TaskHandle_t xHandle_LED[4];
volatile uint32_t xButtonPress[2];

/*================================================================*/
const char *enum_type_names[] = {"D1", "D2", "D3", "D4"};
/*================================================================*/
/* The error routine that is called if the driver library         */
/* encounters an error.                                           */
/*================================================================*/
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif

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

/*================================================================*/
/*           Task function to blink the LEDs                      */
/*================================================================*/
void vLedBlinker(void *pvParameters) {
  struct xParam *param = (struct xParam *)
      pvParameters; // cast conversion of the parameters to the correct format
  TickType_t xDelay = pdMS_TO_TICKS(((param->xLed + 1) / 1) *
                                    1000); // Converting from MS to TICKS
  TickType_t xLastWakeTime =
      xTaskGetTickCount(); // Get the ticks since vTaskStartScheduler begun.
  while (1) {

    vTaskDelayUntil(&xLastWakeTime,
                    xDelay);                // Wait for specified amount of
    param->xLedStatus = !param->xLedStatus; // Toggle the LED status
    if ((param->xLed == 1 || param->xLed == 2) &&
        (xButtonPress[param->xLed - 1]) ==
            1) { // Skip if true xButtonPress[param->xLed - 1], the -1 is there
                 // since the array is 0 to 1 and it can only bet param->xLed =
                 // 1 || 2.
    } else {

      LEDWrite(
          led_array[param->xLed],
          !(param->xLedStatus) *
              (led_array[param->xLed])); // Toggle the led, it is either 1 or 0

      if (!(param->xLedStatus)) {
        UARTprintf("LED [%s] ON\n", enum_type_names[param->xLed]);
      } else {

        UARTprintf("LED [%s] OFF\n", enum_type_names[param->xLed]);
      }
    }
  }
  vPortFree(param);
}

static void buttonTimerCallback(TimerHandle_t xTimer) {
  struct xParam *xParam =
      pvTimerGetTimerID(xTimer); // Get the ID of the timer to know which timer
                                 // has expired. and steal the arguments.
  xButtonPress[xParam->xButton] = 0; // Reset the flag
}
/*================================================================*/
/*           Task to check for button presses                     */
/*================================================================*/
void vButtonPress(void *pvParameters) {
  unsigned char ucDelta, ucState;
  struct xParam *param = (struct xParam *)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay10Seconds = pdMS_TO_TICKS(10000);

  TimerHandle_t buttonTimer =
      xTimerCreate((const char *)"10STimer", xDelay10Seconds, pdFALSE,
                   (void *)param, buttonTimerCallback);
  while (1) {
    ucState = ButtonsPoll(&ucDelta, 0);

    if (BUTTON_PRESSED(button_array[param->xButton], ucState,
                       ucDelta)) { // Check for button presses
      UARTprintf("Button pressed: %d\n", button_array[param->xButton]);
      xButtonPress[param->xButton] = 1;
      LEDWrite(led_array[param->xLed], led_array[param->xLed]);
      xTimerStart(buttonTimer, 0);
    }
  }
  vPortFree(param);
}

void GPIO_init() {
  PinoutSet(false, false);
  ButtonsInit();
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Enable module for pins N & F
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) { // Make sure they are on before continuing
    ;
  }
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {
    ;
  }
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, CLP_D1_PIN | CLP_D2_PIN);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, CLP_D3_PIN | CLP_D4_PIN);
}
/*================================================================*/
/*                          Main                                  */
/*================================================================*/
int main(void) {
  uint32_t systemClock;
  uint32_t i, j;

  BaseType_t xButtonReturned; // types used to check whether the task was
                              // created succesfully
  BaseType_t xLedReturned;
  TaskHandle_t xHandle_BUTTON = NULL;

  struct xParam *param; // Struct to use as parameters to the tasks
  systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   120000000);

  ConfigureUART(); // Enable uart
  GPIO_init();     // Enable the pins for the LEDs and buttons

  for (i = 0; i < 4; i++) { // Create 4 tasks, each task controls 1 LED
    param = pvPortMalloc(
        sizeof(struct xParam)); // Allocate memory for each task parameters
    if (param == NULL) {
      break;
    }
    param->xLed =
        i; // xLed is used to toggle the correct LED defined in the led_array
    param->xButton = i; // not used in vLedBlinker
    param->xLedStatus = 0;
    xLedReturned = xTaskCreate(vLedBlinker, "Blink", 64, (void *)param, 3,
                               &xHandle_LED[i]);

    if (xLedReturned == pdPASS) {
      UARTprintf("vLedTASK: %d, created\n", i + 1);
    }
  }

  for (j = 0; j < 2; j++) { // Create 2 tasks which controls LEFT and Right
                            // button respectively
    param = pvPortMalloc(sizeof(struct xParam));
    if (param == NULL) {
      break;
    }
    param->xButton = j;
    param->xLed = j + 1; // will toggle the leds in the middle
    param->xLedStatus = 0;
    xButtonReturned = xTaskCreate(vButtonPress, "Button", 64, (void *)param, 3,
                                  &xHandle_BUTTON);
    if (xButtonReturned == pdPASS) {
      UARTprintf("vButtonTask: %d created\n", j + 1);
    }
  }
  vTaskStartScheduler();
}
