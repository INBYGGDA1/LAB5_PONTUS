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
  uint32_t ledStatus;
  TickType_t xDelay =
      pdMS_TO_TICKS(((param->xLed + 1) / 1) * 1000); // Converting from MS to S
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {

    vTaskDelayUntil(&xLastWakeTime,
                    xDelay); // Wait for specified amount of
    param->xLedStatus = !param->xLedStatus;
    if ((param->xLed == 1 || param->xLed == 2) &&
        (xButtonPress[param->xLed - 1]) == 1) { // Skip
    } else {

      LEDWrite(led_array[param->xLed],
               !(param->xLedStatus) *
                   (led_array[param->xLed])); // Toggle the led

      // if (!(param->xLedStatus)) {
      //   UARTprintf("LED [%s] ON\n", enum_type_names[param->xLed]);
      // } else {
      //
      //   UARTprintf("LED [%s] OFF\n", enum_type_names[param->xLed]);
      // }
    }
  }
  vPortFree(param);
}

/*================================================================*/
/*           Task to check for button presses                     */
/*================================================================*/
void vButtonPress(void *pvParameters) {
  unsigned char ucDelta, ucState;
  struct xParam *param = (struct xParam *)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xDelay10Seconds = pdMS_TO_TICKS(10000);

  while (1) {
    ucState = ButtonsPoll(&ucDelta, 0);

    if ((button_array[param->xButton] & ucDelta) &&
        (button_array[param->xButton] & ucState)) { // Check for button presses
      UARTprintf("Button pressed: %d\n", button_array[param->xButton]);
      xButtonPress[param->xButton] = 1;
      // xLed is 1 or 2
      LEDWrite(led_array[param->xLed], led_array[param->xLed]);
      vTaskDelayUntil(&xLastWakeTime, xDelay10Seconds);
      xButtonPress[param->xButton] = 0;
    }
  }
  vPortFree(param);
}

void GPIO_init() {
  PinoutSet(false, false);
  ButtonsInit();
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {
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
    xLedReturned = xTaskCreate(vLedBlinker, "Blink", 100, (void *)param, 3,
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
    xButtonReturned = xTaskCreate(vButtonPress, "Button", 100, (void *)param, 3,
                                  &xHandle_BUTTON);
    if (xButtonReturned == pdPASS) {
      UARTprintf("vButtonTask: %d created\n", j + 1);
    }
  }
  vTaskStartScheduler();
}
