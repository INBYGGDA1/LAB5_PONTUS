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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "drivers/pinout.h"
#include "driverlib/pin_map.h"
#include "drivers/buttons.h"
#include "../inc/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "inc/hw_memmap.h"
#include "task.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

void delay(uint32_t number_of_loops) {
  volatile uint32_t i;
  for (i = 0; i < number_of_loops; i++) {
    // Just looping for delay
  }
}

void ConfigureUART() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
}

void vLedBlinker() {
  unsigned char ucDelta, ucState;
  volatile uint32_t t = 50000;
  uint8_t led_tracker = 0;
  uint8_t led_array[] = {CLP_D1, CLP_D2, CLP_D3, CLP_D4};
  while (1) {
    ucState = ButtonsPoll(&ucDelta, 0);
    // Loop forward from D1 -> D4
    for (led_tracker = 0; led_tracker < 4; led_tracker++) {
      LEDWrite(led_array[led_tracker], led_array[led_tracker]);
      delay(t);
      LEDWrite(led_array[led_tracker], 0);
    }
    // A LEDs are off before going backwards
    // We start at LED D3, Then D2.
    // To stop same led light up twice we dont need to light D1,
    // It is enough to just restart from beginning with a delay at the end
    for (led_tracker = 2; led_tracker > 0; led_tracker--) {
      LEDWrite(led_array[led_tracker], led_array[led_tracker]);
      delay(t);
      LEDWrite(led_array[led_tracker], 0);
    }
    delay(t);
  }
}
void vButtonPress() {
  while (1) {
  }
}
int main(void) {

  ConfigureUART();

  PinoutSet(false, false);
  ButtonsInit();

  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, CLP_D1_PIN | CLP_D2_PIN);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, CLP_D3_PIN | CLP_D4_PIN);

  xTaskCreate(vLedBlinker, "TaskLEDBlink", 100, NULL, 1, NULL);
  xTaskCreate(vButtonPress, "TaskButtonPress", 100, NULL, 2, NULL);
}
