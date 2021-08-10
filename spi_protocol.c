/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

/* Optional includes for USB serial output */
#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif

/*local includes*/
#include "assert.h"

// Included for GPIO_O_LOCK and others
#include "inc/hw_gpio.h" //for macro declaration of GPIO_O_LOCK and GPIO_O_CR
#include <inc/hw_types.h>

#include "timers.h"
#include "timing_verify.h"

#define SPEED_TEST 0


#define LED_R (1<<1)
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)
#define CS (1<<3)
#define SCK (1<<4)
#define SI (1<<5)
#define SO (1<<0)
#define HIGH 1
#define LOW 0
#define INT_B (1<<0)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))
#define pdTICKSTOMS( xTicks ) ((xTicks * 1000 ) / configTICK_RATE_HZ )

#define CS_HIGH (GPIO_PORTA_DATA_R |= CS)
#define CS_LOW (GPIO_PORTA_DATA_R &= ~CS)
#define setCS(x) ((x)?CS_HIGH:CS_LOW)

#define SCK_HIGH (GPIO_PORTA_DATA_R |= SCK)
#define SCK_LOW (GPIO_PORTA_DATA_R &= ~SCK)
#define setSCK(x) ((x)?SCK_HIGH:SCK_LOW)

#define SI_HIGH (GPIO_PORTA_DATA_R |= SI)
#define SI_LOW (GPIO_PORTA_DATA_R &= ~SI)
#define setMOSI(x) ((x)?SI_HIGH:SI_LOW)

uint32_t SystemCoreClock;

#ifdef USB_SERIAL_OUTPUT

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************


static void
_configureUART(void)
{
    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

#endif

uint8_t debouncing = 0;
uint32_t button_press_count = 0;

void debounce()
{
  for (int i = 0; i < 255; i++)
  {
    __asm("    nop\n"
          "    nop\n"
          "    nop\n");
  }
  debouncing = 0;
}

uint8_t
getMISO()
{
  uint8_t value = 0;
  // for (int count=0; count < 8; count++)
  // {
  value = GPIO_PORTE_DATA_R & SO;
  // }
  return value;
}

static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    // This is a TiveDriver library function
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= 0x01;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    HWREG(GPIO_PORTA_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTA_BASE + GPIO_O_CR) = 0x3c;
    HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) = 0x3c;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    HWREG(GPIO_PORTE_BASE + GPIO_O_CR) = 0x03;
    HWREG(GPIO_PORTE_BASE + GPIO_O_DIR) = 0x03;
    HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) = 0x03;
    HWREG(GPIO_PORTE_BASE + GPIO_O_PUR) |= 0x03;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x01;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) = 0x01;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) = 0x01;
    HWREG(GPIO_PORTD_BASE + GPIO_O_PUR) |= 0x01;

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (CS|SCK|SI));
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, SO);

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, INT_B);

    GPIOPadConfigSet(GPIO_PORTD_BASE, INT_B, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //
    // Set the clocking to run at (SYSDIV_2_5) 80.0 MHz from the PLL.
    //                            (SYSDIV_3) 66.6 MHz
    //                            (SYSDIV_4) 50.0 MHz
    //                            (SYSDIV_5) 40.0 MHz
    //                            (SYSDIV_6) 33.3 MHz
    //                            (SYSDIV_8) 25.0 MHz
    //                            (SYSDIV_10) 20.0 MHz
    //
    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_10 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

uint8_t
transfer(uint8_t out)
{
  uint8_t count, in = 0;
  setSCK(LOW);
  for (count = 0; count < 8; count++)
  {
    in <<= 1;
    setMOSI(out & 0x80);
    setSCK(HIGH);
    in += getMISO();
    setSCK(LOW);
    out <<= 1;
  }
  setMOSI(0);

  return (in);
}

static uint8_t
expanderReadByte(uint8_t address)
{
  uint8_t value, preRead = 0x41;
  setSCK(LOW);
  setCS(LOW);
  transfer(preRead);
  transfer(address);
  value = transfer(0);
  return value;
}

void
expanderWriteByte(uint8_t address, uint8_t value)
{

  uint8_t preRead = 0x40;
  setSCK(LOW);
  setCS(LOW);
  transfer(preRead);
  transfer(address);
  transfer(value);
  return;
}

static void
_interruptHandlerPortD(void)
{
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t mask = GPIOIntStatus(GPIO_PORTD_BASE, 1);
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf ("mask: %x\n", (mask) & 0x1);
    #endif

    if (mask & (1<<0))
    {

      if (debouncing==0) {
        LED(LED_B, 1);
        #ifdef USB_SERIAL_OUTPUT
          UARTprintf ("External Button pressed\n");
          UARTprintf("Button pressed %i times\n", button_press_count);
        #endif
        debouncing = 1;
        button_press_count++;
        debounce();
      }
    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....
        // ....stay tuned
    }
    uint8_t value = expanderReadByte(0x19);
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf ("value: %x\n", value);
    #endif

    GPIOIntClear(GPIO_PORTD_BASE, mask);
    // vTaskDelay(10);
}

void
LED_REMOTE(uint8_t on) {
  CS_HIGH;
  expanderWriteByte(0x9, on);
}

static void
_heartbeat( void *notUsed )
{
   uint32_t green500ms = 500; // 1 second for on off
   uint32_t ledOn = 1;

    while(true)
    {
        vTaskDelay(green500ms / portTICK_RATE_MS);
        uint8_t out = 0x0;
        LED(LED_G, ledOn);
        LED_REMOTE(ledOn);
        ledOn = !ledOn;
        #ifdef USB_SERIAL_OUTPUT
          UARTprintf("on:%i\n",ledOn);
        #endif
        CS_HIGH;
        out = expanderReadByte(0x19);
        #ifdef USB_SERIAL_OUTPUT
          UARTprintf("0x19:%x\n",out & (1));
        #endif
    }
}

void
expanderInit(void)
{
    uint8_t out = 0;
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure IOCONA\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x0A, 0xA2);

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure IODIRA\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x00, 0xfe);

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure GPPUA\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x06, 0x01);

    out = 0;
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure IOCONB\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x15, 0xA2);

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure IODIRB\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x10, 0xff);

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure IPOLB\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x11, 0x01);

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure GPPUB\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x16, 0x01);

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure GPINTENB\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x12, 0x01);

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Configure INTCONB\n");
    #endif
    CS_HIGH;
    expanderWriteByte(0x14, 0x01);

    CS_HIGH;
    out = expanderReadByte(0x05);
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("IOCON:%x\n",out);
    #endif

    CS_HIGH;
    out = expanderReadByte(0x00);
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("IODIRA:%x\n",out);
    #endif
    CS_HIGH;
    LED_REMOTE(1);
    CS_HIGH;
    out = expanderReadByte(0x0A);
    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("OLATA:%x\n",out);
    #endif
}

void
_interrupt_polling (void * notUsed)
{
  GPIOIntRegister(GPIO_PORTD_BASE, _interruptHandlerPortD);
  GPIOIntTypeSet(GPIO_PORTD_BASE, INT_B, GPIO_RISING_EDGE);

  IntPrioritySet(INT_GPIOD, 255);  // Required with FreeRTOS 10.1.1,
  GPIOIntEnable(GPIO_PORTD_BASE, INT_B);
  while(1) {
    vTaskDelay(10);
    LED(LED_B, 0);
  }
}

int main( void )
{
    _setupHardware();
    SCK_LOW;
    CS_HIGH;
    #ifdef USB_SERIAL_OUTPUT
    	void spinDelayMs(uint32_t ms);
    	_configureUART();
    	spinDelayMs(1000);  // Allow UART to setup
    	UARTprintf("\n\n\nHello from SPI Protocal main()\n");
    #endif

    expanderInit();

    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );

    xTaskCreate(_interrupt_polling,
                "polling",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 5,  // higher numbers are higher priority..
                NULL );

    /* Start the tasks and timer running. */\
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
