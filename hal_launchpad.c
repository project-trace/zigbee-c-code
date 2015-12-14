/**
* @ingroup hal
* @{ 
* @file hal_launchpad.c
*
* @brief Hardware Abstraction Layer (HAL) for the TI LaunchPad and Anaren Zigbee BoosterPack
*
* This file must be modified if changing hardware platforms. Based on hal_mdb2.c
*
* The Zigbee Module library & examples require several methods to be defined. 
* See hal_helper documentation. Also see hal_launchpad.h for macros that must be defined.
*
* @see hal_helper.c for utilities to assist when changing hardware platforms
*
Clocking:
SMCLK: 4MHz (see oscInit()), sources the following:
- UART - see halUartInit()
- SPI - see halSpiInitModule()
- PWM for RGB LEDs
ACLK: Sourced by VLO, ~12kHz
- Timer - see initTimer()

* $Rev: 1911 $
* $Author: dsmith $
* $Date: 2013-09-23 13:53:16 -0700 (Mon, 23 Sep 2013) $
*
* @section support Support
* Please refer to the wiki at www.anaren.com/air-wiki-zigbee for more information. Additional support
* is available via email at the following addresses:
* - Questions on how to use the product: AIR@anaren.com
* - Feature requests, comments, and improvements:  featurerequests@teslacontrols.com
* - Consulting engagements: sales@teslacontrols.com
*
* @section license License
* Copyright (c) 2012 Tesla Controls. All rights reserved. This Software may only be used with an 
* Anaren A2530E24AZ1, A2530E24CZ1, A2530R24AZ1, or A2530R24CZ1 module. Redistribution and use in 
* source and binary forms, with or without modification, are subject to the Software License 
* Agreement in the file "anaren_eula.txt"
* 
* YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” 
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO 
* EVENT SHALL ANAREN MICROWAVE OR TESLA CONTROLS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, 
* STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR 
* INDIRECT DAMAGES OR EXPENSE INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, 
* PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE 
* GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY 
* DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*/

#include "hal_launchpad.h"
#include "hal_version.h"
#include <stdint.h>

/** 
This is a function pointer for the Interrupt Service Routine called when a debug console character is received.
To use it, declare it with
<code> extern void (*debugConsoleIsr)(char);  </code> 
and then point it to a function you created, e.g.
<code> debugConsoleIsr = &handleDebugConsoleInterrupt;  </code>
and your function handleDebugConsoleInterrupt() will be called when a byte is received on the debug serial port.
*/
void (*debugConsoleIsr)(int8_t);

/** 
Function pointer for the ISR called when the button is pressed. 
Parameter is which button was pressed. */
void (*buttonIsr)(int8_t);

/** Function pointer for the ISR called when a timer generates an interrupt */
void (*timerIsr)(void);

/** Function pointer for the ISR called when a SRDY interrupt occurs */
void (*srdyIsr)(void);

/** Function pointer for the ISR called when a sysTick interrupt occurs */
void (*sysTickIsr)(void);


/** 
Flags to indicate when to wake up the processor. These are read in the various ISRs. 
If the flag is set then the processor will be woken up with HAL_WAKEUP() at the end of the ISR. 
This is required because HAL_WAKEUP() cannot be called anywhere except in an ISR. */
uint16_t wakeupFlags = 0;

/** 
The post-calibrated frequency of the Very Low Oscillator (VLO). 
This MUST be calibrated by calibrateVlo() prior to use.
Set with calibrateVlo() and read by initTimer(). */
uint16_t vloFrequency = 0;

/** Declaration so we can use this inside hal_launchpad.c for debugging */
int putchar(int c);

/** Debug console interrupt service routine, called when a byte is received on USCIB0. */
#pragma vector = USCIAB0RX_VECTOR 
__interrupt void USCIAB0RX_ISR(void)
{
#ifdef DEBUG_USCIAB0RX_ISR
    putchar('@');
    while (1)
    {
        toggleLed(1);
        delayMs(100);
    }
#endif
    if (IFG2 & UCA0RXIFG)               //debug console character received
    {
      //printf("%02X", UCA0RXBUF);
      debugConsoleIsr(UCA0RXBUF);    //reading this register clears the interrupt flag
    }
}


/** Port 1 interrupt service routine, called when an interrupt-enabled pin on port 1 changes state. */
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    if (P1IFG & BIT3)                   //Modify this based on which pin is connected to Button
    {
        buttonIsr(0);   // Button 0 was pressed
        if (wakeupFlags & WAKEUP_AFTER_BUTTON)    
            HAL_WAKEUP();
    }
    P1IFG = 0;                          // clear the interrupt
}


/** Port 2 interrupt service routine, called when an interrupt-enabled pin on port 2 changes state. */
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if (P2IFG & BIT2)                   //Modify this based on which pin is connected to SRDY
    {
        srdyIsr();
        if (wakeupFlags & WAKEUP_AFTER_SRDY)    
            HAL_WAKEUP();          
    }
    P2IFG = 0;                          // clear the interrupt
}

/**
Whether the selected button is pressed.
@param button which button - must be ANY_BUTTON or BUTTON_0 on this implementation.
@return >0 if the selected button is pressed, otherwise 0.
*/
uint8_t buttonIsPressed(uint8_t button)
{
    if ((button == ANY_BUTTON) || (button == BUTTON_0))
        return (~P1IN & BIT3);

    return 0;
}

/** Simple placeholder to point the function pointers to so that they don't cause mischief */
static void doNothing(int8_t a)
{
    volatile int8_t b = a;  //prevent this from getting optimized out
}

/** Initializes Oscillator: turns off WDT, configures MCLK to 8MHz using internal DCO and sets SMCLK to 4MHz */
void oscInit()
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    
    if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)                                     
    {  
        while(1); // Stop if calibration constants erased
    }   
    BCSCTL1 = CALBC1_8MHZ;          // Set DCO = 8MHz for MCLK
    DCOCTL = CALDCO_8MHZ;
    BCSCTL2 |= DIVS_1;              // SMCLK = DCO/2 (4MHz) 
    BCSCTL3 |= LFXT1S_2;            // Use VLO for ACLK
}

/** Initializes Ports/Pins: sets direction, interrupts, pullup/pulldown resistors etc. */
void portInit()
{
    //
    //    Initialize Ports
    //
    /*PORT 1:
    1.0     LED (Red on BoosterPack)
    1.1     UCA0RXD
    1.2     UCA0TXD
    1.3     Button
    1.4     Current Sensor / MRDY / External CS
    1.5     SCLK
    1.6     MISO / LED2 (Green on BoosterPack)
    1.7     MOSI
    */
    P1DIR = BIT0;                                       // Output
    P1IE  = BIT3;                                       // Enable Button Interrupt
    P1IES = BIT3;                                       // Interrupt on high-to-low transition    
    P1SEL = (BIT1 | BIT2 | BIT5 | BIT6 | BIT7);         // Enable USCI - NOTE: default value is NOT 0!
    P1SEL2 =(BIT1 | BIT2 | BIT5 | BIT6 | BIT7);         
    P1IFG = 0;                                          // Clear P1 interrupts
    //Note: Reading analog input overrides USCI control of pins P1.4
    
    /*PORT 2
    2.0     Zigbee Module CS 
    2.1     RGB_LED_RED (TA1.1)
    2.2     Module SRDY
    2.3     Bit-Bang I2C SDA
    2.4     Bit-Bang I2C SCL / LED3
    2.5     RGB_LED_GREEN (TA1.2)
    2.6     RGB_LED_BLUE (TA0.1)
    2.7     Module Reset
    */
    P2DIR = BIT0 | BIT1 | BIT5 | BIT6 | BIT7;           // Configure outputs
    P2SEL = 0;
    P2SEL2 = 0;
    P2IFG = 0;
    P2OUT &= ~(BIT0);                                   //turn off module
    // Note: Bit-Bang I2C signals will be configured by the Bit-Bang I2C Driver
    
    /*PORT 3 is not connected in this package, put pull-downs should be enabled as per datasheet */
    P3REN = 0xFF;            //Enable pull-downs on all pins
}


/** 
Initialize UART debug console for I/O. Configures the USCI in the processor to use UART mode. 
@post UART may be used by putchar() etc. 
@note LaunchPad debugger can only handle 9600 baud, no faster
*/
void halUartInit()
{
    UCA0CTL1 = UCSWRST;                         // Stop USCIA0 state machine
    UCA0CTL0 = 0;
    UCA0CTL1 |= UCSSEL_2;                       // USCIA0 source from SMCLK
    UCA0BR0 = 26; UCA0BR1 = 0;                  // 4mHz smclk w/modulation for 9,600bps, table 15-5 
    UCA0MCTL = UCBRS_0 +UCBRF_1 + UCOS16;       // Modulation UCBRSx=1, over sampling      
    UCA0CTL1 &= ~UCSWRST;                       // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                            // Enable USCI_A0 RX interrupt
}

/** Display information about this driver firmware */
void displayVersion()
{
    int i = 0;
    printf("\r\n\r\n");
    for (i=0; i<8; i++)
        printf("-");
    printf(" Module Interface and Examples %s ", MODULE_INTERFACE_STRING);
    for (i=0; i<8; i++)
            printf("-");
    printf("\r\n");
    printf("%s", MODULE_VERSION_STRING);
}

/** 
Configures hardware for the particular hardware platform:
- Ports: sets direction, interrupts, pullup/pulldown resistors etc. 
- Holds radio in reset (active-low)

@post the board is configured for proper operation.
*/
void halInit()
{
    oscInit();                      // Initialize oscillators
    portInit();                     // Configure GPIO ports
    halUartInit();                  // Initialize UART
    SPI_SS_CLEAR();                 // Deselect Module
    TACTL = MC_0;                   //  Stop Timer A:

    //Point the function pointers to doNothing() so that they don't trigger a restart
    buttonIsr = &doNothing;
    //Note: during development you may want to comment out the line below. 
    //That way you can restart the micro by pressing any key in the terminal window.
    debugConsoleIsr = &doNothing;


#ifdef TESTING_SPI_UART_VERSION
    halSpiInitModule();
#endif
    clearLeds();
    displayVersion();    
}

/** Send one byte via hardware UART. Required for printf() etc. in stdio.h */
int putchar(int c)
{	
    while (!(IFG2 & UCA0TXIFG));   // Wait for ready
    UCA0TXBUF = (uint8_t) (c & 0xFF); 
    return c;
}

/**
Initializes the Serial Peripheral Interface (SPI) interface to the Zigbee Module (ZM).
@note Maximum module SPI clock speed is 4MHz. SPI port configured for clock polarity of 0, clock phase of 0, and MSB first.
@note On the MDB2 the MSP430 uses USCIB0 SPI port to communicate with the module.
@pre SPI pins configured correctly: 
- Clock, MOSI, MISO configured as SPI function
- Chip Select configured as an output
- SRDY configured as an input.
@post SPI port is configured for communications with the module.
@note this function is not required if using the UART to communicate with the module
*/
void halSpiInitModule()
{
    UCB0CTL1 |= UCSSEL_2 | UCSWRST;                 //serial clock source = SMCLK, hold SPI interface in reset
    UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC;     //clock polarity = inactive is LOW (CPOL=0); Clock Phase = 0; MSB first; Master Mode; Synchronous Mode    
    UCB0BR0 = 2;  UCB0BR1 = 0;                      //SPI running at 2MHz (SMCLK / 2)
    UCB0CTL1 &= ~UCSWRST;                           //start USCI_B1 state machine  
}

/**
Sends a message over SPI to the Module.
The Module uses a "write-to-read" approach: to read data out, you must write data in.
This is a private function that gets wrapped by other methods, e.g. spiSreq(), spiAreq, etc.
To Write, set *bytes and numBytes. To Read, set *bytes only. Don't need to set numBytes because the 
Module will stop when no more bytes are received.
@param bytes the data to be sent or received.
@param numBytes the number of bytes to be sent. This same buffer will be overwritten with the received data.
@pre SPI port configured for the Module and Module has been initialized properly
@post bytes contains received data, if any
@see zm_phy_spi.c
@note this function is not required if using the UART to communicate with the module
*/
void spiWrite(uint8_t *bytes, uint8_t numBytes)
{
    while (numBytes--)
    {  
        UCB0TXBUF = *bytes;
        while (!(IFG2 & UCB0RXIFG)) ;     //WAIT for a character to be received, if any
        *bytes++ = UCB0RXBUF;             //read bytes
    }
}

/** 
A fairly accurate blocking delay for waits in the millisecond range. Good for 1mSec to 1000mSec. 
@note At 1MHz, error of zero for 100mSec or 1000mSec. For 10mSec, error of 100uSec. At 1mSec, error is 20uSec.
@note At 8MHz, error of zero for 1000mSec.
@note Accuracy will depend on the clock source. MSP430F2xx internal DCO is typically +/-1%. 
For better timing accuracy, use a timer, or a crystal.
@pre TICKS_PER_MS is defined correctly for the clock setting used.
@param delay number of milliseconds to delay
*/
void delayMs(uint16_t delay)
{
    while (delay--)
    {
        __delay_cycles(TICKS_PER_MS);
    }
}

/** 
Turns ON the specified LED. 
@param led the LED to turn on
@post The specified LED is turned on. 
@return 0 if success, -1 if invalid LED specified
*/
int16_t setLed(uint8_t led)
{
    switch (led)
    {
    case 0:
        P1OUT |= BIT0;
        return 0;
    case 1:
        P2OUT &= ~BIT1;      // RGB Red
        return 0;
    case 2:
        P2OUT &= ~BIT5;     // RGB Green
        return 0;
    case 3:
        P2OUT &= ~BIT6;      // RGB Blue
        return 0;
    case 4:
        P2OUT &= ~BIT4;
        return 0;
    default:
        return -1;
    }
}

/**
Turns OFF the specified LED.
@param led the LED to turn off
@post The specified LED is turned off.
@return 0 if success, -1 if invalid LED specified
*/
int16_t clearLed(uint8_t led)
{
    switch (led)
    {
    case 0:
        P1OUT &= ~BIT0;
        return 0;
    case 1:
        P2OUT |= BIT1;      // RGB Red
        return 0;
    case 2:
        P2OUT |= BIT5;     // RGB Green
        return 0;
    case 3:
        P2OUT |= BIT6;      // RGB Blue
        return 0;
    case 4:
        P1OUT &= ~BIT6;
        return 0;
    default:
        return -1;
    }
}

/** 
Turns OFF LEDs. 
@post LEDs are turned off. 
*/
void clearLeds()
{ 
    P1OUT &= ~BIT0;
    P1OUT &= ~BIT6;
    P2OUT |= (BIT1 | BIT5 | BIT6);
}

/** 
Toggles the specified LED. 
@param led the LED to toggle
@post The specified LED is toggled. 
@return 0 if success, -1 if invalid LED specified
*/
int16_t toggleLed(uint8_t led)
{
    switch (led)
    {
    case 0:
        P1OUT ^= BIT0;
        return 0;
    case 1:
        P2OUT ^= BIT1;      // RGB Red
        return 0;
    case 2:
        P2OUT ^= BIT5;     // RGB Green        
        return 0;
    case 3:
        P2OUT ^= BIT6;      // RGB Blue
        return 0;
    case 4:
        P2OUT ^= BIT4;
        return 0;
    default:
        return -1;
    }
}

//
//
//          LAUNCHPAD PERIPHERALS
//          NOT REQUIRED FOR MODULE OPERATION
//
//

/** 
Reads the MSP430 supply voltage using the Analog to Digital Converter (ADC) and internal voltage reference. 
ADC clocked by ADC10OSC
@return Vcc supply voltage, in millivolts
*/
uint16_t getVcc3()
{
    ADC10CTL0 = SREF_1 + REFON + REF2_5V + ADC10ON + ADC10SHT_3;  // use internal ref, turn on 2.5V ref, set samp time = 64 cycles
    ADC10CTL1 = INCH_11;                         
    delayMs(1);                                     // Allow internal reference to stabilize
    ADC10CTL0 |= ENC + ADC10SC;                     // Enable conversions
    while (!(ADC10CTL0 & ADC10IFG));                // Conversion done?   
    // Note: result is now ready in ADC10MEM
#define REF2_5V_VOLTAGE_MV    2500l
#define VCC_DIVIDED_BY_2        2
#define ADC_TO_MV_MULTIPLIER    (REF2_5V_VOLTAGE_MV * VCC_DIVIDED_BY_2)
    unsigned long temp = (ADC10MEM * ADC_TO_MV_MULTIPLIER);        // Convert raw ADC value to millivolts
    return ((uint16_t) (temp / 1024l));
}

/** 
Reads the current sense amplifier output using the Analog to Digital Converter (ADC). 
Current sense resistor is 0.2 ohms, amplifier multiplies voltage drop by 50. Therefore, every mA of
current translates into 10mV measured by MSP430.
ADC clocked by ADC10OSC
@note Current sense input is on P1.4/A4 on MSP430G2553
@pre Current sense shunt is installed on the BoosterPack
@return Current in mA multiplied by 10 (e.g. return value of 713 = 71.3mA)
*/
uint16_t getCurrentSensor()
{
    ADC10CTL0 = SREF_1 + REFON + ADC10ON + ADC10SHT_3;  // use internal ref, set samp time = 64 cycles
    ADC10CTL1 = INCH_4;                         
    delayMs(1);                                     // Allow internal reference to stabilize
    ADC10CTL0 |= ENC + ADC10SC;                     // Enable conversions
    while (!(ADC10CTL0 & ADC10IFG));                // Conversion done?   
    // Note: result is now ready in ADC10MEM
    uint32_t temp = (ADC10MEM * 1500l);        // Convert raw ADC value to milliamperes
    return ((uint16_t) (temp / 1024l));    
}

/** 
Configures all module interface signals as inputs to allow the module to be programmed.
Toggles LED0 quickly to indicate application is running. 
*/
void halSetAllPinsToInputs(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)                                     
    {  
        while(1); // Stop if calibration constants erased
    }   
    BCSCTL1 = CALBC1_8MHZ; // Set DCO = 8MHz for MCLK
    DCOCTL = CALDCO_8MHZ;
    BCSCTL2 |= DIVS_3;     //SMCLK = DCO/2 (4MHz)    
    
    P1DIR = 1; P1REN = 0; P1IE = 0; P1SEL = 0; 
    P2DIR = 0; P2REN = 0; P2IE = 0; P2SEL = 0; 
    P3REN = 0xFF;
    for (;;)
    {
        toggleLed(4);
        delayMs(100);   
    }
}

/** Configures which events will wake up the processor if sleeping. */
void halSetWakeupFlags(uint16_t wakeupFlagsToSet)
{
    wakeupFlags |= wakeupFlagsToSet;  
}

/** No flags will wakeup the processor while it is sleeping. */
void halClearWakeupFlags(uint16_t wakeupFlagsToClear)
{
    wakeupFlags &= ~wakeupFlagsToClear;  
}

/** The longest delay allowed by the timer, in seconds. */
#define TIMER_MAX_SECONDS 4

/** 
Configures timer.
@pre ACLK sourced from VLO
@pre VLO has been calibrated; number of VLO counts in one second is in vloFrequency.
@param seconds period of the timer. Maximum is 0xFFFF / vloFrequency; or about 4 since VLO varies between 9kHz - 15kHz. 
@note Cannot use initTimer() and RGB PWM functions at the same time since they both use the same timer module.
@note Use a prescaler on timer (e.g. set IDx bits in TACTL register) for longer times. 
@note Maximum prescaling of Timer A is divide by 8. Even longer times can be obtained by prescaling ACLK 
if this doesn't affect other system peripherals.
@return 0 if success; -1 if illegal parameter or -2 if VLO not calibrated
*/
int16_t initTimer(uint8_t seconds)
{  
    if ((seconds > TIMER_MAX_SECONDS) || (seconds == 0))
        return -1;
    if (vloFrequency == 0)
        return -2;

    wakeupFlags |= WAKEUP_AFTER_TIMER;
    TACCTL0 = CCIE;
    TACCR0 = vloFrequency * (seconds);
    TACTL = TASSEL_1 + MC_1;
    return 0;
}

/** Halts the timer. Leaves period unchanged. */
void stopTimer()
{
    TACTL = MC_0; 
}

//
//  System Tick (sysTick)
//

/** 
Initializes the systick interval timer. Default Interval is approximately 8mSec and is not
adjustable because we are using the watchdog timer which has fixed timeout periods.
@note we are using the Watchdog Timer module to be a timer, not a classic watchdog. This is because 
we need the systick function at the same time as the PWM. The PWM uses all the normal timer modules
on the microcontroller so we have to use the WDT module instead. If you aren't using PWM then we
recommend just using initTimer() instead.
@note WDT sourced from SMCLK
*/
void initSysTick(void)
{
  WDTCTL = WDT_MDLY_32;                     // Set Watchdog Timer interval, source from SMCLK (4MHz), 8Msec
  IE1 |= WDTIE;                             // Enable WDT interrupt
}

// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
  sysTickIsr();
}


/** Interrupt Service Routine called by Timer A0. May wakeup processor based on value of wakeupFlags. */
#pragma vector=  TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
{
    timerIsr();
    if (wakeupFlags & WAKEUP_AFTER_TIMER)    
    {
        HAL_WAKEUP();     
    }
}

/** 
Calibrate VLO. Once this is done, the VLO can be used semi-accurately for timers etc. Once 
calibrated, VLO is within ~2% of actual when using a 1% calibrated DCO frequency and temperature and 
supply voltage remain unchanged.
@return VLO frequency (number of VLO counts in 1sec), or -1 if out of range
@pre SMCLK is 4MHz
@pre MCLK is 8MHz
@pre ACLK sourced by VLO (BCSCTL3 = LFXT1S_2 in MSP430F2xxx)
@note Calibration is only as good as MCLK source. Obviously, if using the internal DCO (+/- 1%) then 
this value will only be as good as +/- 1%. YMMV.
@note On MSP430F248 or MSP430F22x2 or MSP430F22x4, must use TACCR2. On MSP430F20x2, must use TACCR0. 
On MSP430G2553, must use CCI0B on CCR0
Check device-specific datasheet to see which module block has ACLK as a compare input.
Modify TACCTLx, TACCRx, and CCIS_x accordingly
For example, see page 23 of the MSP430F24x datasheet or page 17 of the MSP430F20x2 datasheet, or 
page 18 of the MSP430F22x4 datasheet.
@note If application will require accuracy over change in temperature or supply voltage, recommend 
calibrating VLO more often.
@post Timer A settings restored to what they were beforehand except for TACCR0 which is reset.
*/
int16_t calibrateVlo()
{
    WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
    delayMs(1000);                        // Wait for oscillators to settle
    uint16_t temp_BCSCTL1 = BCSCTL1;
    uint16_t temp_TACCTL0 = TACCTL0;
    uint16_t temp_TACTL = TACTL;
    
    BCSCTL1 |= DIVA_3;                    // Divide ACLK by 8
    TACCTL0 = CM_1 + CCIS_1 + CAP;        // Capture on ACLK
    TACTL = TASSEL_2 + MC_2 + TACLR;      // Start TA, SMCLK(DCO), Continuous
    while ((TACCTL0 & CCIFG) == 0);       // Wait until capture
    
    TACCR0 = 0;                           // Ignore first capture
    TACCTL0 &= ~CCIFG;                    // Clear CCIFG
    
    while ((TACCTL0 & CCIFG) == 0);       // Wait for next capture
    unsigned int firstCapture = TACCR0;   // Save first capture
    TACCTL0 &= ~CCIFG;                    // Clear CCIFG
    
    while ((TACCTL0 & CCIFG) ==0);        // Wait for next capture
    
    unsigned long counts = (TACCR0 - firstCapture);        // # of VLO clocks in 8Mhz
    BCSCTL1 = temp_BCSCTL1;                  // Restore original settings
    TACCTL0 = temp_TACCTL0;
    TACTL = temp_TACTL;
    
    //TACCTL0 = 0; TACTL = 0;                 // Clear Timer settings
    
    vloFrequency = ((uint16_t) (32000000l / counts));
    if ((vloFrequency > VLO_MIN) && (vloFrequency < VLO_MAX))
        return vloFrequency;
    else
        return -1;
}

/** Required for compatibility with processors (like Stellaris) that have a UART FIFO */
uint8_t halUartBusy()
{
	return (0);
}

//
//RGB LEDs:
//
#define RED_PWM                 TA1CCR1
#define BLUE_PWM                TA0CCR1
#define GREEN_PWM               TA1CCR2  

/**
Initializes the PWM engine used for the RGB LED. This allows the RGB LED to display many colors.
@post RGB LED may be used, with halRgbSetLeds().
@note Uses SMCLK and Timer modules TA1 and TA0.
*/
void halRgbLedPwmInit()
{
    P2SEL |= BIT1 | BIT5 | BIT6;    // Select Timer functionality
    P2DIR |= BIT1 | BIT5 | BIT6;    // Output
    P2SEL2 = 0;
    
    TA1CCR0 = RGB_LED_PWM_PERIOD - 1;                             // PWM Period
    TA1CCTL1 = OUTMOD_7;                         // CCR1 reset/set
    TA1CCTL2 = OUTMOD_7;                         // CCR1 reset/set    
    TA1CTL = TASSEL_2 + MC_1;                  // SMCLK, up mode
    
    TA0CCR0 = RGB_LED_PWM_PERIOD - 1;                             // PWM Period
    TA0CCTL1 = OUTMOD_7;                         // CCR1 reset/set
    TA0CTL = TASSEL_2 + MC_1;                  // SMCLK, up mode
    
    halRgbSetLeds(0,0,0);                   // Initialization done, turn them off
}

/* Multiply given values by this to get true white. Measured empirically. */
#define COLOR_BALANCE_RED (0.27f)
#define COLOR_BALANCE_BLUE (0.75f)
#define COLOR_BALANCE_GREEN (1.0f)

/** 
Sets RGB LED color to the selected values. Adjusts intensities so that illuminance of each color is 
even. This way calling the function with (0xFF, 0xFF, 0xFF) will result in a fairly good white.
@param red the amount of red - 0 to 0xFF
@param blue the amount of blue - 0 to 0xFF
@param green the amount of green - 0 to 0xFF
@pre halRgbLedPwmInit() has been called to initialize the PWM engine.
@post RGB LED displays the selected colors.
*/
void halRgbSetLeds(uint8_t red, uint8_t blue, uint8_t green)
{
    /* Adjust intensity of each color for white balance */
    float colorBalancedRed = COLOR_BALANCE_RED * ((float) red);
    float colorBalancedGreen = COLOR_BALANCE_GREEN * ((float) green);
    float colorBalancedBlue = COLOR_BALANCE_BLUE * ((float) blue);

    /* Now, need to set the PWM cycle. 
    PWM register of 0 = LED totally ON (LEDs are active-low)
    PWM register of RGB_LED_PWM_PERIOD means LED is totally OFF. */
    RED_PWM  = RGB_LED_PWM_PERIOD - ((uint8_t) colorBalancedRed);
    GREEN_PWM  = RGB_LED_PWM_PERIOD - ((uint8_t) colorBalancedGreen);
    BLUE_PWM  = RGB_LED_PWM_PERIOD - ((uint8_t) colorBalancedBlue);
}

/** Simple test of RGB LED. */
void halRgbLedTest()
{
    int i;
    for (i=0; i< 0xFF; i++)
    {
        halRgbSetLeds(0xFF-i,0xFF,0xFF);
        delayMs(5);
    }
    for (i=0; i< 0xFF; i++)
    {
        halRgbSetLeds(i,0xFF,0xFF);
        delayMs(5);
    }
    
    for (i=0; i< 0xFF; i++)
    {
        halRgbSetLeds(0xFF,0xFF-i,0xFF);
        delayMs(5);
    }        
    for (i=0; i< 0xFF; i++)
    {
        halRgbSetLeds(0xFF,i,0xFF);
        delayMs(5);
    }
    
    for (i=0; i< 0xFF; i++)
    {
        halRgbSetLeds(0xFF,0xFF,0xFF-i);
        delayMs(5);
    }        
    for (i=0; i< 0xFF; i++)
    {
        halRgbSetLeds(0xFF,0xFF,i);
        delayMs(5);
    }
}

/* @} */
