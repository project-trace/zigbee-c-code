/**
* @ingroup apps
* @{
*
* @file example_simple_application_coordinator_afzdo.c
*
* @brief Resets Module, configures this device to be a Zigbee Coordinator, and displays any messages
* that are received. If the message contains a known object-identifier (OID) then parses the
* contents of the OID. Also displays received value to RGB LED and allows user to use the pushbutton
* to select which OID is displayed on the RGB LED.
*
* Uses the AF/ZDO interface.
*
* $Rev: 1918 $
* $Author: dsmith $
* $Date: 2013-09-27 16:36:38 -0700 (Fri, 27 Sep 2013) $
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

#include "../HAL/hal.h"
#include "../ZM/module.h"
#include "../ZM/application_configuration.h"
#include "../ZM/af.h"
#include "../ZM/zdo.h"
#include "../ZM/zm_phy_spi.h"
#include "../ZM/module_errors.h"
#include "../ZM/module_utilities.h"
#include "../Common/utilities.h"
#include "Messages/infoMessage.h"
#include "Messages/kvp.h"
#include "Messages/oids.h"
#include "module_example_utils.h"
#include <stdint.h>

static void parseMessages();

/** Function pointer (in hal file) for the function that gets called when a button is pressed*/
extern void (*buttonIsr)(int8_t);

/** Our button interrupt handler */
static void handleButtonPress(int8_t button);

/** STATES for state machine */
enum STATE
{
    STATE_IDLE,
    STATE_MODULE_STARTUP,
    STATE_DISPLAY_NETWORK_INFORMATION,
};

/** STATES for tracking algorithm */
enum TRACK_STATE
{
   ALL_ITEMS_CONNECTED,
   SUSPECTED_ITEM_LOSS,
   ITEM_LOST_ALARM,
   ITEM_LOST_SILENCED,
};

/** This is the current state of the application. 
* Gets changed by other states, or based on messages that arrive. */
enum STATE state = STATE_MODULE_STARTUP;

/** The main application state machine */
static void stateMachine();

/* Tracking state machine */
static void trackingStateMachine(int router_index);

/** Various utility functions */
static char* getRgbLedDisplayModeName(uint8_t mode);
static uint8_t setModuleLeds(uint8_t mode);

#define STATE_FLAG_MESSAGE_WAITING      0x01
#define STATE_FLAG_BUTTON_PRESSED      0x02
/** Various flags between states */
volatile uint16_t stateFlags = 0;

#define LQI_THRESHOLD                   0x50
#define LQI_NUM_SAMPLES                 6
#define NUM_DEVICES                     10
int DEVICES_REGISTERED = 0;

int current_router_index = 0;
int coordinator_on = 1;

#define BUZZER                          BIT0

struct router_device {
  /** State of the tracking algorithm state machine */
  enum TRACK_STATE track_state;
  uint8_t MAC_address[8];
  uint8_t LQI_running_average[LQI_NUM_SAMPLES];
  uint8_t LQI;
  uint8_t LQI_iter;
  uint16_t LQI_total;
  uint8_t LQI_average;
  uint8_t LQI_initialized;
};

struct router_device routers[NUM_DEVICES];
uint8_t alarm_sounding = 0;
uint8_t alarm_silenced = 0;
uint8_t program_mode = 0;

void structInit();

#define NWK_OFFLINE                     0
#define NWK_ONLINE                      1
/** Whether the zigbee network has been started, etc.*/
uint8_t zigbeeNetworkStatus = NWK_OFFLINE;

#define RGB_LED_DISPLAY_MODE_NONE           0
#define RGB_LED_DISPLAY_MODE_TEMP_IR        1
#define RGB_LED_DISPLAY_MODE_COLOR          2
#define RGB_LED_DISPLAY_MODE_MAX            RGB_LED_DISPLAY_MODE_COLOR

/* What to display on the RGB LED */
uint8_t rgbLedDisplayMode = 0;

extern uint8_t zmBuf[ZIGBEE_MODULE_BUFFER_SIZE];

void addMacAddress(int index, uint8_t* mac_address);

//uncomment below to see more information about the messages received.
//#define VERBOSE_MESSAGE_DISPLAY

int main( void )
{
    structInit();
    halInit();
    moduleInit();
    buttonIsr = &handleButtonPress;    
    printf("\r\n****************************************************\r\n");
    printf("Simple Application Example - COORDINATOR\r\n");
    
    routers[0].MAC_address[0] = 0x5E;
    routers[0].MAC_address[1] = 0xD2;
    routers[0].MAC_address[2] = 0x5D;
    routers[0].MAC_address[3] = 0x02;
    routers[0].MAC_address[4] = 0x00;
    routers[0].MAC_address[5] = 0x4B;
    routers[0].MAC_address[6] = 0x12;
    routers[0].MAC_address[7] = 0x00;
    
    routers[1].MAC_address[0] = 0xD3;
    routers[1].MAC_address[1] = 0xD3;
    routers[1].MAC_address[2] = 0x5D;
    routers[1].MAC_address[3] = 0x02;
    routers[1].MAC_address[4] = 0x00;
    routers[1].MAC_address[5] = 0x4B;
    routers[1].MAC_address[6] = 0x12;
    routers[1].MAC_address[7] = 0x00;
    
    HAL_ENABLE_INTERRUPTS();
    clearLeds();
    
    halRgbLedPwmInit();
    
    while (1) {
        stateMachine();    //run the state machine
        if (alarm_sounding == 1 && !alarm_silenced) {
          delayMs(2);
          toggleLed(0);
        }
    }
}

void structInit() {
  
  int i;
  for (i = 0; i < NUM_DEVICES; i++) {
    routers[i].LQI = 0;
    routers[i].LQI_average = 0;
    routers[i].LQI_initialized = 0;
    routers[i].LQI_iter = 0;
    routers[i].LQI_total = 0;

    int j = 0;
    for (j = 0; j < 8; j++) {
      routers[i].MAC_address[i] = 0;
    }

    int k = 0;
    for (k = 0; k < LQI_NUM_SAMPLES; k++) {
      routers[i].LQI_running_average[k] = 0;
    }
      
    routers[i].track_state = ALL_ITEMS_CONNECTED;
  }
}

/*
void addMacAddress(int index, uint8_t* mac_address) {
    routers[index].MAC_address[0] = mac_address[0];
    routers[index].MAC_address[1] = mac_address[1];
    routers[index].MAC_address[2] = mac_address[2];
    routers[index].MAC_address[3] = mac_address[3];
    routers[index].MAC_address[4] = mac_address[4];
    routers[index].MAC_address[5] = mac_address[5];
    routers[index].MAC_address[6] = mac_address[6];
    routers[index].MAC_address[7] = mac_address[7];
}*/

/** 
Called from state machine when a button was pressed. 
Selects which KVP value is displayed on the RGB LED. 
*/
void processButtonPress()
{
  if (alarm_sounding == 1) {
      halRgbSetLeds(0, 0xFF, 0);
      alarm_silenced = 1;
  }
}

void processButtonHold()
{
  if (coordinator_on == 0) {
    coordinator_on = 1;
    halRgbSetLeds(0, 0, 0);
  }
  else {
    coordinator_on = 0;
    halRgbSetLeds(0xFF, 0xFF, 0xFF);
  }
}

/** 
Simple button debouncing routine. Polls the button every few milliseconds and adds up the number of 
times that button is ON vs. OFF. At the end of the time period if the number of times that the 
button is ON is greater than the number of times that it is OFF then the button is determined to be 
pressed.
@return 1 if button is pressed, else 0
@note If you have more sophisticated processor (e.g. more timers available) then there are better 
ways of implementing this.
*/
static uint8_t debounceButton(uint8_t button)
{
#define BUTTON_DEBOUNCE_TIME_MS  150    // How long to poll the button, total
#define BUTTON_POLL_INTERVAL_MS 5       // How long to wait between polling button
    int16_t time = 0;                   // The amount of time that has elapsed in the debounce routine
    int16_t buttonOnCount = 0;          // Number of times button was polled and ON
    int16_t buttonOffCount = 0;         // Number of times button was polled and OFF 
    
    while (time < BUTTON_DEBOUNCE_TIME_MS)
    {
        if (buttonIsPressed(button))
            buttonOnCount++;
        else
            buttonOffCount++;
        time += BUTTON_POLL_INTERVAL_MS;
        delayMs(BUTTON_POLL_INTERVAL_MS);
    }
    
    return (buttonOnCount > buttonOffCount);
}

static uint8_t debounceButtonHold(uint8_t button)
{
#define BUTTON_DEBOUNCE_HOLD_TIME_MS  5000    // How long to poll the button, total
#define BUTTON_POLL_INTERVAL_MS 5       // How long to wait between polling button
    int16_t time = 0;                   // The amount of time that has elapsed in the debounce routine
    int16_t buttonOnCount = 0;          // Number of times button was polled and ON
    int16_t buttonOffCount = 0;         // Number of times button was polled and OFF 
    
    while (time < BUTTON_DEBOUNCE_HOLD_TIME_MS)
    {
        if (buttonIsPressed(button))
            buttonOnCount++;
        else
            buttonOffCount++;
        time += BUTTON_POLL_INTERVAL_MS;
        delayMs(BUTTON_POLL_INTERVAL_MS);
    }
    
    return (buttonOnCount > buttonOffCount);
}

/** 
The main state machine for the application.
Never exits.
*/
void stateMachine()
{
 //   while (1)
  //  {
        if (zigbeeNetworkStatus == NWK_ONLINE)
        {
            if(moduleHasMessageWaiting())      //wait until SRDY goes low indicating a message has been received. 
                stateFlags |= STATE_FLAG_MESSAGE_WAITING;
        }
        
        switch (state)
        {
        case STATE_IDLE:
            {
                if (stateFlags & STATE_FLAG_MESSAGE_WAITING & coordinator_on)    // If there is a message waiting...
                {
                    parseMessages();                            // ... then display it
                    trackingStateMachine(current_router_index);
                    stateFlags &= ~STATE_FLAG_MESSAGE_WAITING;
                }
                
                if (stateFlags & STATE_FLAG_BUTTON_PRESSED)     // If ISR set this flag...
                {
                    if (debounceButton(ANY_BUTTON))             // ...then debounce it
                    {
                        processButtonPress();                   // ...and process it
                    }
                    if (debounceButtonHold(ANY_BUTTON))
                    {
                        processButtonHold();
                    }
                    stateFlags &= ~STATE_FLAG_BUTTON_PRESSED;
                }
                
                /* Other flags (for different messages or events) can be added here */
                break;
            }
            
        case STATE_MODULE_STARTUP:              // Start the Zigbee Module on the network
            {
#define MODULE_START_DELAY_IF_FAIL_MS 5000
                moduleResult_t result;
                struct moduleConfiguration defaultConfiguration = DEFAULT_MODULE_CONFIGURATION_COORDINATOR;
                
                /* Uncomment below to restrict the device to a specific PANID
                defaultConfiguration.panId = 0x1234;
                */
                
                /* Below is an example of how to restrict the device to only one channel:
                defaultConfiguration.channelMask = CHANNEL_MASK_17;
                printf("DEMO - USING CUSTOM CHANNEL 17\r\n");
                */
                
                while ((result = startModule(&defaultConfiguration, GENERIC_APPLICATION_CONFIGURATION)) != MODULE_SUCCESS)
                {
                    printf("FAILED. Error Code 0x%02X. Retrying...\r\n", result);
                    delayMs(MODULE_START_DELAY_IF_FAIL_MS);
                }
                //printf("Success\r\n");
                zigbeeNetworkStatus = NWK_ONLINE;
                
                state = STATE_DISPLAY_NETWORK_INFORMATION;
                break;
            }
        case STATE_DISPLAY_NETWORK_INFORMATION:
            {
                printf("~ni~");
                /* On network, display info about this network */
                displayNetworkConfigurationParameters();
                displayDeviceInformation();
                if (sysGpio(GPIO_SET_DIRECTION, ALL_GPIO_PINS) != MODULE_SUCCESS)   //Set module GPIOs as output
                {
                    printf("ERROR\r\n");
                }
                /*
                printf("Press button to change which received value is displayed on RGB LED. D6 & D5 will indicate mode:\r\n");
                printf("    None = None\r\n");
                printf("    Yellow (D9) = IR Temp Sensor\r\n");
                printf("    Red (D8) = Color Sensor\r\n");
                */
                printf("Displaying Messages Received\r\n");
                setModuleLeds(RGB_LED_DISPLAY_MODE_NONE);
                
                /* Now the network is running - wait for any received messages from the ZM */
#ifdef VERBOSE_MESSAGE_DISPLAY    
                printAfIncomingMsgHeaderNames();
#endif                
                state = STATE_IDLE;
                break;
            }
            
        default:     //should never happen
            {
                printf("UNKNOWN STATE\r\n");
                state = STATE_MODULE_STARTUP;
            }
            break;
        }
 //   } 
}    

void trackingStateMachine(int router_index) {
  if (routers[router_index].LQI != 0) {
    
    if (routers[router_index].LQI_iter == LQI_NUM_SAMPLES) {
      routers[router_index].LQI_iter = 0;
      routers[router_index].LQI_initialized = 1;
    }
    
    uint8_t oldest_LQI = routers[router_index].LQI_running_average[(routers[router_index].LQI_iter)];
    routers[router_index].LQI_running_average[routers[router_index].LQI_iter] = routers[router_index].LQI;
    
    if (routers[router_index].LQI_initialized == 1)
      routers[router_index].LQI_total -= oldest_LQI;
    routers[router_index].LQI_total += routers[router_index].LQI;

    routers[router_index].LQI_average = routers[router_index].LQI_total / LQI_NUM_SAMPLES;
    routers[router_index].LQI_iter++;
    
    int i, j, k;
    for (i = 0; i < NUM_DEVICES; i++) {
      printf("Most recent LQI value: %02X\r\n", routers[i].LQI);      
      
      printf("LQI ARRAY for device at MAC address: ");
      for (k = 7; k >= 0; k--) {
        printf("%02X", routers[i].MAC_address[k]);
      }
      printf("\r\n");
      for (j = 0; j < LQI_NUM_SAMPLES; j++) {
        printf("%d:", j);
        printf("%02X ", routers[i].LQI_running_average[j]);
      }
      printf("\r\n");
      printf("AVERAGE: %02X\r\n", routers[i].LQI_average);
    }
   
    switch(routers[router_index].track_state) {
      
    case ALL_ITEMS_CONNECTED:
      alarm_silenced = 0;
      if (alarm_sounding == 0) {
        int items_connected = 0;
        int j;
        for (j = 0; j < NUM_DEVICES; j++) {
          if (routers[router_index].track_state == ALL_ITEMS_CONNECTED)
            items_connected++;
        }
        if (items_connected == NUM_DEVICES)
          printf("ALL DEVICES CONNECTED\r\n");

        halRgbSetLeds(0, 0, 0xFF);
      }
      if (routers[router_index].LQI_average < LQI_THRESHOLD && routers[router_index].LQI_initialized == 1) {
          routers[router_index].track_state = ITEM_LOST_ALARM;
      }
      break;
    /*
    case SUSPECTED_ITEM_LOSS:
      halRgbSetLeds(0, 0xFF, 0);
      if (LQI_average < LQI_THRESHOLD && LQI_initialized == 1) {
          track_state = ITEM_LOST_ALARM;
      }
      else {
        track_state = ALL_ITEMS_CONNECTED;
      }
      break;
    */
    case ITEM_LOST_ALARM:
      printf("LOST ITEM AT ROUTER INDEX: %d\r\n", router_index);
      if (alarm_sounding == 0) {
        halRgbSetLeds(0xFF, 0, 0);
        alarm_sounding = 1;
      }
      if (routers[router_index].LQI_average > LQI_THRESHOLD) {
        routers[router_index].track_state = ALL_ITEMS_CONNECTED;
      }
      int i;
      int devices_connected = 0;
      for (i = 0; i < NUM_DEVICES; i++) {
        if (routers[router_index].track_state == ALL_ITEMS_CONNECTED)
          devices_connected++;
      }
      if (devices_connected == NUM_DEVICES)
        alarm_sounding = 0;
      break;
    
      /*
    case ITEM_LOST_SILENCED:
      halRgbSetLeds(0, 0xFF, 0);
      if (routers[router_index].LQI_average > LQI_THRESHOLD) {
        routers[router_index].track_state = ALL_ITEMS_CONNECTED;
      }
      break;  
      */
    }
  }
}


/** Parse any received messages. If it's one of our OIDs then display the value on the RGB LED too. */
void parseMessages()
{
    getMessage();
    if ((zmBuf[SRSP_LENGTH_FIELD] > 0) && (IS_AF_INCOMING_MESSAGE()))
    {
        setLed(4);                                  //LED will blink to indicate a message was received
#ifdef VERBOSE_MESSAGE_DISPLAY
        printAfIncomingMsgHeader(zmBuf);
        printf("\r\n");
#endif
        if ((AF_INCOMING_MESSAGE_CLUSTER()) == INFO_MESSAGE_CLUSTER)
        {
            struct infoMessage im;
            deserializeInfoMessage(zmBuf+20, &im);  // Convert the bytes into a Message struct
            int j = 0;
#ifdef VERBOSE_MESSAGE_DISPLAY                
            printInfoMessage(&im);
            displayZmBuf();
#else
            printf("From:");                        // Display the sender's MAC address
            for (j = 7; j>(-1); j--)
            {
                printf("%02X", im.header.mac[j]);
            }
            int k;
            for (k = 0; k < NUM_DEVICES; k++) {
              int match = 1;
              for (j = 7; j>(-1); j--) {
                if (routers[k].MAC_address[j] != im.header.mac[j]) {
                  match = 0;
                }
              }
              if (match == 1) {
                current_router_index = k;
                routers[current_router_index].LQI = zmBuf[AF_INCOMING_MESSAGE_LQI_FIELD];
              }
            }
            
            printf(", LQI=%02X, ", zmBuf[AF_INCOMING_MESSAGE_LQI_FIELD]);   // Display the received signal quality (Link Quality Indicator)
            //LQI = zmBuf[AF_INCOMING_MESSAGE_LQI_FIELD];

#endif
            printf("%u KVPs received:\r\n", im.numParameters);
#define NO_VALUE_RECEIVED   0xFF
            uint8_t redIndex = NO_VALUE_RECEIVED; 
            uint8_t blueIndex = NO_VALUE_RECEIVED;
            uint8_t greenIndex = NO_VALUE_RECEIVED;
            for (j=0; j<im.numParameters; j++)                              // Iterate through all the received KVPs
            {
                printf("    %s (0x%02X) = %d  ", getOidName(im.kvps[j].oid), im.kvps[j].oid, im.kvps[j].value);    // Display the Key & Value
                displayFormattedOidValue(im.kvps[j].oid, im.kvps[j].value);
                printf("\r\n");
                // If the received OID was an IR temperature OID then we can just display it on the LED
                if ((rgbLedDisplayMode == RGB_LED_DISPLAY_MODE_TEMP_IR) && (im.kvps[j].oid == OID_TEMPERATURE_IR)) 
                    displayTemperatureOnRgbLed(im.kvps[j].value);
                // But for the color sensor we need to get all three values before displaying
                else if (im.kvps[j].oid == OID_COLOR_SENSOR_RED)
                    redIndex = j;
                else if (im.kvps[j].oid == OID_COLOR_SENSOR_BLUE)
                    blueIndex = j;
                else if (im.kvps[j].oid == OID_COLOR_SENSOR_GREEN)
                    greenIndex = j;
            }
            // Now done iterating through all KVPs. If we received color then update RGB LED
#define RED_VALUE   (im.kvps[redIndex].value)
#define BLUE_VALUE  (im.kvps[blueIndex].value)
#define GREEN_VALUE (im.kvps[greenIndex].value)
            
            if ((rgbLedDisplayMode == RGB_LED_DISPLAY_MODE_COLOR) && 
                ((redIndex != NO_VALUE_RECEIVED) && (blueIndex != NO_VALUE_RECEIVED) && (greenIndex != NO_VALUE_RECEIVED)))
            {
                displayColorOnRgbLed(RED_VALUE, BLUE_VALUE, GREEN_VALUE);
            }
            printf("\r\n");
            
        } else {
            printf("Rx: ");
            printHexBytes(zmBuf+SRSP_HEADER_SIZE+17, zmBuf[SRSP_HEADER_SIZE+16]);   //print out message payload
        }
        clearLeds(0);    
    } else if (IS_ZDO_END_DEVICE_ANNCE_IND()) {
        displayZdoEndDeviceAnnounce(zmBuf);
    } else { //unknown message, just print out the whole thing
        printf("MSG: ");
        printHexBytes(zmBuf, (zmBuf[SRSP_LENGTH_FIELD] + SRSP_HEADER_SIZE));
    }
    zmBuf[SRSP_LENGTH_FIELD] = 0;
}


/* 
Displays the pretty name of the LED display mode.
@param mode which LED display mode
@return name of mode, or "UNKNOWN" if not known
*/
static char* getRgbLedDisplayModeName(uint8_t mode)
{
    switch(mode)
    {
    case RGB_LED_DISPLAY_MODE_NONE:
        return "RGB_LED_DISPLAY_MODE_NONE";
    case RGB_LED_DISPLAY_MODE_TEMP_IR:
        return "RGB_LED_DISPLAY_MODE_TEMP_IR";
    case RGB_LED_DISPLAY_MODE_COLOR:
        return "RGB_LED_DISPLAY_MODE_COLOR";
    default:
        return "UNKNOWN";
    }
}


/** 
Sets the module LEDs to the selected mode.
On Zigbee BoosterPack, GPIO2 & GPIO3 are connected to LEDs. 
@param mode - LED Display mode
@pre module GPIOs were configured as outputs
@return 0 if success, else error
@note on Zigbee BoosterPack, DIP switch S4 must have switches "3" and "4" set to "ON" to see the LEDs.
*/
static uint8_t setModuleLeds(uint8_t mode)
{
    if (mode > RGB_LED_DISPLAY_MODE_MAX)
        return 1;
    
    mode <<= 2;         // Since GPIO2 & GPIO3 are used, need to shift over 2 bits
    
    if (sysGpio(GPIO_CLEAR, ALL_GPIO_PINS) != MODULE_SUCCESS)   //First, turn all off
    {
        return 2;       // Module error
    }      
    if (mode != 0)      // If mode is 0 then don't leave all off
    {
        if (sysGpio(GPIO_SET, (mode & 0x0C)) != MODULE_SUCCESS)
        {
            return 3;   // Module error
        }
    }
    return 0;
}


/** 
Button interrupt service routine. Called when interrupt generated on the button.
@pre Button connects input to GND.
@pre pins are configured as interrupts appropriately and have pull-UP resistors.
*/
static void handleButtonPress(int8_t button)
{
    stateFlags |= STATE_FLAG_BUTTON_PRESSED;
}

/* @} */
