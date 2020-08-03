/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    ccard.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _CCARD_H
#define _CCARD_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// Firmware version. Coded in HEX, 1 byte per digit.
// For example: Version R2.3.1 will be 0x020301
#define FIRMWARE_VERSION 0x020000   // R2.0.0

// Number of TES relays
#define NUM_TES_CHANNELS 12

// ADC channels configurations
// NOTE,Harmony always reads ADC channels in numerical order (ANxx)
#define ADC_50K_BIAS_CHAN     0  // AN3
#define ADC_TEMPERATURE_CHAN  1  // AN4
#define ADC_HEMT_BIAS_CHAN    2  // AN5
#define ADC_ID_VOLT_CHAN      3  // AN6
#define ADC_CHAN_COUNT        4  // Number of ADC channels in use
#define ADC_CHAN_SAMPLE_COUNT 5  // Number of averaged samples per channel

// SPI Function addresses
#define ADDR_VERSION      0x00 // return the firmware version number, no write
#define ADDR_STATUS       0x01 // returns status register, no write
#define ADDR_RELAY        0x02 // relay address
#define ADDR_HEMT_BIAS    0x03 // returns the HEMT bias value, no write
#define ADDR_50K_BIAS     0x04 // returns the 50K bias value, no write
#define ADDR_TEMPERATURE  0x05 // returns the temperature value, no write
#define ADDR_COUNTER      0x06 // return the cycle counts, no write
#define ADDR_PS_EN        0x07 // PS enable (HEMT and 50k)
#define ADDR_FLUX_RAMP    0x08 // Flux ramp controls
#define ADDR_ID_VOLT      0x09 // return the ID voltage value, no write
#define ADDR_COUNT          10 // number of addreses

// *****************************************************************************
// Global, external variable
extern uint32_t tes_port[NUM_TES_CHANNELS];
extern uint32_t tes_bit[NUM_TES_CHANNELS];

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	CCARD_STATE_INIT=0,
	CCARD_STATE_SERVICE_TASKS,
    CCARD_READ_SPI,
    CCARD_READ_ADC, // ADC data ready
            
} CCARD_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    CCARD_STATES state;
    
} CCARD_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void CCARD_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    CCARD_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void CCARD_Initialize ( void );


/*******************************************************************************
  Function:
    void CCARD_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    CCARD_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void CCARD_Tasks( void );


/*******************************************************************************
  Function:
    void TES_relay_set ( unsigned int )
 * 
 * Summary:
 * 
 * Description:
 * 
 * Parameters:
 * 
 * Returns:
 * 
 * Example:
 * 
 * Remarks:
 * 
 */
void TES_relay_set(unsigned int);

#endif /* _CCARD_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

