/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    ccard.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "ccard.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// Taken from ccard.h
uint32_t tes_reset_port[NUM_TES_CHANNELS] = { 4,  6,  6,  4,  0,  5,  3,  3,  3,  3,  3,  3, 4  };
uint32_t tes_reset_bit[NUM_TES_CHANNELS]  = { 3,  13, 14, 0,  6,  0,  6,  4,  12, 11, 2,  9, 5  };
uint32_t tes_set_port[NUM_TES_CHANNELS]   = { 4,  4,  6,  4,  0,  5,  3,  3,  3,  3,  3,  3, 6  };
uint32_t tes_set_bit[NUM_TES_CHANNELS]    = { 4,  2,  12, 1,  7,  1,  7,  5,  13, 10, 3,  8, 15 };

// Relays default state
uint32_t RELAY_DEFAULT = 0x00;
bool RELAY_LATCHING = true;

typedef uint32_t SPI_DATA_TYPE;
DRV_HANDLE            SPIHandle;
DRV_SPI_BUFFER_HANDLE Write_Buffer_Handle; // Write buffer handle
DRV_SPI_BUFFER_HANDLE Read_Buffer_Handle; // Read buffer handle
SPI_DATA_TYPE         TXbuffer[6]; // SPI Driver TX buffer
SPI_DATA_TYPE         RXbuffer[6]; // SPI Driver RX buffer
uint32_t              SPI_BYTES = 4; // So far this is all that works.
DRV_SPI_BUFFER_EVENT  test;
uint32_t              data_bits = 20; // number of data bits

// Helper functions
static inline bool cmd_read(uint32_t data)
{
    return(!!(data & 0x80000000));
};
// check read bit
static inline uint32_t cmd_address(uint32_t data){
    return((data & 0x7FF00000)>> 20);
};

static inline uint32_t cmd_data(uint32_t data)
{
    return(data & 0xFFFFF);
};

static inline uint32_t make_cmd(bool read, uint32_t address, uint32_t data)
{
    return((read << 31) | ((address & (1 << (32-data_bits-1))-1) << data_bits) | (data & ((1 <<data_bits)-1)) );
};

// Variables used as register maps
uint32_t firmware_version;  // Firmware version
uint32_t status;            // Status register
uint32_t relay;             // TES relays control
uint32_t hemt_bias;         // HEMT bias value
uint32_t a50k_bias;         // 50K bias value
uint32_t hemt2_bias;        // HEMT bias value 2
uint32_t a50k2_bias;        // 50K bias value 2
uint32_t temperature;       // Temperature value
uint32_t cycle_count;       // Cycles counts
uint32_t ps_en;             // Power supplies (HEMT and 50k) enable
uint32_t flux_ramp_control; // Flux ramp (voltage and current mode) controls, no longer used
uint32_t id_volt;           // ID voltage value
uint32_t id2_volt;          // ID voltage value, 2


bool relay_busy = false;

uint32_t default_addr;        // Address used when an invalid address is received
uint32_t *regptr[ADDR_COUNT]; // will hold pointers to various registers

// Accumulator variables used to average the ADC readings
uint32_t adc_samples_cnt;   // Number of ADC samples accumulated
uint32_t hemt_bias_acc;     // HEMT bias accumulator
uint32_t a50k_bias_acc;     // 50K bias accumulator
uint32_t temperature_acc;   // Temperature accumulator
uint32_t id_volt_acc;       // ID voltage accumulator
uint32_t hemt2_bias_acc;    // HEMT bias accumulator
uint32_t a50k2_bias_acc;    // 50K bias accumulator
uint32_t id2_volt_acc;      // ID voltage accumulator

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

CCARD_DATA ccardData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void CCARD_Initialize ( void )

  Remarks:
    See prototype in ccard.h.
 */

void CCARD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ccardData.state = CCARD_STATE_INIT;

    TXbuffer[0] = 0;
}


/******************************************************************************
  Function:
    void CCARD_Tasks ( void )

  Remarks:
    See prototype in ccard.h.
 */

void CCARD_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( ccardData.state )
    {
        /* Application's initial state. */
        case CCARD_STATE_INIT:
        {
            // Set initial register values
            firmware_version  = FIRMWARE_VERSION; // Set firmware version value
            status            = 0;                // Clear status word
            relay             = RELAY_DEFAULT;    // Initial relay state
            cycle_count       = 0;                // Clear cycle counter
            ps_en             = 0;                // Power supplies are disabled
            flux_ramp_control = 0;                // Flux ramp are DC coupled
            default_addr      = 0x00;             // Used until set to some specific value

            // Set TES relays to default value
            TES_relay_set(relay);

            // Start relay timer
            relay_busy = true;
            ccardData.hDelayTimer = SYS_TMR_DelayMS(RELAY_DELAY); // start relay timer

            // Disable power supplies
            PS_HEMT_ENOff();
            PS_50k_ENOff();
            PS_HEMT2_ENOff();
            PS_50K2_ENOff();

            // set up register map
            regptr[ADDR_VERSION]      = &firmware_version;
            regptr[ADDR_STATUS]       = &status;
            regptr[ADDR_RELAY]        = &relay;
            regptr[ADDR_HEMT_BIAS]    = &hemt_bias;
            regptr[ADDR_50K_BIAS]     = &a50k_bias;
            regptr[ADDR_TEMPERATURE]  = &temperature;
            regptr[ADDR_COUNTER]      = &cycle_count;
            regptr[ADDR_PS_EN]        = &ps_en;
            regptr[ADDR_FLUX_RAMP]    = &flux_ramp_control;
            regptr[ADDR_ID_VOLT]      = &id_volt;
            regptr[ADDR_HEMT2_BIAS]   = &hemt2_bias;
            regptr[ADDR_50K2_BIAS]    = &a50k2_bias;
            regptr[ADDR_ID2_VOLT]     = &id2_volt;

            // this is the SPI used for receiving commands
            SPIHandle = DRV_SPI_Open(DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE );

            // read buffer
            Read_Buffer_Handle = DRV_SPI_BufferAddWriteRead(
                    SPIHandle,
                    (SPI_DATA_TYPE *)& TXbuffer[0],
                    SPI_BYTES,
                    (SPI_DATA_TYPE *)& RXbuffer[0],
                    SPI_BYTES,
                    0,
                    0);

            // Reset the number of accumulated ADC samples
            adc_samples_cnt = 0;

            // Start ADC
            DRV_ADC_Open();
            DRV_ADC_Start();

            ccardData.state = CCARD_STATE_SERVICE_TASKS;

            break;
        }

        case CCARD_STATE_SERVICE_TASKS:
        {
            if (SYS_TMR_DelayStatusGet(ccardData.hDelayTimer)) // relay timer timed out
            {
                ccardData.state = CCARD_RELAY_TIMEOUT; // need to clear relays
                break;
            }
            if (DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(Read_Buffer_Handle)) // check for SPI data
            {
                ccardData.state = CCARD_READ_SPI;  // need to read SPI data
                break;
            }

            if (DRV_ADC_SamplesAvailable())
            {
                DRV_ADC_Stop();
                ccardData.state = CCARD_READ_ADC;
                break;
            }

            break;
        }

        case CCARD_RELAY_TIMEOUT:  // reset relays for latching mode of operation
        {
            if (RELAY_LATCHING)
            {
                TES_relay_clear(); // clears relay drive
            }
            relay_busy = false;  // relay done
            relay = relay & ((1 << (data_bits-1))-1);  // clear relay busy bit
            ccardData.state = CCARD_STATE_SERVICE_TASKS;
            break;
        }

        /* TODO: implement your application state machine.*/
        case CCARD_READ_SPI:  // this is where we receive commands
        {
            cycle_count++;
            uint32_t command = RXbuffer[0];               // Command from controller
            bool     rd      = cmd_read(command);         // Read/write flag from command
            uint32_t addr    = cmd_address(command);      // Register address from command
            uint32_t data    = cmd_data(command);         // Register data from command
            ccardData.state  = CCARD_STATE_SERVICE_TASKS; // May be overridden later

            if (rd)
            {
                // Read command received.

                if (addr < ADDR_COUNT)  // address in range
                {
                    TXbuffer[0] = make_cmd(0,  addr, *regptr[addr] );
                    //TXbuffer[0] = 0x01;
                    default_addr = addr; // this is now the default read back.
                }
                else
                {
                    TXbuffer[0] = make_cmd(0, default_addr, *regptr[default_addr]);
                }

                // start new data read
                Read_Buffer_Handle = DRV_SPI_BufferAddWriteRead(
                        SPIHandle,
                        (SPI_DATA_TYPE *)& TXbuffer[0],
                        SPI_BYTES,
                        (SPI_DATA_TYPE *)& RXbuffer[0],
                        SPI_BYTES,
                        0,
                        0);
            }
            else
            {
                // Write command received.

                TXbuffer[0] = make_cmd(0, default_addr, *regptr[default_addr]);
                //TXbuffer[0] = 0x01;

                switch (addr)
                {
                    case ADDR_RELAY:
                    {
                        // Only NUM_TES_CHANNELS bits are used.
                        relay = data & ((1 << NUM_TES_CHANNELS) - 1);
                        if (relay_busy) break;  // just ignore if relay already busy
                        relay_busy = true;

                        // Set the relay  mask
                        TES_relay_set(relay);
                        ccardData.hDelayTimer = SYS_TMR_DelayMS(RELAY_DELAY); // start relay timer
                        break;
                    }
                    case ADDR_PS_EN:
                    {
                        // Only 2 bits are used
                        ps_en = data & 0x03;

                        // HEMT_EN (bit 0)
                        PS_HEMT_ENStateSet( ps_en & 0x01 );

                        // 50k_EN (bit 1)
                        PS_50k_ENStateSet( ( ps_en >> 1 ) & 0x01 );

                        // HEMT2_EN (bit 2)
                        PS_HEMT2_ENStateSet( ( ps_en >> 2) & 0x01 );

                        // 50k2_EN (bit 3)
                        PS_50K2_ENStateSet( ( ps_en >> 3 ) & 0x01 );

                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                Read_Buffer_Handle = DRV_SPI_BufferAddWriteRead(
                        SPIHandle,
                        (SPI_DATA_TYPE *)& TXbuffer[0],
                        SPI_BYTES,
                        (SPI_DATA_TYPE *)& RXbuffer[0],
                        SPI_BYTES,
                        0,
                        0); // start new data read
            }
            break;
        }

        case CCARD_READ_ADC:
        {

            // Accumulate the ADC values
            a50k_bias_acc   += DRV_ADC_SamplesRead(ADC_50K_BIAS_CHAN);
            temperature_acc += DRV_ADC_SamplesRead(ADC_TEMPERATURE_CHAN);
            hemt_bias_acc   += DRV_ADC_SamplesRead(ADC_HEMT_BIAS_CHAN);
            id_volt_acc     += DRV_ADC_SamplesRead(ADC_ID_VOLT_CHAN);
            a50k2_bias_acc  += DRV_ADC_SamplesRead(ADC_50K2_BIAS_CHAN);
            hemt2_bias_acc  += DRV_ADC_SamplesRead(ADC_HEMT2_BIAS_CHAN);
            id2_volt_acc    += DRV_ADC_SamplesRead(ADC_ID2_VOLT_CHAN);

            // start ADC running again
            DRV_ADC_Start();

            if (++adc_samples_cnt >= ADC_CHAN_SAMPLE_COUNT)
            {
                // Move the accumulated values to the registers
                a50k_bias   = a50k_bias_acc;
                temperature = temperature_acc;
                hemt_bias   = hemt_bias_acc;
                id_volt     = id_volt_acc;
                a50k2_bias  = a50k2_bias_acc;
                hemt2_bias  = hemt2_bias_acc;
                id2_volt    = id2_volt_acc;

                // Reset accumulators
                a50k_bias_acc   = 0;
                temperature_acc = 0;
                hemt_bias_acc   = 0;
                id_volt_acc     = 0;
                a50k2_bias_acc  = 0;
                hemt2_bias_acc  = 0;
                id2_volt_acc    = 0;

                // Reset accumulated ADC sample counter
                adc_samples_cnt = 0;
            }

            ccardData.state = CCARD_STATE_SERVICE_TASKS;
            break;
         }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}



/*******************************************************************************
 End of File
 */
