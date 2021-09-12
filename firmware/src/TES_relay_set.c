/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#include "ccard.h"

void TES_relay_set(uint32_t x)
{
    unsigned int n;
    for (n = 0; n < NUM_TES_CHANNELS; n++)
    {
        if (x & (0x1 << n))
        {
            PLIB_PORTS_PinSet(PORTS_ID_0, tes_set_port[n], tes_set_bit[n]);
        }
        else
        {
            PLIB_PORTS_PinClear(PORTS_ID_0, tes_set_port[n], tes_set_bit[n]);
            if (RELAY_LATCHING)  // Need to fire reset test channels
            {
              PLIB_PORTS_PinSet(PORTS_ID_0, tes_reset_port[n], tes_reset_bit[n]);
            }
        }
   }

}

void TES_relay_clear(void)
{
    unsigned int n;
    for (n = 0; n < NUM_TES_CHANNELS; n++)
    {
        PLIB_PORTS_PinClear(PORTS_ID_0, tes_set_port[n], tes_set_bit[n]);
        PLIB_PORTS_PinClear(PORTS_ID_0, tes_reset_port[n], tes_reset_bit[n]);
    }
}
