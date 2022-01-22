/******************************************************************************
* Includes
*******************************************************************************/
#include "gpio_config.h"

/******************************************************************************
* Variable Definitions and Look Up Tables
*******************************************************************************/

/********************************************************************/
/*Configuration table containing all GPIO Settings                  */
/*This will be iterated through during the GpioInit function        */
/********************************************************************/
static const GpioConfig_t config[] = {
    {.Pin = {Pin06, PortA}, .Mode = {Input, Alternate00}, .Pull = PinPulldown, .Speed = SlowSpeed, .OutputType = PushPull, .State = PinLow, .IrqMode = RisingEdge},
    {.Pin = {Pin05, PortA}, .Mode = {Output, Alternate00}, .Pull = PinPullup, .Speed = SlowSpeed, .OutputType = PushPull, .State = PinLow, .IrqMode = IrqOff},
    {.Pin = {Pin04, PortA}, .Mode = {Output, Alternate00}, .Pull = PinPullup, .Speed = SlowSpeed, .OutputType = PushPull, .State = PinLow, .IrqMode = IrqOff},
};

/******************************************************************************
* Function Definitions
*******************************************************************************/

/********************************************************************/
/* Function allowing user to retrieve GpioConfig table              */
/********************************************************************/
const GpioConfig_t *const GetGpioConfig(void){
    return (const GpioConfig_t *const) config;
}