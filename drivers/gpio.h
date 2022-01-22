#ifndef GPIO_H_
#define GPIO_H_
/******************************************************************************
* Includes
*******************************************************************************/
#include "gpio_config.h"
#include <stdint.h>

/******************************************************************************
* Typedefs
*******************************************************************************/
typedef enum{
    GpioNOK,
    GpioOK,
    GpioStatusMax
}GpioStatus_t;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
GpioStatus_t Gpio_Init(const GpioConfig_t *const Config);

GpioStatus_t Gpio_DeInit(const GpioConfig_t *const Config);

GpioPinState_t Gpio_PinRead(const GpioPin_t Pin);

GpioStatus_t Gpio_PinWrite(const GpioPin_t Pin, const GpioPinState_t State);

GpioStatus_t Gpio_PinToggle(const GpioPin_t Pin);

GpioStatus_t Gpio_PinModeSet(const GpioPin_t Pin, const GpioMode_t Mode);

GpioStatus_t Gpio_PullDirectionSet(const GpioPin_t Pin, const PullDir_t Dir);

GpioStatus_t Gpio_PinSpeedSet(const GpioPin_t Pin, const PinSpeed_t Speed);

GpioStatus_t Gpio_OutputTypeSet(const GpioPin_t Pin, const OutputType_t OutputType);

GpioStatus_t Gpio_IrqConfig(const GpioPin_t Pin, const GpioInterruptMode_t IrqMode);

GpioStatus_t Gpio_CallbackRegister(const GpioPin_t Pin, const GpioCallback_t Function);

#endif