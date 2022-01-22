/******************************************************************************
* Includes
*******************************************************************************/
#include "gpio.h"
#include <stddef.h>
#include "stm32f4xx.h"
#include "common.h"

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void DefaultHandler(void);

/******************************************************************************
* Variable Definitions and Look Up Tables
*******************************************************************************/

/*************************************************************************/
/*Table to look up GPIO Port Struct pointers using the "GpioPort_t" enum */
/*See : gpio_config.h for enum definition                                */
/*************************************************************************/
static GPIO_TypeDef *const GpioLut[] = {
    #ifdef GPIOA
    GPIOA,
    #else
    NULL,
    #endif

    #ifdef GPIOB
    GPIOB,
    #else
    NULL,
    #endif

    #ifdef GPIOC
    GPIOC,
    #else
    NULL,
    #endif

    #ifdef GPIOD
    GPIOD,
    #else
    NULL,
    #endif

    #ifdef GPIOE
    GPIOE,
    #else
    NULL,
    #endif

    #ifdef GPIOF
    GPIOF,
    #else
    NULL,
    #endif

    #ifdef GPIOG
    GPIOG,
    #else
    NULL,
    #endif

    #ifdef GPIOH
    GPIOH,
    #else
    NULL,
    #endif

    #ifdef GPIOI
    GPIOI,
    #else
    NULL,
    #endif 
};

/**************************************************************************/
/*Table to look up the IRQ Number for an EXTI Line using the Pin Number as*/
/*the index; that is, "GpioPinNumber_t" as the index                      */
/*See : gpio_config.h for enum definition                                 */
/**************************************************************************/
static const IRQn_Type IrqNumbersLut[] = {
    EXTI0_IRQn,
    EXTI1_IRQn,
    EXTI2_IRQn,
    EXTI3_IRQn,
    EXTI4_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn
};

/**************************************************************************/
/*Callback function look up table. Default Handler is provided, but the   */
/*user may provide their own by changing the entries here prior to compile*/ 
/*time. EXTI IRQ Handlers index this using GpioPinNumber_t                */
/*These may also be changed at run time using Gpio_CallbackRegister() fn  */
/**************************************************************************/
GpioCallback_t GpioCallback[NUMBER_OF_EXTI_SIGNALS] = {
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler,
    DefaultHandler
};

/******************************************************************************
* Function Definitions
*******************************************************************************/

/********************************************************/
/*Function to enable GPIO Clock Peripheral              */
/*parameters : GpioPort_t (see gpio_config.h)           */
/********************************************************/
static void ClockEnable(GpioPort_t Port){
    assert_param(IsValidPort(Port));
    SET_BIT(RCC->AHB1ENR, 1 << Port);
}

/********************************************************/
/*Function to disable GPIO Clock Peripheral             */
/*parameters : GpioPort_t (see gpio_config.h)           */
/********************************************************/
static void ClockDisable(GpioPort_t Port){
    assert_param(IsValidPort(Port));
    CLEAR_BIT(RCC->AHB1ENR, 1 << Port);
}

/*************************************************************/
/*Default GPIO EXTI IRQ Callback function                    */
/*User should modify this function for desired functionality */
/*************************************************************/
static void DefaultHandler(){
    return;
}

/*********************************************************************/
/*Function to initialize all GPIO Pins and enable clock              */
/*parameters : Table of GPIO Configurations. Located in GpioConfig.c */
/*return     : GpioOK on Success                                     */
/*             GpioNOK on failure                                    */
/*********************************************************************/
GpioStatus_t Gpio_Init(const GpioConfig_t *const Config){
    uint8_t ConfIdx = 0;
    for(ConfIdx = 0; ConfIdx < NUMBER_OF_PINS_USED; ConfIdx++){

        ClockEnable(Config[ConfIdx].Pin.Port);
        //PinMode
        if (Gpio_PinModeSet(Config[ConfIdx].Pin, Config[ConfIdx].Mode) == GpioNOK){
            return GpioNOK;
        }
        //Pull
        if (Gpio_PullDirectionSet(Config[ConfIdx].Pin, Config[ConfIdx].Pull) == GpioNOK){
            return GpioNOK;
        }
        //Speed
        if (Gpio_PinSpeedSet(Config[ConfIdx].Pin, Config[ConfIdx].Speed) == GpioNOK){
            return GpioNOK;
        }
        //State and Output Type
        if(Config[ConfIdx].Mode.PinMode != Input){
            if(Gpio_OutputTypeSet(Config[ConfIdx].Pin, Config[ConfIdx].OutputType) == GpioNOK){
                return GpioNOK;
            }
            if(Gpio_PinWrite(Config[ConfIdx].Pin, Config[ConfIdx].State) == GpioNOK){
                return GpioNOK;
            }
        }
        //Irq Config
        if(Gpio_IrqConfig(Config[ConfIdx].Pin, Config[ConfIdx].IrqMode) == GpioNOK){
                return GpioNOK;
        }
    }
    return GpioOK;
}

/*********************************************************************/
/*Function to DeInitialize all GPIO Pins and disable clock           */
/*parameters : Table of GPIO Configurations. Located in GpioConfig.c */
/*return     : GpioOK on Success                                     */
/*             GpioNOK on failure                                    */
/*********************************************************************/
GpioStatus_t Gpio_DeInit(const GpioConfig_t *const Config){
    uint8_t ConfIdx = 0;
    GpioMode_t defaut_mode = {Analog, Alternate00};
    //Reset each pin to a 'default' state
    for(ConfIdx = 0; ConfIdx < NUMBER_OF_PINS_USED; ConfIdx++){
        //PinMode
        if (Gpio_PinModeSet(Config[ConfIdx].Pin, defaut_mode) == GpioNOK){
            return GpioNOK;
        }
        //Pull
        if (Gpio_PullDirectionSet(Config[ConfIdx].Pin, PinFloat) == GpioNOK){
            return GpioNOK;
        }
        //Speed
        if (Gpio_PinSpeedSet(Config[ConfIdx].Pin, SlowSpeed) == GpioNOK){
            return GpioNOK;
        }
        //Output Type
        if(Gpio_OutputTypeSet(Config[ConfIdx].Pin, PushPull) == GpioNOK){
            return GpioNOK;
        }
        //State
        if(Gpio_PinWrite(Config[ConfIdx].Pin, PinLow) == GpioNOK){
            return GpioNOK;
        }
        //Irq Config
        if(Gpio_IrqConfig(Config[ConfIdx].Pin, IrqOff) == GpioNOK){
                return GpioNOK;
        }
    }
    //Disable GPIO Clock after pins are reset
    ConfIdx = 0;
    for(ConfIdx = 0; ConfIdx < NUMBER_OF_PINS_USED; ConfIdx++){
        ClockDisable(Config[ConfIdx].Pin.Port);
    }
    //Disable IRQs in NVIC
    GpioPinNumber_t IrqCount = Pin00;
    for(IrqCount = Pin00; IrqCount < PinMax; IrqCount++){
        NVIC_DisableIRQ(IrqNumbersLut[IrqCount]);
    }

    return GpioOK;
}

/********************************************************/
/*Function to read a GPIO Pin                           */
/*parameters : GpioPin_t (Port + PinNumber)             */
/*return     : PinHigh on if pin is high                */
/*             PinLow if pin is low                     */
/********************************************************/
GpioPinState_t Gpio_PinRead(const GpioPin_t Pin){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    //Check if the bit is high and return high if it is
    if (READ_BIT(GpioLut[Pin.Port]->ODR,(1 << Pin.Number))){
        return PinHigh;
    } else {
        return PinLow;
    }
}

/*************************************************************************************/
/*Function to write to GPIO Pin                                                      */
/*parameters : GpioPin_t (Port + PinNumber) and the desired Pin State (low or high)  */
/*return     : GpioOK if ODR changed to desired value                                */
/*             GpioNOK if ODR did not change to correct value                        */
/*************************************************************************************/
GpioStatus_t Gpio_PinWrite(const GpioPin_t Pin, const GpioPinState_t State){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    assert_param(IsValidState(State));

    if(State == PinHigh){
        //Write to output data register
        SET_BIT(GpioLut[Pin.Port]->ODR, 1 << Pin.Number);
        //Validate that the register was written
        if(READ_BIT(GpioLut[Pin.Port]->ODR, (1 << Pin.Number))){
            return GpioOK;
        }
    } else {
        //Clear output data register bit
        CLEAR_BIT(GpioLut[Pin.Port]->ODR, 1 << Pin.Number);
        //Validate that register was cleared
        if(READ_BIT(GpioLut[Pin.Port]->ODR, (1 << Pin.Number))){
            return GpioNOK;
        }
    }
    return GpioOK;
}

/**************************************************************/
/*Function to toggle a GPIO Pin                               */
/*parameters : GpioPin_t (Port + Pin Number)                  */
/*return     : GpioOK if ODR changed to desired value         */
/*             GpioNOK if ODR did not change to desired value */
/**************************************************************/
GpioStatus_t Gpio_PinToggle(const GpioPin_t Pin){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    //Create a copy of the original ODR bit
    uint32_t Original = READ_BIT(GpioLut[Pin.Port]->ODR, 1 << Pin.Number);
    //Toggle bit in ODR
    GpioLut[Pin.Port]->ODR ^= (1 << Pin.Number);
    //Validate that the pin changed state
    if(Original == READ_BIT(GpioLut[Pin.Port]->ODR, 1 << Pin.Number)){
        return GpioNOK;
    }
    return GpioOK;
}

/************************************************************************/
/*Function to configure a Pin to desired pin mode (I/O or Alternate)    */
/*parameters : GpioPin and Desired Mode                                 */
/*return     : GpioOK on MODER and AFSEL reg changing succesfully       */
/*             GpioNOK on if MODER and AFSEL did not change succesfully */
/************************************************************************/
GpioStatus_t Gpio_PinModeSet(const GpioPin_t Pin, const GpioMode_t Mode){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    assert_param(IsValidMode(Mode.PinMode));
    assert_param(IsValidAltMode(Mode.Alternate));
    uint32_t GoldenReg = Mode.PinMode << Pin.Number*2;
    //Set the Mode Register
    MODIFY_REG(GpioLut[Pin.Port]->MODER, GPIO_MODER_MODER0_Msk << Pin.Number*2, Mode.PinMode << Pin.Number*2);
    if(READ_BIT(GpioLut[Pin.Port]->MODER, GPIO_MODER_MODER0_Msk << Pin.Number*2) != GoldenReg){
        return GpioNOK;
    }
    GoldenReg = Mode.Alternate << (Pin.Number % 8)*4;
    //Set the Alt Fn Register
    MODIFY_REG(GpioLut[Pin.Port]->AFR[Pin.Number / 8], GPIO_AFRL_AFSEL0_Msk << (Pin.Number % 8)*4, Mode.Alternate << (Pin.Number % 8)*4);
    if(READ_BIT(GpioLut[Pin.Port]->AFR[Pin.Number / 8], GPIO_AFRL_AFSEL0_Msk << (Pin.Number % 8)*4) != GoldenReg){
        return GpioNOK;
    }
    return GpioOK;
};

/*********************************************************/
/*Function to set pins pull resistor direction           */
/*parameters : GpioPin and desired direction(or float)   */
/*return     : GpioOK if PUPDR gets desired value        */
/*             GpioNOK if PUPDR did not get correct value*/
/*********************************************************/
GpioStatus_t Gpio_PullDirectionSet(const GpioPin_t Pin, const PullDir_t Dir){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    assert_param(IsValidPull(Dir));
    //Create a Golden Register to compare the modified register to
    uint32_t GoldenReg = Dir << Pin.Number*2;
    //Clear and set Pull Dir Reg
    MODIFY_REG(GpioLut[Pin.Port]->PUPDR, GPIO_PUPDR_PUPD0_Msk << Pin.Number*2, Dir << Pin.Number*2);
    //Validate PUPDR and Golden Reg Match
    if(READ_BIT(GpioLut[Pin.Port]->PUPDR, GPIO_PUPDR_PUPD0_Msk << Pin.Number*2) != GoldenReg){
        return GpioNOK;
    }
    return GpioOK;
};

/***********************************************************/
/*Function to set GPIO Pin Speed                           */
/*parameters : GpioPin and desired speed                   */
/*return     : GpioOK if OSPEEDR gets desired value        */
/*             GpioNOK if OSPEEDR did not get correct value*/
/***********************************************************/
GpioStatus_t Gpio_PinSpeedSet(const GpioPin_t Pin, const PinSpeed_t Speed){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    assert_param(IsValidSpeed(Speed));
    //Create a Golden Register to compare the modified register to
    uint32_t GoldenReg = Speed << Pin.Number*2;
    //Clear and set Speed Reg
    MODIFY_REG(GpioLut[Pin.Port]->OSPEEDR, GPIO_OSPEEDR_OSPEED0_Msk << Pin.Number*2, Speed << Pin.Number*2);
    //Validate OSPEEDR and Golden Reg Match
    if(READ_BIT(GpioLut[Pin.Port]->OSPEEDR, GPIO_OSPEEDR_OSPEED0_Msk << Pin.Number*2) != GoldenReg){
        return GpioNOK;
    }
    return GpioOK;
};

/***********************************************************/
/*Function set GPIO Output Type                            */
/*parameters : GpioPin and desired output type             */
/*return     : GpioOK if OTYPER gets desired value         */
/*             GpioNOK if OTYPER did not get correct value */
/***********************************************************/
GpioStatus_t Gpio_OutputTypeSet(const GpioPin_t Pin, const OutputType_t OutputType){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    assert_param(IsValidOutputType(OutputType));
    //Create a Golden Register to compare the modified register to
    uint32_t GoldenReg = OutputType << Pin.Number;
    //Clear and set Speed Reg
    MODIFY_REG(GpioLut[Pin.Port]->OTYPER, GPIO_OTYPER_OT0_Msk << Pin.Number, OutputType << Pin.Number);
    //Validate OTYPER and Golden Reg Match
    if(READ_BIT(GpioLut[Pin.Port]->OTYPER, GPIO_OTYPER_OT0_Msk << Pin.Number) != GoldenReg){
        return GpioNOK;
    }
    return GpioOK;
};

/*********************************************************************/
/*Function to configure IRQ for a gpio pin                           */
/*parameters : GpioPin and desired IRQ Trigger                       */
/*return     : GpioOK If EXTI RTSR, FTSR, and EXTICR modified        */
/*             GpioNOK If EXTI RTSR, FTSR, and EXTICR not modified   */
/*Note: Function will NOT Disable IRQs at the NVIC, only enable them.*/
/*User must call NVIC_DisableIRQ() to disable IRQ at NVIC            */
/*It WILL Mask the IRQ at the EXTI                                   */
/*********************************************************************/
GpioStatus_t Gpio_IrqConfig(const GpioPin_t Pin, const GpioInterruptMode_t IrqMode){
    assert_param(IsValidPort(Pin.Port));
    assert_param(IsValidPinNumber(Pin.Number));
    assert_param(IsValidIrqMode(IrqMode));
    //Clear Rising and Falling Trigger Bits
    CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR0_Msk << Pin.Number);
    CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_TR0_Msk << Pin.Number);
    //Clear EXTI Mux Bits
    CLEAR_BIT(SYSCFG->EXTICR[Pin.Number / 4], SYSCFG_EXTICR1_EXTI0_Msk << (Pin.Number % 4)*4);
    
    //Set Falling Edge Bit
    uint32_t GoldenReg = EXTI_FTSR_TR0_Msk << Pin.Number;
    if(IrqMode == FallingEdge || IrqMode == EitherEdge){
        SET_BIT(EXTI->FTSR, EXTI_FTSR_TR0_Msk << Pin.Number);
        if(READ_BIT(EXTI->FTSR,EXTI_FTSR_TR0_Msk << Pin.Number) != GoldenReg){
            return GpioNOK;
        }
    }
    //Set Rising Edge Bit
    GoldenReg = EXTI_RTSR_TR0_Msk << Pin.Number;
    if(IrqMode == RisingEdge || IrqMode == EitherEdge){
        SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0_Msk << Pin.Number);
        if(READ_BIT(EXTI->RTSR,EXTI_RTSR_TR0_Msk << Pin.Number) != GoldenReg){
            return GpioNOK;
        }
    }
    
    //Config the EXTI Mux
    GoldenReg = Pin.Port << (Pin.Number % 4)*4;
    if(IrqMode != IrqOff){
        //Enable Syscfg Clk
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN_Msk);

        SET_BIT(SYSCFG->EXTICR[Pin.Number / 4], Pin.Port << (Pin.Number % 4)*4);
        if(READ_BIT(SYSCFG->EXTICR[Pin.Number / 4], SYSCFG_EXTICR1_EXTI0_Msk << (Pin.Number % 4)*4) != GoldenReg){
            return GpioNOK;
        }
        //Unmask IRQ
        SET_BIT(EXTI->IMR, 1 << Pin.Number);
        NVIC_EnableIRQ(IrqNumbersLut[Pin.Number]);
    } else {
        //Mask IRQ
        CLEAR_BIT(EXTI->IMR, 1 << Pin.Number);
    }
    return GpioOK;
};

/********************************************************************/
/*Function to assign a callback function to callback function table */
/*parameters : GpioPin, pointer to callback function                */
/*return     : GpioOK always (failure)                              */
/*             Param check: To be implemented later                 */
/********************************************************************/
GpioStatus_t Gpio_CallbackRegister(const GpioPin_t Pin, const GpioCallback_t Function){
    assert_param(IsValidPinNumber(Pin.Number));
    assert_param(IsValidPort(Pin.Port));
    GpioCallback[Pin.Number] = Function;
    return GpioOK;
}