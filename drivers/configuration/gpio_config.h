#ifndef GPIO_CONFIG_H_
#define GPIO_CONFIG_H_
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/**************************************************************************/
/*Constant should be changed to the desired number of GpioPins being used */
/**************************************************************************/
#define NUMBER_OF_PINS_USED 3U


#define NUMBER_OF_EXTI_SIGNALS 16U
#define IsValidPort(Port)             (GpioLut[Port] != (NULL))
#define IsValidPinNumber(Number)      (Number >= Pin00 && Number < PinMax)
#define IsValidMode(PinMode)          (PinMode >= Input && PinMode < PinModeMax)
#define IsValidAltMode(AltMode)       (AltMode >= Alternate00 && AltMode < AlternateMax)
#define IsValidPull(Pull)             (Pull >= PinFloat && Pull < PinPullMax)
#define IsValidSpeed(Speed)           (Speed >= SlowSpeed && Speed < SpeedMax)
#define IsValidState(State)           (State == PinLow || State == PinHigh)
#define IsValidOutputType(OutputType) (OutputType == PushPull || OutputType == OpenDrain)
#define IsValidIrqMode(IrqMode)       (IrqMode >= IrqOff && IrqMode < IrqMax)

/******************************************************************************
* Typedefs
*******************************************************************************/

/**********************************************************/
/*Type used for callback function pointers                */
/**********************************************************/
typedef void (*GpioCallback_t)(void);

/**********************************************************/
/*Param to select pin mode                                */
/**********************************************************/
typedef enum{
    Input,
    Output,
    Alternate,
    Analog,
    PinModeMax
}PinMode_t;

/**********************************************************/
/*Param to select an alternate function                   */
/**********************************************************/
typedef enum{
     Alternate00,
     Alternate01,
     Alternate02,
     Alternate03,
     Alternate04,
     Alternate05,
     Alternate06,
     Alternate07,
     Alternate08,
     Alternate09,
     Alternate10,
     Alternate11,
     Alternate12,
     Alternate13,
     Alternate14,
     Alternate15,
     AlternateMax
}GpioAlternateMode_t;

/**********************************************************/
/*Param to select pull resistor type                      */
/**********************************************************/
typedef enum{
    PinFloat,
    PinPullup,
    PinPulldown,
    PinPullMax
}PullDir_t;

/**********************************************************/
/*Param to select pin speed                               */
/**********************************************************/
typedef enum{
    SlowSpeed,
    MediumSpeed,
    HighSpeed,
    VeryHighSpeed,
    SpeedMax
}PinSpeed_t;

/**********************************************************/
/*Param to select gpio output state                       */
/**********************************************************/
typedef enum{
    PinLow,
    PinHigh,
    PinStateMax
}GpioPinState_t;

/**********************************************************/
/*Param to select gpio output type                        */
/**********************************************************/
typedef enum{
    PushPull,
    OpenDrain,
    OutputTypeMax
}OutputType_t;


/**********************************************************/
/*Param to indicate Gpio Pin Number                       */
/**********************************************************/
typedef enum{
     Pin00,
     Pin01,
     Pin02,
     Pin03,
     Pin04,
     Pin05,
     Pin06,
     Pin07,
     Pin08,
     Pin09,
     Pin10,
     Pin11,
     Pin12,
     Pin13,
     Pin14,
     Pin15,
     PinMax
}GpioPinNumber_t;

/**********************************************************/
/*Param to indicate Gpio Port                             */
/**********************************************************/
typedef enum{
    PortA,
    PortB,
    PortC,
    PortD,
    PortE,
    PortF,
    PortG,
    PortH,
    PortI,
    PortMax
}GpioPort_t;

/**********************************************************/
/*Param to select IRQ Trigger                             */
/**********************************************************/
typedef enum{
    IrqOff,
    RisingEdge,
    FallingEdge,
    EitherEdge,
    IrqMax
}GpioInterruptMode_t;

/**********************************************************/
/*Combines a Pin Number and Port Letter, used as an arg   */
/*for must driver functions                               */
/**********************************************************/
typedef struct{
    GpioPinNumber_t Number;
    GpioPort_t Port;
}GpioPin_t;

/**********************************************************/
/*Combines a pin mode and alternate fn number             */
/*used to initialize pin mode                             */
/**********************************************************/
typedef struct{
    PinMode_t PinMode;
    GpioAlternateMode_t Alternate;
}GpioMode_t;

/**********************************************************/
/*Param for GPIO Init function. Contains all info needed  */
/*to initialize a pin                                     */
/**********************************************************/
typedef struct{
    GpioPin_t Pin;
    GpioMode_t Mode;
    PullDir_t Pull;
    PinSpeed_t Speed;
    OutputType_t OutputType;
    GpioPinState_t State;
    GpioInterruptMode_t IrqMode;
}GpioConfig_t;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
const GpioConfig_t *const GetGpioConfig(void);

#endif