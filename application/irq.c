#include "stm32f4xx.h"
#include "gpio.h"

extern GpioCallback_t GpioCallback[];

/***************************************************************************/
/*All IRQs call the appropriate callback (see gpio.c) and clear Pending bit*/
/***************************************************************************/
void EXTI0_IRQHandler(){
    GpioCallback[Pin00]();
    SET_BIT(EXTI->PR, 1 << 0);
}

void EXTI1_IRQHandler(){
    GpioCallback[Pin01]();
    SET_BIT(EXTI->PR, 1 << 1);
}

void EXTI2_IRQHandler(){
    GpioCallback[Pin02]();
    SET_BIT(EXTI->PR, 1 << 2);
}

void EXTI3_IRQHandler(){
    GpioCallback[Pin03]();
    SET_BIT(EXTI->PR, 1 << 3);
}

void EXTI4_IRQHandler(){
    GpioCallback[Pin04]();
    SET_BIT(EXTI->PR, 1 << 4);
}

void EXTI9_5_IRQHandler(){
    uint8_t i = 5;
    for(i = 5; i < 10; i++){
        if(READ_BIT(EXTI->PR, 1 << i)){
            GpioCallback[i]();
            SET_BIT(EXTI->PR, 1 << i);
        }
    }
}

void EXTI15_10_IRQHandler(){
    uint8_t i = 10;
    for(i = 10; i < 16; i++){
        if(READ_BIT(EXTI->PR, 1 << i)){
            GpioCallback[i]();
            SET_BIT(EXTI->PR, 1 << i);
        }
    }
}