#include "gpio.h"
#include "stm32f4xx.h"

int main(){
    Gpio_Init(GetGpioConfig());
    while(1);
}