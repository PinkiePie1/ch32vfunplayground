#include "ch32fun.h"
#include <stdbool.h>
#include <stdio.h>


/**
 * @brief 初始化GPIO
 *
 * PA1由timer1ch2直接控制
 * PA2由DMA控制
 * 
 */
static void GPIO_init( void )
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA; //开启GPIOA时钟

    //PA2设置为推挽输出
    GPIOA->CFGLR &= ~(0xf<<(4*2));
	GPIOA->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP)<<(4*2);

    //PA1设置为复用型推挽输出。
    GPIOA->CFGLR &= ~(0xf<<(4*1));
	GPIOA->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*1); 
    
    
//	funGpioInitAll();
//	funPinMode( PA1, GPIO_CFGLR_OUT_50Mhz_AF_PP ); 
//	funPinMode( PA2, GPIO_CFGLR_OUT_50Mhz_PP );
}

static void PWM_init( void )
{
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1; //开启TIM1时钟

    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1; //重置timer1
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    TIM1->CCER |= TIM_CC2E; //使能ch2输出
    TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1; //110：PWM 模式1，向上计数时会先拉高，达到比较值后拉低。

    TIM1->CTLR1 = TIM_ARPE; //使能自动重装
	TIM1->PSC = 0x0002 - 1; // 2分频
	TIM1->ATRLR = 0xFFFF;//重装值65536

    TIM1->CH2CVR = 0xFFFF>>1;//暂时设置到一半的占空比
	TIM1->SWEVGR |= TIM_UG;

	//开启输出。由于OSSI和OSSR为0，启动定时器之前引脚不会输出。这是我们想要的，在TIMER开启之前不会启动H桥
	TIM1->BDTR |= TIM_MOE;

    TIM1->CTLR1 |= TIM_CEN;//启动定时器。

}

static void PWM_Setduty(uint32_t duty)
{
    TIM1->CH2CVR = duty;//重设占空比
	TIM1->SWEVGR |= TIM_UG;//触发更新事件，立刻更新。
}

int main( void )
{   
    SystemInit(); //初始化时钟。ch32fun的sdk默认跑在48mhz且外设不分频。
    GPIO_init();
    PWM_init();
    while(1)
    {
        GPIOA->BSHR |= 1<<1 | 1<<2;
        PWM_Setduty(0xFFFF>>3);
        Delay_Ms( 2500 );
        PWM_Setduty(0xFFFF>>1);
        GPIOA->BCR  |= 1<<1 | 1<<2;
        Delay_Ms( 2500 );


    }
}