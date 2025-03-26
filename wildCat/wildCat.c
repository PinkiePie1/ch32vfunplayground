#include "ch32fun.h"
#include <stdbool.h>
#include <stdio.h>

/*参数定义*/
#define PWM_PERIOD 0xEA3C//pwm周期，计算公式为PWM_PERIOD=183.105/所需pwm频率*0xFFFF。

/*全局变量*/
uint16_t adcval = 0;
uint32_t adcinterrupttimes = 0;


void ADC1_IRQHandler(void)
	__attribute__((interrupt))
	__attribute__((section(".srodata")));


/**
 * @brief 初始化GPIO
 * 
 * PA1和PA2为H桥的控制脚
 * PA1由timer1ch2直接控制
 * PA2由DMA控制。
 * PD2由DMA控制。
 * 
 * PD4为升压电路的控制脚，由TIM2
 * 控制。
 * 
 * PC4为升压电路监测脚，是ADC的channel2.
 */
static void GPIO_init( void )
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |  RCC_APB2Periph_GPIOC; //开启GPIOA，C和D时钟

    //PA2设置为推挽输出
    GPIOA->CFGLR &= ~(0xf<<(4*2));
	GPIOA->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP)<<(4*2);
    
    //PD2设置为推挽输出
    GPIOD->CFGLR &= ~(0xf<<(4*2));
	GPIOD->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP)<<(4*2);

    //PA1设置为复用型推挽输出。
    GPIOA->CFGLR &= ~(0xf<<(4*1));
	GPIOA->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*1); 

    //PD4设置为复用性推挽输出，是boost电路的控制信号
    GPIOD->CFGLR &= ~(0xf<<(4*4));
	GPIOD->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*4); 

    //PC4设置为模拟输入。
    GPIOC->CFGLR &= ~(0xf<<(4*4));
}

//初始化ADC。
static void ADC_init( void )
{   
    RCC->APB2PCENR |= RCC_APB2Periph_ADC1;//开启ADC时钟并初始化所有寄存器
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

    //设置ADC时钟分频
	RCC->CFGR0 &= ~RCC_ADCPRE;  
	RCC->CFGR0 |= RCC_ADCPRE_DIV4;	// 4分频，12Mhz。如果5V供电能开到24Mhz，但这里就先不这么做。
    
    ADC1->RSQR1 = 0;
	ADC1->RSQR3 = 2;//只开一个通道，转换pc4，也就是A2。后续可考虑加上DMA并测vref。vref为1.2V内置基准，测它可得VCC电压。

    ADC1->SAMPTR2=1<<(3*2);//设置采样源的采样时间，注意这里是设置采样源（0-9）的时间，并非规则组内通道的时间。

    ADC1->CTLR1 |= ADC_EOCIE | ADC_SCAN;//开启转换结束中断。

    ADC1->CTLR2 |= ADC_EXTTRIG |ADC_EXTSEL_0|ADC_EXTSEL_1;//开启TRGO触发转换功能。

    ADC1->CTLR2 |= ADC_ADON;
    
    //校准ADC
    ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);

	//开启中断。
	NVIC_EnableIRQ( ADC_IRQn );

}

//初始化DMA
static void DMA_init( void )
{
    static uint32_t gpio2 = 1<<2;

    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;//开启DMA时钟
    
    //TIM1-CH1是连接到DMA2通道的，CH1产生DMA请求则通过DMA将PA1拉高。
    //配置channel2，32位到32位，从存储器搬到外设，单次操作，无中断，最高优先级，开启循环模式。要开循环模式，不开的话搬一次之后数据数量和地址都清零了就没法继续触发了。
    DMA1_Channel2->CFGR |= DMA_CFGR2_EN | DMA_Priority_VeryHigh | DMA_MemoryDataSize_Word | DMA_PeripheralDataSize_Word | DMA_DIR_PeripheralDST | DMA_Mode_Circular;
    DMA1_Channel2->CNTR = 0x0001;//数据传输数量，就一个。
    DMA1_Channel2->PADDR = (uint32_t)&(GPIOD->BSHR); //存储器和外设的地址，外设是GPIOA的置位寄存器。由于进行了搬运，会使得GPIOA其他所有GPIO都不能再当通用IO口，但ch32v003本来就只有PA1和PA2，所以无所谓。
    DMA1_Channel2->MADDR = (uint32_t)&gpio2;

    //TIM1-CH3是连接到DMA6通道的，产生DMA请求时通过DMA将PA1拉低。
    DMA1_Channel6->CFGR |= DMA_CFGR2_EN | DMA_Priority_VeryHigh | DMA_MemoryDataSize_Word | DMA_PeripheralDataSize_Word|DMA_DIR_PeripheralDST|DMA_Mode_Circular;
    DMA1_Channel6->CNTR = 0x0001;//数据传输数量，就一个。
    DMA1_Channel6->PADDR = (uint32_t)&(GPIOD->BCR); //存储器和外设的地址，外设是GPIOA的清零寄存器。由于进行了搬运，会使得GPIOA其他所有GPIO都不能再被利用。
    DMA1_Channel6->MADDR = (uint32_t)&gpio2;

}

//初始化高压驱动需要的时钟
static void HVDriver_init( void )
{
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2; //开启tim2时钟

    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2; //重置timer2
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    TIM2->CCER |= TIM_CC1E;//使能channel1输出
    TIM2->CHCTLR1 |= ( TIM_OC1M_2 | TIM_OC1M_1 );//PWM模式1，对比较值的更改立刻生效。
    TIM2->CTLR2 |= TIM_MMS_1;//更新事件被输出到TRGO，用于触发ADC。这意味着每次计时器溢出就会触发ADC。数据手册中的描述可能有误，不是输入，是输出。

    TIM2->PSC = 0x0000;//不分频，48Mhz为tim2时钟
    TIM2->ATRLR = (uint32_t)160;//周期为160次计数，因此为300khz输出。

    TIM2->CH1CVR = (uint32_t)0;//开始占空比为0

    TIM2->BDTR |= TIM_MOE;
    TIM2->CTLR1 |= TIM_CEN;//启动定时器。
}

//初始化PWM时钟
static void PWM_init( void )
{
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1; //开启TIM1时钟

    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1; //重置timer1
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    TIM1->CCER |= TIM_CC2E; //使能ch2输出
    TIM1->CHCTLR1 |= ( TIM_OC2M_2 | TIM_OC2M_1 | TIM_ARPE | TIM_OC2PE | TIM_OC3PE); //110：PWM 模式1，向上计数时会先拉高，达到比较值后拉低,开启预装载寄存器，对占空比的更新在下一个周期才会生效。
    TIM1->DMAINTENR |= TIM_TDE|TIM_CC1DE|TIM_CC3DE;//允许channel1和channel3产生DMA请求，请求会直接接入dma那边。

    //这两行定义了PWM的频率，计算公式为48Mhz/分频系数/重装值
	TIM1->PSC = 0x0004 - 1; // 4分频
	TIM1->ATRLR = PWM_PERIOD;//重装值

    TIM1->CH2CVR = 0x0001;//暂时设置到无占空比
    TIM1->CH1CVR = PWM_PERIOD>>1; //Channel1的比较值，达到该值时PA1拉高
    TIM1->CH3CVR = PWM_PERIOD>>1;//Channel3的比较值，达到该值时PA1拉低。由于ch3所在的DMA通道优先级较低，如果两者同时触发，DMA会先拉高再拉低，符合预期。
	//TIM1->SWEVGR |= TIM_UG;

	//开启输出。由于OSSI和OSSR为0，启动定时器之前引脚不会输出。这是我们想要的，在TIMER开启之前不会启动H桥
	TIM1->BDTR |= TIM_MOE;

    TIM1->CTLR1 |= TIM_CEN;//启动定时器。
}

//设置占空比，最小为0，最大为PWM_PERIOD/2，再大会导致H桥贯通。
static void PWM_Setduty(uint32_t duty)
{   
    duty = duty > PWM_PERIOD>>1 ? (PWM_PERIOD>>1) : duty ;//限制duty的最大值。
    //重设占空比
    TIM1->CH2CVR = duty+1;
    TIM1->CH3CVR = (PWM_PERIOD>>1)+duty;
}

int main( void )
{   
    SystemInit(); //初始化系统时钟。ch32fun的sdk默认跑在HSI-PLL-48mhz,外设不分频。
    GPIO_init();
    ADC_init();
    DMA_init();
    PWM_init();
    HVDriver_init();
    for(;;)
    {   
        PWM_Setduty(PWM_PERIOD>>2);
        Delay_Ms(2550);
        PWM_Setduty(PWM_PERIOD>>4);
        Delay_Ms(2550);
    }
}

//ADC的中断处理函数


void ADC1_IRQHandler(void)
{
    ADC1->STATR = 0;//清空标志位
    adcval = ADC1->RDATAR;//读取ADC值


    /*PI控制器部分。*/

    int32_t error = 30-adcval;//计算误差
    int32_t CO = 0;//控制输出。
    static int32_t intergal;//积分项

    intergal += (error>>2); //积分系数为0.25

    //控制积分上下限
    intergal = intergal > 160 ? 160 : intergal;
    intergal = intergal < -20 ? -20 :intergal;

    CO = (error<<2) + intergal;//比例系数为4

    //限制CO上下限。
    CO = CO >= 128 ? 128 : CO;
    CO = CO <= 0 ? 0 : CO; 

    TIM2->CH1CVR = CO;

}
