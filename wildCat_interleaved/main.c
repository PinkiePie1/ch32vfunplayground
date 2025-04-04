#include "ch32fun.h"
#include <stdbool.h>
#include <stdio.h>

/*参数定义*/

/*全局变量*/
uint16_t adcval = 0;
uint32_t vccval = 0;
uint32_t adcinterrupttimes = 0;
int32_t controloutput=0;

/*
 *  总的思路是第一个timer一个通道直出，另外两个通道控制DMA。
 *  第二个timer由第一个timer的TRGO启动，预输入CNT的值。
 *  不论如何都很难绕开DMA。dma最小要六个时钟周期，所以无法实现0占空比输出。
 *  这一点上使用其他的单片机，比如cw32l031或许会更好。或者干脆用通用系列
 *  定时器多，同时启动就好。
 */


//声明中断处理函数
//由于函数放在了RAM里加快运行速度，编译器会报警
//暂时不需要为此担心。但后续应在串口里堵死
//可能的不安全因素，比如采用静态内存分配并限制串口信息长度
void ADC1_IRQHandler(void)
	__attribute__((interrupt))
	__attribute__((section(".srodata")));



/**
 * @brief 初始化GPIO
 *
 * PA1和PA2还有PD7三个脚分别为三个flyback的控制脚
 * PA2由DMA控制，就用T1CH4和T1CH3吧
 * PA1由T1CH2。
 * PD7由timer2CH4直接控制
 *
 * 注意PD7也是reset脚，所以要在下载软件里把PD7设置为普通IO
 * minichlink或者官方的wchlinkutility都可以做到。
 *
 * PC4为升压电路监测脚，是ADC的channel2.
 */
static void GPIO_init( void )
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA |
                      RCC_APB2Periph_GPIOD |
                      RCC_APB2Periph_GPIOC; //开启GPIOA，C和D时钟

    //PA2设置为推挽输出
    GPIOA->CFGLR &= ~(0xf<<(4*2));
    GPIOA->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP)<<(4*2);

    //PA1设置为复用型推挽输出。
    GPIOA->CFGLR &= ~(0xf<<(4*1));
    GPIOA->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*1);

    //PD7设置为复用性推挽输出。
    GPIOD->CFGLR &= ~(0xf<<(4*7));
    GPIOD->CFGLR |= (GPIO_Speed_30MHz | GPIO_CNF_OUT_PP_AF)<<(4*7);

    //PC4设置为模拟输入。
    GPIOC->CFGLR &= ~(0xf<<(4*4));
}

/*初始化ADC。
 * 试过很多种方式，但注入通道确实得和普通通道一起
 * 开启才能有效监控电源电压
 * 希望在频率降到原来三分之一之后可以变得宽松些。
 */
static void ADC_init( void )
{

    RCC->APB2PCENR |= RCC_APB2Periph_ADC1;//开启ADC时钟并初始化所有寄存器
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

    //设置ADC时钟分频
    RCC->CFGR0 &= ~RCC_ADCPRE;
    RCC->CFGR0 |= RCC_ADCPRE_DIV2;	// 如果5V供电能开到24Mhz

    ADC1->RSQR1 = 0;
    ADC1->RSQR3 = 2UL;//只开一个通道，转换pc4，也就是A2。

    ADC1->ISQR = 8UL<<15;//注入通道转换通道8，即Vref。

    ADC1->SAMPTR2=3<<(3*2)|3<<(3*8);
    //设置采样源的采样时间，并非规则组内通道的时间。

    ADC1->CTLR1 |= ADC_JEOCIE | ADC_SCAN | ADC_JAUTO;//开启转换结束中断

    ADC1->CTLR2 |= ADC_EXTTRIG |ADC_EXTSEL_0|ADC_EXTSEL_1;
    //开启TRGO触发转换功能，开启注入通道软件使能

    //开启ADC
    ADC1->CTLR2 |= ADC_ADON;

    //校准ADC
    ADC1->CTLR2 |= ADC_RSTCAL;
    while(ADC1->CTLR2 & ADC_RSTCAL);
    ADC1->CTLR2 |= ADC_CAL;
    while(ADC1->CTLR2 & ADC_CAL);
}

/*
 * 初始化DMA
 * DMA用于产生有一定相移的pwm信号。注意搬运过程有延时，为了防止故障
 * 考虑额外加一个channel将GPIO在结束时拉低。
 *
 * T1CH4 -> channel 4 -> PA2 BCR
 * T1CH3 -> channel 6 -> PA2 BSHR
 */
static void DMA_init( void )
{
    static uint32_t gpio2 = 1<<2;

    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;//开启DMA时钟

    //TIM1-CH4和T1CH3分别在DMA4和6通道的，CH3拉高，CH4拉低好了。
    //配置channel2，32位到32位，从存储器搬到外设，单次操作，无中断，
    //最高优先级，开启循环模式。要开循环模式，不开的话搬一次之后数
    //据数量和地址都清零了就没法继续触发了。
    DMA1_Channel4->CFGR |= DMA_CFGR2_EN |
    DMA_Priority_VeryHigh | DMA_MemoryDataSize_Word |
    DMA_PeripheralDataSize_Word | DMA_DIR_PeripheralDST | DMA_Mode_Circular;

    DMA1_Channel4->CNTR = 0x0001;//数据传输数量，就一个。
    DMA1_Channel4->PADDR = (uint32_t)&(GPIOA->BCR);
    //存储器和外设的地址，外设是GPIO的清零寄存器。
    DMA1_Channel4->MADDR = (uint32_t)&gpio2;

    //TIM1-CH3是连接到DMA6通道的，产生DMA请求时通过DMA将PA2拉高。
    DMA1_Channel6->CFGR |= DMA_CFGR2_EN | DMA_Priority_VeryHigh |
    DMA_MemoryDataSize_Word | DMA_PeripheralDataSize_Word |
    DMA_DIR_PeripheralDST | DMA_Mode_Circular;

    DMA1_Channel6->CNTR = 0x0001;//数据传输数量，就一个。
    DMA1_Channel6->PADDR = (uint32_t)&(GPIOA->BCR);
    //存储器和外设的地址，外设是GPIO的置位寄存器。
    //但初始化的时候不应该输出，所以先设置为清零寄存器，这样引脚只会拉低
    //后续在Set_duty函数中这里会在适当的时候改为可以正常输出的。
    //由于进行了搬运，会使得GPIOA其他所有GPIO都不能再被作为通用IO利用。
    DMA1_Channel6->MADDR = (uint32_t)&gpio2;

}

/* 设置占空比。
 * 如果占空比小于6的话就直接关掉第三个输出。
 * 没有限制上限，需要在程序里限制。
 */

static void Set_duty(uint32_t duty)
{
    //如果Duty小于6，则DMA不会拉高PA2，否则会。
    DMA1_Channel6->PADDR = (duty>=6) ? (uint32_t)&(GPIOA->BSHR) : (uint32_t)&(GPIOA->BCR);
    TIM2->CH4CVR = duty;
    TIM1->CH2CVR = duty;
    TIM1->CH4CVR = 190+duty;

}

/* 初始化timer2
 *
 */
static void TIM2_init( void )
{
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2; //开启tim2时钟

    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2; //重置timer2
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    TIM2->CTLR1 |= TIM_ARPE;//开启预装载
    TIM2->CCER |= TIM_CC4E;//使能channel4输出
    TIM2->CHCTLR2 |= ( TIM_OC4M_2 | TIM_OC4M_1 );
    //PWM模式1。
    TIM2->SMCFGR = TIM_SMS_2|TIM_SMS_1;
    //由TRGI，也就是TIM1的触发信号启动定时器
    //TS = 000，输入触发连接到ITR0

    TIM2->CTLR2 |= TIM_MMS_1;
    //更新事件被输出到TRGO，用于触发ADC。这意味着每次计时器溢出就会触发ADC。
    //数据手册中的描述可能有误，产生的是输出而不是输入。

    TIM2->PSC = 0x0000;//不分频，48Mhz为tim2时钟
    TIM2->CNT = 129UL;//由于启动存在延时，这里的初始值比较魔法。
    TIM2->ATRLR = (uint32_t)288;//周期288，也是166khz。

    TIM2->CH4CVR = (uint32_t)0;//开始占空比为0

    TIM2->BDTR |= TIM_MOE;


}

/* 初始化timer1
 * CH4和CH3发DMA
 * CH2直接控制引脚
 * CH1给TIM2发TRGO信号。
 *
 */
static void TIM1_init( void )
{
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1; //开启TIM1时钟

    RCC->APB2PRSTR |= RCC_APB2Periph_TIM1; //重置timer1
    RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

    TIM1->CTLR1 |= TIM_ARPE;
    TIM1->CTLR2 |= TIM_MMS_0;
    //启动事件触发输出（TRGO）
    TIM1->DMAINTENR |= TIM_TDE | TIM_CC4DE | TIM_CC3DE;//开启CH4和CH3的DMA

    TIM1->PSC = 0x0001 - 1;//不分频，跑在48Mhz
    TIM1->ATRLR = 288UL;
    //288就是跑在166khz，交错三倍频对于flyback就是500khz。

    TIM1->CHCTLR1 |=  ( TIM_OC2M_2 | TIM_OC2M_1 ) | TIM_OC2PE;
    TIM1->CHCTLR2 |=  ( TIM_OC3M_2 | TIM_OC3M_1 ) |
                      ( TIM_OC4M_2 | TIM_OC4M_1 ) |
                        TIM_OC3PE | TIM_OC4PE;
    //均为PWM模式1，在达到计数值前为有效电平,均开启预装载，
    //配置为输出。

    TIM1->CCER |= TIM_CC2E;//开启通道2输出到引脚

    TIM1->CH2CVR = 0UL;    //这里是直出
    TIM1->CH3CVR = 190UL; //CH3是拉高，起点在三分之二，相当于240相移
    TIM1->CH4CVR = 190+1UL; //CH4是拉低

    TIM1->BDTR = TIM_MOE;//使能输出。

    //这里还不能启动TIM1，TIM2配置完成后再启动。
}

/**
 * @brief 软启动升压电路。在初始化 ADC 和 HV 驱动器之后需要使用这个函数启动升压电路。
 *
 * 该函数固定占空比等待输出电压逐渐上升到足够的水平。
 * 启动过程中会禁用 ADC 中断，不是PID控制。
 *
 * @note 如果输出没接东西，程序会卡死在这里。
 *
 */
static void Soft_Start()
{
    //软启动过程，以低占空比启动直到输出电压足够。
    NVIC_DisableIRQ( ADC_IRQn );
    TIM2->CNT = 129UL;
    TIM1->CNT = 0UL;


	
	Set_duty(3);
    TIM1->CTLR1 |= TIM_CEN; //开启PWM输出
	Delay_Us(1500);
    while(ADC1->RDATAR < 30);
    Set_duty(8);
    while(ADC1->RDATAR < 60);
    Set_duty(10);
    
    while(ADC1->RDATAR < 90);
    NVIC_EnableIRQ( ADC_IRQn );
}

/**
 *
 * @brief 停止输出。
 *
 *
 * @note PA2是DMA控制的，光是关掉计数器还不够。
 */

static void Cease_Output()
{
    TIM2->CTLR1 &= ~TIM_CEN;
    TIM1->CTLR1 &= ~TIM_CEN;
    TIM2->CH4CVR = 0;
    GPIOA->BCR = 1<<2;//PA2需要手动关掉以防万一。
    NVIC_DisableIRQ( ADC_IRQn );

}


int main( void )
{
    SystemInit();
    //初始化系统时钟。
    //ch32fun的sdk默认跑在HSI-PLL-48mhz,外设不分频。
    GPIO_init(); //初始化外设
    ADC_init(); //初始化ADC
    DMA_init(); //初始化DMA操作器，用于实现相移输出。
    TIM2_init();
    TIM1_init(); //初始化升压电路驱动
    //printf("Init complete. Soft start.\n");

   Soft_Start();
    //TIM1->CTLR1 |= TIM_CEN; //开启PWM输出
     //NVIC_EnableIRQ( ADC_IRQn );
    for(;;)
    {    	
        Delay_Ms(2000);
        printf("alive. PWM output at:%ld, ADC measurement:%ld, idata: %ld\n",TIM1->CH2CVR,ADC1->RDATAR,ADC1->IDATAR1);
	//	printf("control output:%ld.\n",controloutput);
    }

}
/*
//快速乘法，比默认的*稍微快一点。
static inline uint32_t fastMul(uint32_t bigval, uint32_t smallval)
{
	uint32_t ret = 0;
	uint32_t multiplicand = smallval;
	uint32_t mutliplicant = bigval;
	do
	{
		if( multiplicand & 1 )
			ret += mutliplicant;
		mutliplicant<<=1;
		multiplicand>>=1;
	} while( multiplicand );
	return ret;
}
*/

//ADC的中断处理函数，也是PID环路所在。
void ADC1_IRQHandler( void )
{
    ADC1->STATR = 0;//清空标志位
    adcval =  ( adcval >> 1 ) + ((ADC1->RDATAR) >> 1);
    vccval =  ( vccval >> 1 ) + ((ADC1->IDATAR1)>> 1);
    //读取ADC值，这里是一半旧值加一半新值，滤波

    //这里才是设定值
    //因为是乘法所以很慢
    //后续考虑换ch32v002或者其他带硬件乘法的
    int32_t setpoint_vdd = vccval + ( vccval>>2 ) + ( vccval>>3 ) + ( vccval>>6 );
    //如果需要任意设定值，这里应该是 (设定值*vccval/242)
    //一种近似是(360*vccval)>>8。
    //如果是5V稳定，vccval=242，这里360被处理过后会变成344，167.9V
    //如果是3v3稳定，vccval=372，则会变成523，对应168.75v
    //但如果设定值不需要动态变化，我们就能把乘法部分去掉。
    //目前，这里设定值是vccval*1.4，即目标电压为1.2*1.4*100=168V。
    //另一方面，我们也可以直接把vccval的不同移位加一起，一定程度上改变输出电压
    //即使这样的改变不是线性的，也可以由串口的另一端来负担。
    
    /*PI控制器部分。*/

    int32_t error = setpoint_vdd-adcval;//计算误差
    int32_t CO = 0;//控制输出。
    static int32_t intergal;//积分项
    static int32_t preverror;//上一次的误差

    intergal += ( error>>2 ); //积分系数为0.25

    //控制积分上下限
    intergal = intergal > 90 ? 90 : intergal;
    intergal = intergal < -20 ? -20 :intergal;

    CO = ( error>>2 ) + intergal + ( (error-preverror)<<1 );
    //比例系数为1,微分系数为2

    //限制CO上下限。
    CO = CO >= 80 ? 80 : CO;
    CO = CO <= 0 ? 0 : CO;

    Set_duty(CO);
    controloutput =CO;

    preverror = error;

}
