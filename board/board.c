/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-09     WCH        the first version
 */
 
#include "board.h"
#include <stdint.h>
#include "drv_usart.h"
#include <rthw.h>


extern uint32_t SystemCoreClock;

static uint32_t _SysTick_Config(rt_uint32_t ticks)
{
    NVIC_SetPriority(SysTicK_IRQn,0xf0);
    NVIC_SetPriority(Software_IRQn,0xf0);
    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_EnableIRQ(Software_IRQn);
    SysTick->CTLR=0;
    SysTick->SR=0;
    SysTick->CNT=0;
    SysTick->CMP=ticks-1;
    SysTick->CTLR=0xF;
    return 0;
}

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE (4096)
static uint32_t rt_heap[RT_HEAP_SIZE];
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
    /* System Tick Configuration */
    _SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
	
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
    /* USART driver initialization is open by default */
#ifdef RT_USING_SERIAL
    rt_hw_usart_init();
#endif
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

}

void LED1_BLINK_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    GET_INT_SP();
    /* enter interrupt */
    rt_interrupt_enter();
    SysTick->SR=0;
    rt_tick_increase();
    /* leave interrupt */
    rt_interrupt_leave();
    FREE_INT_SP();

}


//#ifdef BSP_USING_HWTIMER
//#ifdef 1
#if 1
void ch32v3_tim_clock_init(TIM_TypeDef *timx)
{
    if (timx == TIM1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    }

    if (timx == TIM2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    }

    if (timx == TIM3)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    }

    if (timx == TIM4)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    }

    if (timx == TIM5)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    }

    if (timx == TIM6)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    }

    if (timx == TIM7)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    }

    if (timx == TIM8)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    }

    if (timx == TIM9)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    }

    if (timx == TIM10)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    }
}

rt_uint32_t ch32v3_tim_clock_get(TIM_TypeDef *timx)
{
    RCC_ClocksTypeDef RCC_Clocks;
    rt_uint32_t rcc_cfgr0;
    rt_uint8_t ppre1, ppre2;

    RCC_GetClocksFreq(&RCC_Clocks);

    rcc_cfgr0 = RCC->CFGR0;

    ppre1 = (rcc_cfgr0 >> 8) & 0x7;  // CFGR0[10:8]
    ppre2 = (rcc_cfgr0 >> 11) & 0x7; // CFGR0[13:11]

    if (timx == TIM1 || timx == TIM8 || timx == TIM9 || timx == TIM10)
    {
        return ppre2 >= 4 ? RCC_Clocks.PCLK2_Frequency * 2 : RCC_Clocks.PCLK2_Frequency;
    }

    // TIM2~7
    return ppre1 >= 4 ? RCC_Clocks.PCLK1_Frequency * 2 : RCC_Clocks.PCLK1_Frequency;
}

struct rt_hwtimer_info hwtimer_info1 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info2 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info3 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info4 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info5 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info6 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info7 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info8 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info9 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info hwtimer_info10 =
    {
        .maxfreq = 1000000,
        .minfreq = 2000,
        .maxcnt = 0xFFFF,
        .cntmode = HWTIMER_CNTMODE_UP,

};

struct rt_hwtimer_info *ch32v3_hwtimer_info_config_get(TIM_TypeDef *timx)
{
    struct rt_hwtimer_info *info = RT_NULL;

    if (timx == TIM1)
    {
        info = &hwtimer_info1;
    }
    else if (timx == TIM2)
    {
        info = &hwtimer_info2;
    }
    else if (timx == TIM3)
    {
        info = &hwtimer_info3;
    }
    else if (timx == TIM4)
    {
        info = &hwtimer_info4;
    }
    else if (timx == TIM4)
    {
        info = &hwtimer_info4;
    }
    else if (timx == TIM5)
    {
        info = &hwtimer_info5;
    }
    else if (timx == TIM6)
    {
        info = &hwtimer_info6;
    }
    else if (timx == TIM7)
    {
        info = &hwtimer_info7;
    }
    else if (timx == TIM8)
    {
        info = &hwtimer_info8;
    }
    else if (timx == TIM9)
    {
        info = &hwtimer_info9;
    }
    else if (timx == TIM10)
    {
        info = &hwtimer_info10;
    }

    return info;
}

void ch32v3_pwm_io_init(TIM_TypeDef *timx, rt_uint8_t channel)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (timx == TIM1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
    }

    if (timx == TIM2)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
    }

    if (timx == TIM3)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
    }

    if (timx == TIM4)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
    }

    if (timx == TIM5)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
    }

    if (timx == TIM8)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
    }

    if (timx == TIM9)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
    }

    if (timx == TIM10)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

        if (channel == TIM_Channel_1)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_2)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_3)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        if (channel == TIM_Channel_4)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
        }
    }
}
#endif
