/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-09     WCH        the first version
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#include "ch32v30x.h"
#include <rtthread.h>

/* board configuration */
#define SRAM_SIZE  96
#define SRAM_END (0x20000000 + SRAM_SIZE * 1024)

extern int _ebss;
#define HEAP_BEGIN  ((void *)&_ebss)
#define HEAP_END    (SRAM_END-_stack_size)

//extern volatile unsigned long  interrupter_sp_saver;
void rt_hw_board_init(void);
void LED1_BLINK_INIT(void);




#ifdef BSP_USING_TIM
void ch32v3_tim_clock_init(TIM_TypeDef *timx);
rt_uint32_t ch32v3_tim_clock_get(TIM_TypeDef *timx);
#endif

#ifdef BSP_USING_HWTIMER
struct rt_hwtimer_info* ch32v3_hwtimer_info_config_get(TIM_TypeDef *timx);
#endif

#ifdef BSP_USING_PWM
void ch32v3_pwm_io_init(TIM_TypeDef *timx, rt_uint8_t channel);
#endif



#endif /* __BOARD_H__ */
