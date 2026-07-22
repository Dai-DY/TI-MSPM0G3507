/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_2
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM15)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM15_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_3
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM16)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM16_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIM_delay_ms */
#define TIM_delay_ms_INST                                                (TIMA0)
#define TIM_delay_ms_INST_IRQHandler                            TIMA0_IRQHandler
#define TIM_delay_ms_INST_INT_IRQN                              (TIMA0_INT_IRQn)
#define TIM_delay_ms_INST_LOAD_VALUE                                      (999U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                400000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_DEBUG */
#define UART_DEBUG_INST                                                    UART0
#define UART_DEBUG_INST_FREQUENCY                                       32000000
#define UART_DEBUG_INST_IRQHandler                              UART0_IRQHandler
#define UART_DEBUG_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_UART_DEBUG_RX_PORT                                            GPIOA
#define GPIO_UART_DEBUG_TX_PORT                                            GPIOA
#define GPIO_UART_DEBUG_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_UART_DEBUG_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_UART_DEBUG_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_UART_DEBUG_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_UART_DEBUG_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_DEBUG_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define UART_DEBUG_BAUD_RATE                                            (115200)
#define UART_DEBUG_IBRD_32_MHZ_115200_BAUD                                  (17)
#define UART_DEBUG_FBRD_32_MHZ_115200_BAUD                                  (23)





/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for B21: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY_B21_PIN                                             (DL_GPIO_PIN_21)
#define KEY_B21_IOMUX                                            (IOMUX_PINCM49)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for B22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_B22_PIN                                             (DL_GPIO_PIN_22)
#define LED_B22_IOMUX                                            (IOMUX_PINCM50)
/* Port definition for Pin Group CAR_KEY */
#define CAR_KEY_PORT                                                     (GPIOA)

/* Defines for PIN_KEY: GPIOA.18 with pinCMx 40 on package pin 11 */
#define CAR_KEY_PIN_KEY_PIN                                     (DL_GPIO_PIN_18)
#define CAR_KEY_PIN_KEY_IOMUX                                    (IOMUX_PINCM40)
/* Port definition for Pin Group CAR_LED */
#define CAR_LED_PORT                                                     (GPIOB)

/* Defines for PIN_LED: GPIOB.9 with pinCMx 26 on package pin 61 */
#define CAR_LED_PIN_LED_PIN                                      (DL_GPIO_PIN_9)
#define CAR_LED_PIN_LED_IOMUX                                    (IOMUX_PINCM26)
/* Port definition for Pin Group MOTOR_STBY */
#define MOTOR_STBY_PORT                                                  (GPIOB)

/* Defines for STBY: GPIOB.14 with pinCMx 31 on package pin 2 */
#define MOTOR_STBY_STBY_PIN                                     (DL_GPIO_PIN_14)
#define MOTOR_STBY_STBY_IOMUX                                    (IOMUX_PINCM31)
/* Port definition for Pin Group MPU6050_INT */
#define MPU6050_INT_PORT                                                 (GPIOA)

/* Defines for PIN: GPIOA.7 with pinCMx 14 on package pin 49 */
// pins affected by this interrupt request:["PIN"]
#define MPU6050_INT_INT_IRQN                                    (GPIOA_INT_IRQn)
#define MPU6050_INT_INT_IIDX                    (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define MPU6050_INT_PIN_IIDX                                 (DL_GPIO_IIDX_DIO7)
#define MPU6050_INT_PIN_PIN                                      (DL_GPIO_PIN_7)
#define MPU6050_INT_PIN_IOMUX                                    (IOMUX_PINCM14)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOA)

/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_16)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM38)
/* Defines for BIN2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define BIN_BIN2_PIN                                            (DL_GPIO_PIN_17)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM39)
/* Port definition for Pin Group AIN */
#define AIN_PORT                                                         (GPIOA)

/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_13)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM35)
/* Defines for AIN2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_14)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM36)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.26 with pinCMx 59 on package pin 30 */
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_26)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM59)
/* Defines for E1B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_25)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM55)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.24 with pinCMx 52 on package pin 23 */
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_24)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM52)
/* Defines for E2B: GPIOB.20 with pinCMx 48 on package pin 19 */
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_20)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM48)
/* Port definition for Pin Group EIGHT_IR */
#define EIGHT_IR_PORT                                                    (GPIOB)

/* Defines for AD0: GPIOB.15 with pinCMx 32 on package pin 3 */
#define EIGHT_IR_AD0_PIN                                        (DL_GPIO_PIN_15)
#define EIGHT_IR_AD0_IOMUX                                       (IOMUX_PINCM32)
/* Defines for AD1: GPIOB.16 with pinCMx 33 on package pin 4 */
#define EIGHT_IR_AD1_PIN                                        (DL_GPIO_PIN_16)
#define EIGHT_IR_AD1_IOMUX                                       (IOMUX_PINCM33)
/* Defines for AD2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define EIGHT_IR_AD2_PIN                                        (DL_GPIO_PIN_17)
#define EIGHT_IR_AD2_IOMUX                                       (IOMUX_PINCM43)
/* Defines for OUT: GPIOB.18 with pinCMx 44 on package pin 15 */
#define EIGHT_IR_OUT_PIN                                        (DL_GPIO_PIN_18)
#define EIGHT_IR_OUT_IOMUX                                       (IOMUX_PINCM44)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIM_delay_ms_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_DEBUG_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
