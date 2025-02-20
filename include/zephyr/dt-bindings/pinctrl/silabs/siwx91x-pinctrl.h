/*
 * Copyright (c) 2024 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INCLUDE_ZEPHYR_DT_BINDINGS_PINCTRL_SILABS_SIWX91X_PINCTRL_H_
#define INCLUDE_ZEPHYR_DT_BINDINGS_PINCTRL_SILABS_SIWX91X_PINCTRL_H_

#include <zephyr/dt-bindings/pinctrl/silabs-pinctrl-siwx91x.h>

/* clang-format off */

#define AGPIO_ULP0           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  0)
#define AGPIO_ULP1           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  1)
#define AGPIO_ULP2           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  2)
#define AGPIO_ULP4           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  4)
#define AGPIO_ULP5           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  5)
#define AGPIO_ULP6           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  6)
#define AGPIO_ULP7           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  7)
#define AGPIO_ULP8           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  8)
#define AGPIO_ULP9           SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0,  9)
#define AGPIO_ULP10          SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0, 10)
#define AGPIO_ULP11          SIWX91X_GPIO(0xFF, 7, 0xFF, 4, 0, 11)

#define AUXULP_TRIG0_PA11    SIWX91X_GPIO(9,     5,    6, 0, 11,  5)
#define AUXULP_TRIG0_PB14    SIWX91X_GPIO(11,    5,    0, 1, 14, 11)
#define AUXULP_TRIG0_PD1     SIWX91X_GPIO(9,     5,   13, 3,  1, 11)
#define AUXULP_TRIG0_ULP5    SIWX91X_GPIO(0xFF,  5, 0xFF, 4,  0,  5)
#define AUXULP_TRIG0_ULP6    SIWX91X_GPIO(0xFF, 10, 0xFF, 4,  0,  6)
#define AUXULP_TRIG0_ULP11   SIWX91X_GPIO(0xFF,  5, 0xFF, 4,  0, 11)
#define AUXULP_TRIG1_ULP4    SIWX91X_GPIO(0xFF,  5, 0xFF, 4,  0,  4)
#define AUXULP_TRIG1_ULP7    SIWX91X_GPIO(0xFF, 10, 0xFF, 4,  0,  7)

#define CLK_I2SPLL_PB11      SIWX91X_GPIO(12, 0xFF,  0, 1, 11, 0)
#define CLK_I2SPLL_PD0       SIWX91X_GPIO(10, 0xFF, 12, 3,  0, 0)
#define CLK_I2SPLL_PD6       SIWX91X_GPIO(10, 0xFF, 18, 3,  6, 0)
#define CLK_INTFPLL_PB10     SIWX91X_GPIO(12, 0xFF,  0, 1, 10, 0)
#define CLK_INTFPLL_PC15     SIWX91X_GPIO(10, 0xFF, 11, 2, 15, 0)
#define CLK_INTFPLL_PD5      SIWX91X_GPIO(10, 0xFF, 17, 3,  5, 0)
#define CLK_MCUOUT_PA11      SIWX91X_GPIO(12, 0xFF,  6, 0, 11, 0)
#define CLK_MEMSREF_PD2      SIWX91X_GPIO(10, 0xFF, 14, 3,  2, 0)
#define CLK_MEMSREF_PD8      SIWX91X_GPIO(10, 0xFF, 20, 3,  8, 0)
#define CLK_OUT_PA12         SIWX91X_GPIO(8,  0xFF,  7, 0, 12, 0)
#define CLK_OUT_PA15         SIWX91X_GPIO(8,  0xFF,  8, 0, 15, 0)
#define CLK_PLLTESTMODE_PD3  SIWX91X_GPIO(10, 0xFF, 15, 3,  3, 0)
#define CLK_SOCPLL_PB9       SIWX91X_GPIO(12, 0xFF,  0, 1,  9, 0)
#define CLK_SOCPLL_PC14      SIWX91X_GPIO(10, 0xFF, 10, 2, 14, 0)
#define CLK_SOCPLL_PD4       SIWX91X_GPIO(10, 0xFF, 16, 3,  4, 0)
#define CLK_XTALONIN_PB12    SIWX91X_GPIO(12, 0xFF,  0, 1, 12, 0)
#define CLK_XTALONIN_PD9     SIWX91X_GPIO(10, 0xFF, 21, 3,  9, 0)

#define COMP1_OUT_PA8        SIWX91X_GPIO(9,    5,    3, 0,  8, 2)
#define COMP1_OUT_PB12       SIWX91X_GPIO(11,   5,    0, 1, 12, 9)
#define COMP1_OUT_PC15       SIWX91X_GPIO(9,    5,   11, 2, 15, 9)
#define COMP1_OUT_ULP2       SIWX91X_GPIO(0xFF, 5, 0xFF, 4,  0, 2)
#define COMP1_OUT_ULP6       SIWX91X_GPIO(0xFF, 9, 0xFF, 4,  0, 6)
#define COMP2_OUT_ULP7       SIWX91X_GPIO(0xFF, 9, 0xFF, 4,  0, 7)

#define GSPI_CLK_PA8         SIWX91X_GPIO(4,  0xFF,  3, 0,  8, 0)
#define GSPI_CLK_PB9         SIWX91X_GPIO(4,  0xFF,  0, 1,  9, 0)
#define GSPI_CLK_PC14        SIWX91X_GPIO(4,  0xFF, 10, 2, 14, 0)
#define GSPI_CLK_PD4         SIWX91X_GPIO(4,  0xFF, 16, 3,  4, 0)
#define GSPI_CS0_PA9         SIWX91X_GPIO(4,  0xFF,  4, 0,  9, 0)
#define GSPI_CS0_PB12        SIWX91X_GPIO(4,  0xFF,  0, 1, 12, 0)
#define GSPI_CS0_PD1         SIWX91X_GPIO(4,  0xFF, 13, 3,  1, 0)
#define GSPI_CS0_PD5         SIWX91X_GPIO(4,  0xFF, 17, 3,  5, 0)
#define GSPI_CS1_PA10        SIWX91X_GPIO(4,  0xFF,  5, 0, 10, 0)
#define GSPI_CS1_PB13        SIWX91X_GPIO(4,  0xFF,  0, 1, 13, 0)
#define GSPI_CS1_PD2         SIWX91X_GPIO(4,  0xFF, 14, 3,  2, 0)
#define GSPI_CS1_PD6         SIWX91X_GPIO(4,  0xFF, 18, 3,  6, 0)
#define GSPI_CS2_PA15        SIWX91X_GPIO(4,  0xFF,  8, 0, 15, 0)
#define GSPI_CS2_PB14        SIWX91X_GPIO(4,  0xFF,  0, 1, 14, 0)
#define GSPI_CS2_PD3         SIWX91X_GPIO(4,  0xFF, 15, 3,  3, 0)
#define GSPI_CS2_PD7         SIWX91X_GPIO(4,  0xFF, 19, 3,  7, 0)
#define GSPI_MISO_PA11       SIWX91X_GPIO(4,  0xFF,  6, 0, 11, 0)
#define GSPI_MISO_PB10       SIWX91X_GPIO(4,  0xFF,  0, 1, 10, 0)
#define GSPI_MISO_PC15       SIWX91X_GPIO(4,  0xFF, 11, 2, 15, 0)
#define GSPI_MISO_PD8        SIWX91X_GPIO(4,  0xFF, 20, 3,  8, 0)
#define GSPI_MOSI_PA6        SIWX91X_GPIO(12, 0xFF,  1, 0,  6, 0)
#define GSPI_MOSI_PA12       SIWX91X_GPIO(4,  0xFF,  7, 0, 12, 0)
#define GSPI_MOSI_PB11       SIWX91X_GPIO(4,  0xFF,  0, 1, 11, 0)
#define GSPI_MOSI_PD0        SIWX91X_GPIO(4,  0xFF, 12, 3,  0, 0)
#define GSPI_MOSI_PD9        SIWX91X_GPIO(4,  0xFF, 21, 3,  9, 0)

#define I2C0_SCL_PA7         SIWX91X_GPIO(4,  0xFF,  2, 0,  7,  0)
#define I2C0_SCL_PC0         SIWX91X_GPIO(11, 0xFF,  9, 2,  0,  0)
#define I2C0_SCL_ULP1        SIWX91X_GPIO(4,     6, 23, 4,  1,  1)
#define I2C0_SCL_ULP2        SIWX91X_GPIO(4,     6, 24, 4,  2,  2)
#define I2C0_SCL_ULP11       SIWX91X_GPIO(4,     6, 33, 4, 11, 11)
#define I2C0_SDA_PA6         SIWX91X_GPIO(4,  0xFF,  1, 0,  6,  0)
#define I2C0_SDA_PB15        SIWX91X_GPIO(11, 0xFF,  9, 1, 15,  0)
#define I2C0_SDA_ULP0        SIWX91X_GPIO(4,     6, 22, 4,  0,  0)
#define I2C0_SDA_ULP3        SIWX91X_GPIO(4,     6, 25, 4,  3,  3)
#define I2C0_SDA_ULP10       SIWX91X_GPIO(4,     6, 32, 4, 10, 10)

#define I2C1_SCL_PA6         SIWX91X_GPIO(5,  0xFF,  1, 0,  6, 0)
#define I2C1_SCL_PB13        SIWX91X_GPIO(5,  0xFF,  0, 1, 13, 0)
#define I2C1_SCL_PC1         SIWX91X_GPIO(11, 0xFF,  9, 2,  1, 0)
#define I2C1_SCL_PD2         SIWX91X_GPIO(5,  0xFF, 14, 3,  2, 0)
#define I2C1_SCL_PD6         SIWX91X_GPIO(5,  0xFF, 18, 3,  6, 0)
#define I2C1_SCL_ULP0        SIWX91X_GPIO(5,     6, 22, 4,  0, 0)
#define I2C1_SCL_ULP2        SIWX91X_GPIO(5,     6, 24, 4,  2, 2)
#define I2C1_SCL_ULP6        SIWX91X_GPIO(5,     6, 28, 4,  6, 6)
#define I2C1_SDA_PA7         SIWX91X_GPIO(5,  0xFF,  2, 0,  7, 0)
#define I2C1_SDA_PB14        SIWX91X_GPIO(5,  0xFF,  0, 1, 14, 0)
#define I2C1_SDA_PC2         SIWX91X_GPIO(11, 0xFF,  9, 2,  2, 0)
#define I2C1_SDA_PD3         SIWX91X_GPIO(5,  0xFF, 15, 3,  3, 0)
#define I2C1_SDA_PD7         SIWX91X_GPIO(5,  0xFF, 19, 3,  7, 0)
#define I2C1_SDA_ULP1        SIWX91X_GPIO(5,     6, 23, 4,  1, 1)
#define I2C1_SDA_ULP3        SIWX91X_GPIO(5,     6, 25, 4,  3, 3)
#define I2C1_SDA_ULP7        SIWX91X_GPIO(5,     6, 29, 4,  7, 7)

#define I2S0_CLK_PA8         SIWX91X_GPIO(7, 0xFF,  3, 0,  8, 0)
#define I2S0_CLK_PB9         SIWX91X_GPIO(7, 0xFF,  0, 1,  9, 0)
#define I2S0_CLK_PC14        SIWX91X_GPIO(7, 0xFF, 10, 2, 14, 0)
#define I2S0_CLK_PD4         SIWX91X_GPIO(7, 0xFF, 16, 3,  4, 0)
#define I2S0_DIN0_PA10       SIWX91X_GPIO(7, 0xFF,  5, 0, 10, 0)
#define I2S0_DIN0_PB11       SIWX91X_GPIO(7, 0xFF,  0, 1, 11, 0)
#define I2S0_DIN0_PD0        SIWX91X_GPIO(7, 0xFF, 12, 3,  0, 0)
#define I2S0_DIN0_PD8        SIWX91X_GPIO(7, 0xFF, 20, 3,  8, 0)
#define I2S0_DIN1_PA6        SIWX91X_GPIO(7, 0xFF,  1, 0,  6, 0)
#define I2S0_DIN1_PB13       SIWX91X_GPIO(7, 0xFF,  0, 1, 13, 0)
#define I2S0_DIN1_PD2        SIWX91X_GPIO(7, 0xFF, 14, 3,  2, 0)
#define I2S0_DIN1_PD6        SIWX91X_GPIO(7, 0xFF, 18, 3,  6, 0)
#define I2S0_DOUT0_PA11      SIWX91X_GPIO(7, 0xFF,  6, 0, 11, 0)
#define I2S0_DOUT0_PB12      SIWX91X_GPIO(7, 0xFF,  0, 1, 12, 0)
#define I2S0_DOUT0_PD1       SIWX91X_GPIO(7, 0xFF, 13, 3,  1, 0)
#define I2S0_DOUT0_PD9       SIWX91X_GPIO(7, 0xFF, 21, 3,  9, 0)
#define I2S0_DOUT1_PA7       SIWX91X_GPIO(7, 0xFF,  2, 0,  7, 0)
#define I2S0_DOUT1_PB13      SIWX91X_GPIO(7, 0xFF,  0, 1, 14, 0)
#define I2S0_DOUT1_PD3       SIWX91X_GPIO(7, 0xFF, 15, 3,  3, 0)
#define I2S0_DOUT1_PD7       SIWX91X_GPIO(7, 0xFF, 19, 3,  7, 0)
#define I2S0_WS_PA9          SIWX91X_GPIO(7, 0xFF,  4, 0,  9, 0)
#define I2S0_WS_PB10         SIWX91X_GPIO(7, 0xFF,  0, 1, 10, 0)
#define I2S0_WS_PC15         SIWX91X_GPIO(7, 0xFF, 11, 2, 15, 0)
#define I2S0_WS_PD5          SIWX91X_GPIO(7, 0xFF, 17, 3,  5, 0)

#define IR_INPUT_PA15        SIWX91X_GPIO(9,     1,    8, 0, 15,  7)
#define IR_INPUT_PB10        SIWX91X_GPIO(11,    1,    0, 1, 10,  7)
#define IR_INPUT_PB13        SIWX91X_GPIO(11,    4,    0, 1, 13, 10)
#define IR_INPUT_PD0         SIWX91X_GPIO(9,     4,   12, 3,  0, 10)
#define IR_INPUT_ULP4        SIWX91X_GPIO(0xFF, 10, 0xFF, 4,  0,  4)
#define IR_INPUT_ULP7        SIWX91X_GPIO(0xFF,  1, 0xFF, 4,  0,  7)
#define IR_INPUT_ULP10       SIWX91X_GPIO(0xFF,  4, 0xFF, 4,  0, 10)
#define IR_OUTPUT_PA11       SIWX91X_GPIO(9,     1,    6, 0, 11,  5)
#define IR_OUTPUT_ULP5       SIWX91X_GPIO(0xFF,  1, 0xFF, 4,  0,  5)

#define PMU_TEST1_PA6        SIWX91X_GPIO(8,  0xFF,  1, 0,  6,  0)
#define PMU_TEST1_PB13       SIWX91X_GPIO(8,  0xFF,  0, 1, 13,  0)
#define PMU_TEST1_PB14       SIWX91X_GPIO(12, 0xFF,  0, 1, 14,  0)
#define PMU_TEST1_ULP0       SIWX91X_GPIO(13,    6, 22, 4,  0,  0)
#define PMU_TEST1_ULP2       SIWX91X_GPIO(10,    6, 24, 4,  2,  2)
#define PMU_TEST1_ULP6       SIWX91X_GPIO(12,    6, 28, 4,  6,  6)
#define PMU_TEST1_ULP10      SIWX91X_GPIO(10,    6, 32, 4, 10, 10)
#define PMU_TEST2_PA7        SIWX91X_GPIO(8,  0xFF,  2, 0,  7,  0)
#define PMU_TEST2_PB14       SIWX91X_GPIO(8,  0xFF,  0, 1, 14,  0)
#define PMU_TEST2_ULP1       SIWX91X_GPIO(13,    6, 23, 4,  1,  1)
#define PMU_TEST2_ULP3       SIWX91X_GPIO(10,    6, 25, 4,  3,  3)
#define PMU_TEST2_ULP7       SIWX91X_GPIO(12,    6, 29, 4,  7,  7)
#define PMU_TEST2_ULP11      SIWX91X_GPIO(10,    6, 33, 4, 11, 11)

#define PSRAM_CLK_PC14       SIWX91X_GPIO(11, 0xFF, 10, 2, 14, 0)
#define PSRAM_CLK_PD4        SIWX91X_GPIO(12, 0xFF, 16, 3,  4, 0)
#define PSRAM_CSN0_PD1       SIWX91X_GPIO(11, 0xFF, 13, 3,  1, 0)
#define PSRAM_CSN0_PD7       SIWX91X_GPIO(12, 0xFF, 19, 3,  7, 0)
#define PSRAM_CSN1_PD5       SIWX91X_GPIO(11, 0xFF, 17, 3,  5, 0)
#define PSRAM_D0_PC15        SIWX91X_GPIO(11, 0xFF, 11, 2, 15, 0)
#define PSRAM_D0_PD5         SIWX91X_GPIO(12, 0xFF, 17, 3,  5, 0)
#define PSRAM_D1_PD0         SIWX91X_GPIO(11, 0xFF, 12, 3,  0, 0)
#define PSRAM_D1_PD6         SIWX91X_GPIO(12, 0xFF, 18, 3,  6, 0)
#define PSRAM_D2_PD2         SIWX91X_GPIO(11, 0xFF, 14, 3,  2, 0)
#define PSRAM_D2_PD8         SIWX91X_GPIO(12, 0xFF, 20, 3,  8, 0)
#define PSRAM_D3_PD3         SIWX91X_GPIO(11, 0xFF, 15, 3,  3, 0)
#define PSRAM_D3_PD9         SIWX91X_GPIO(12, 0xFF, 21, 3,  9, 0)
#define PSRAM_D4_PD6         SIWX91X_GPIO(11, 0xFF, 18, 3,  6, 0)
#define PSRAM_D5_PD7         SIWX91X_GPIO(11, 0xFF, 19, 3,  7, 0)
#define PSRAM_D6_PD8         SIWX91X_GPIO(11, 0xFF, 20, 3,  8, 0)
#define PSRAM_D7_PD9         SIWX91X_GPIO(11, 0xFF, 21, 3,  9, 0)

#define PWM_0H_PA7           SIWX91X_GPIO(10, 0xFF,  2, 0,  7,  0)
#define PWM_0H_ULP1          SIWX91X_GPIO(12,    6, 23, 4,  1,  1)
#define PWM_0L_PA6           SIWX91X_GPIO(10, 0xFF,  1, 0,  6,  0)
#define PWM_0L_ULP0          SIWX91X_GPIO(12,    6, 22, 4,  0,  0)
#define PWM_1H_PA9           SIWX91X_GPIO(10, 0xFF,  4, 0,  9,  0)
#define PWM_1H_ULP3          SIWX91X_GPIO(8,     6, 25, 4,  3,  3)
#define PWM_1H_ULP5          SIWX91X_GPIO(12,    6, 27, 4,  5,  5)
#define PWM_1L_PA8           SIWX91X_GPIO(10, 0xFF,  3, 0,  8,  0)
#define PWM_1L_ULP2          SIWX91X_GPIO(8,     6, 24, 4,  2,  2)
#define PWM_1L_ULP4          SIWX91X_GPIO(12,    6, 26, 4,  4,  4)
#define PWM_2H_PA11          SIWX91X_GPIO(10, 0xFF,  6, 0, 11,  0)
#define PWM_2H_ULP5          SIWX91X_GPIO(8,     6, 27, 4,  5,  5)
#define PWM_2L_PA10          SIWX91X_GPIO(10, 0xFF,  5, 0, 10,  0)
#define PWM_2L_ULP4          SIWX91X_GPIO(8,     6, 26, 4,  4,  4)
#define PWM_3H_PA15          SIWX91X_GPIO(10, 0xFF,  8, 0, 15,  0)
#define PWM_3H_ULP7          SIWX91X_GPIO(8,     6, 29, 4,  7,  7)
#define PWM_3L_PA12          SIWX91X_GPIO(10, 0xFF,  7, 0, 12,  0)
#define PWM_3L_ULP6          SIWX91X_GPIO(8,     6, 28, 4,  6,  6)
#define PWM_EXTTRIG0_PB11    SIWX91X_GPIO(10, 0xFF,  0, 1, 11,  0)
#define PWM_EXTTRIG0_PD3     SIWX91X_GPIO(8,  0xFF, 15, 3,  3,  0)
#define PWM_EXTTRIG0_ULP6    SIWX91X_GPIO(10,    6, 28, 4,  6,  6)
#define PWM_EXTTRIG0_ULP11   SIWX91X_GPIO(8,     6, 33, 4, 11, 11)
#define PWM_EXTTRIG1_PB12    SIWX91X_GPIO(10, 0xFF,  0, 1, 12,  0)
#define PWM_EXTTRIG1_PD6     SIWX91X_GPIO(8,  0xFF, 18, 3,  6,  0)
#define PWM_EXTTRIG1_ULP7    SIWX91X_GPIO(10,    6, 29, 4,  7,  7)
#define PWM_EXTTRIG2_PB13    SIWX91X_GPIO(10, 0xFF,  0, 1, 13,  0)
#define PWM_EXTTRIG2_PD7     SIWX91X_GPIO(8,  0xFF, 19, 3,  7,  0)
#define PWM_EXTTRIG2_ULP8    SIWX91X_GPIO(10,    6, 30, 4,  8,  8)
#define PWM_EXTTRIG3_PB14    SIWX91X_GPIO(10, 0xFF,  0, 1, 14,  0)
#define PWM_EXTTRIG3_PD2     SIWX91X_GPIO(8,  0xFF, 14, 3,  2,  0)
#define PWM_EXTTRIG3_ULP9    SIWX91X_GPIO(10,    6, 31, 4,  9,  9)
#define PWM_FAULTA_PB9       SIWX91X_GPIO(10, 0xFF,  0, 1,  9,  0)
#define PWM_FAULTA_ULP4      SIWX91X_GPIO(10,    6, 26, 4,  4,  4)
#define PWM_FAULTA_ULP9      SIWX91X_GPIO(8,     6, 31, 4,  9,  9)
#define PWM_FAULTB_PB10      SIWX91X_GPIO(10, 0xFF,  0, 1, 10,  0)
#define PWM_FAULTB_ULP5      SIWX91X_GPIO(10,    6, 27, 4,  5,  5)
#define PWM_FAULTB_ULP10     SIWX91X_GPIO(8,     6, 32, 4, 10, 10)
#define PWM_SLEEPEVENT_ULP8  SIWX91X_GPIO(8,     6, 30, 4,  8,  8)

#define QEI_DIR_PA11         SIWX91X_GPIO(5,  0xFF,  6, 0, 11,  0)
#define QEI_DIR_PB12         SIWX91X_GPIO(5,  0xFF,  0, 1, 12,  0)
#define QEI_DIR_PC2          SIWX91X_GPIO(13, 0xFF,  9, 2,  2,  0)
#define QEI_DIR_PD1          SIWX91X_GPIO(3,  0xFF, 13, 3,  1,  0)
#define QEI_DIR_PD9          SIWX91X_GPIO(5,  0xFF, 21, 3,  9,  0)
#define QEI_DIR_ULP3         SIWX91X_GPIO(3,     6, 25, 4,  3,  3)
#define QEI_DIR_ULP7         SIWX91X_GPIO(3,     6, 29, 4,  7,  7)
#define QEI_DIR_ULP11        SIWX91X_GPIO(3,     6, 33, 4, 11, 11)
#define QEI_IDX_PA8          SIWX91X_GPIO(5,  0xFF,  3, 0,  8,  0)
#define QEI_IDX_PB15         SIWX91X_GPIO(13, 0xFF,  9, 1, 15,  0)
#define QEI_IDX_PB9          SIWX91X_GPIO(5,  0xFF,  0, 1,  9,  0)
#define QEI_IDX_PC14         SIWX91X_GPIO(3,  0xFF, 10, 2, 14,  0)
#define QEI_IDX_PD4          SIWX91X_GPIO(5,  0xFF, 16, 3,  4,  0)
#define QEI_IDX_ULP0         SIWX91X_GPIO(3,     6, 22, 4,  0,  0)
#define QEI_IDX_ULP4         SIWX91X_GPIO(3,     6, 26, 4,  4,  4)
#define QEI_IDX_ULP8         SIWX91X_GPIO(3,     6, 30, 4,  8,  8)
#define QEI_PHA_PA9          SIWX91X_GPIO(5,  0xFF,  4, 0,  9,  0)
#define QEI_PHA_PB10         SIWX91X_GPIO(5,  0xFF,  0, 1, 10,  0)
#define QEI_PHA_PC0          SIWX91X_GPIO(13, 0xFF,  9, 2,  0,  0)
#define QEI_PHA_PC15         SIWX91X_GPIO(3,  0xFF, 11, 2, 15,  0)
#define QEI_PHA_PD5          SIWX91X_GPIO(5,  0xFF, 17, 3,  5,  0)
#define QEI_PHA_ULP1         SIWX91X_GPIO(3,     6, 23, 4,  1,  1)
#define QEI_PHA_ULP5         SIWX91X_GPIO(3,     6, 27, 4,  5,  5)
#define QEI_PHA_ULP9         SIWX91X_GPIO(3,     6, 31, 4,  9,  9)
#define QEI_PHB_PA10         SIWX91X_GPIO(5,  0xFF,  5, 0, 10,  0)
#define QEI_PHB_PB11         SIWX91X_GPIO(5,  0xFF,  0, 1, 11,  0)
#define QEI_PHB_PC1          SIWX91X_GPIO(13, 0xFF,  9, 2,  1,  0)
#define QEI_PHB_PD0          SIWX91X_GPIO(3,  0xFF, 12, 3,  0,  0)
#define QEI_PHB_PD8          SIWX91X_GPIO(5,  0xFF, 20, 3,  8,  0)
#define QEI_PHB_ULP2         SIWX91X_GPIO(3,     6, 24, 4,  2,  2)
#define QEI_PHB_ULP6         SIWX91X_GPIO(3,     6, 28, 4,  6,  6)
#define QEI_PHB_ULP10        SIWX91X_GPIO(3,     6, 32, 4, 10, 10)

#define QSPI_CLK_PA8         SIWX91X_GPIO(11, 0xFF,  3, 0,  8, 0)
#define QSPI_CLK_PC14        SIWX91X_GPIO(1,  0xFF, 10, 2, 14, 0)
#define QSPI_CLK_PD4         SIWX91X_GPIO(9,  0xFF, 16, 3,  4, 0)
#define QSPI_CSN0_PA7        SIWX91X_GPIO(11, 0xFF,  2, 0,  7, 0)
#define QSPI_CSN0_PD1        SIWX91X_GPIO(1,  0xFF, 13, 3,  1, 0)
#define QSPI_CSN0_PD7        SIWX91X_GPIO(9,  0xFF, 19, 3,  7, 0)
#define QSPI_CSN1_PA7        SIWX91X_GPIO(12, 0xFF,  2, 0,  7, 0)
#define QSPI_CSN1_PD5        SIWX91X_GPIO(1,  0xFF, 17, 3,  5, 0)
#define QSPI_CSN9_PD1        SIWX91X_GPIO(10, 0xFF, 13, 3,  1, 0)
#define QSPI_D0_PA6          SIWX91X_GPIO(11, 0xFF,  1, 0,  6, 0)
#define QSPI_D0_PC15         SIWX91X_GPIO(1,  0xFF, 11, 2, 15, 0)
#define QSPI_D0_PD5          SIWX91X_GPIO(9,  0xFF, 17, 3,  5, 0)
#define QSPI_D1_PA9          SIWX91X_GPIO(11, 0xFF,  4, 0,  9, 0)
#define QSPI_D1_PD0          SIWX91X_GPIO(1,  0xFF, 12, 3,  0, 0)
#define QSPI_D1_PD6          SIWX91X_GPIO(9,  0xFF, 18, 3,  6, 0)
#define QSPI_D2_PA10         SIWX91X_GPIO(11, 0xFF,  5, 0, 10, 0)
#define QSPI_D2_PD2          SIWX91X_GPIO(1,  0xFF, 14, 3,  2, 0)
#define QSPI_D2_PD8          SIWX91X_GPIO(9,  0xFF, 20, 3,  8, 0)
#define QSPI_D3_PA11         SIWX91X_GPIO(11, 0xFF,  6, 0, 11, 0)
#define QSPI_D3_PD3          SIWX91X_GPIO(1,  0xFF, 15, 3,  3, 0)
#define QSPI_D3_PD9          SIWX91X_GPIO(9,  0xFF, 21, 3,  9, 0)
#define QSPI_D4_PD6          SIWX91X_GPIO(1,  0xFF, 18, 3,  6, 0)
#define QSPI_D5_PD7          SIWX91X_GPIO(1,  0xFF, 19, 3,  7, 0)
#define QSPI_D6_PD8          SIWX91X_GPIO(1,  0xFF, 20, 3,  8, 0)
#define QSPI_D7_PD9          SIWX91X_GPIO(1,  0xFF, 21, 3,  9, 0)

#define SCT_IN0_PB9          SIWX91X_GPIO(9,  0xFF,  0, 1,  9,  0)
#define SCT_IN0_ULP0         SIWX91X_GPIO(7,     6, 22, 4,  0,  0)
#define SCT_IN0_ULP4         SIWX91X_GPIO(9,     6, 26, 4,  4,  4)
#define SCT_IN1_PB10         SIWX91X_GPIO(9,  0xFF,  0, 1, 10,  0)
#define SCT_IN1_ULP1         SIWX91X_GPIO(7,     6, 23, 4,  1,  1)
#define SCT_IN1_ULP5         SIWX91X_GPIO(9,     6, 27, 4,  5,  5)
#define SCT_IN2_PB11         SIWX91X_GPIO(9,  0xFF,  0, 1, 11,  0)
#define SCT_IN2_ULP2         SIWX91X_GPIO(7,     6, 24, 4,  2,  2)
#define SCT_IN2_ULP6         SIWX91X_GPIO(9,     6, 28, 4,  6,  6)
#define SCT_IN3_PB12         SIWX91X_GPIO(9,  0xFF,  0, 1, 12,  0)
#define SCT_IN3_ULP3         SIWX91X_GPIO(7,     6, 25, 4,  3,  3)
#define SCT_IN3_ULP7         SIWX91X_GPIO(9,     6, 29, 4,  7,  7)
#define SCT_OUT0_PB13        SIWX91X_GPIO(9,  0xFF,  0, 1, 13,  0)
#define SCT_OUT0_ULP4        SIWX91X_GPIO(7,     6, 26, 4,  4,  4)
#define SCT_OUT1_PB14        SIWX91X_GPIO(9,  0xFF,  0, 1, 14,  0)
#define SCT_OUT1_ULP5        SIWX91X_GPIO(7,     6, 27, 4,  5,  5)
#define SCT_OUT2_PA8         SIWX91X_GPIO(12, 0xFF,  3, 0,  8,  0)
#define SCT_OUT2_ULP6        SIWX91X_GPIO(7,     6, 28, 4,  6,  6)
#define SCT_OUT3_PA9         SIWX91X_GPIO(12, 0xFF,  4, 0,  9,  0)
#define SCT_OUT3_ULP7        SIWX91X_GPIO(7,     6, 29, 4,  7,  7)
#define SCT_OUT4_ULP4        SIWX91X_GPIO(13,    6, 26, 4,  4,  4)
#define SCT_OUT4_ULP8        SIWX91X_GPIO(7,     6, 30, 4,  8,  8)
#define SCT_OUT5_ULP5        SIWX91X_GPIO(13,    6, 27, 4,  5,  5)
#define SCT_OUT5_ULP9        SIWX91X_GPIO(7,     6, 31, 4,  9,  9)
#define SCT_OUT6_ULP6        SIWX91X_GPIO(13,    6, 28, 4,  6,  6)
#define SCT_OUT6_ULP10       SIWX91X_GPIO(7,     6, 32, 4, 10, 10)
#define SCT_OUT7_ULP7        SIWX91X_GPIO(13,    6, 29, 4,  7,  7)
#define SCT_OUT7_ULP11       SIWX91X_GPIO(7,     6, 33, 4, 11, 11)

#define SIO_0_PA6            SIWX91X_GPIO(1, 0xFF,  1, 0,  6,  0)
#define SIO_0_PB9            SIWX91X_GPIO(1, 0xFF,  0, 1,  9,  0)
#define SIO_0_ULP0           SIWX91X_GPIO(1,    6, 22, 4,  0,  0)
#define SIO_0_ULP8           SIWX91X_GPIO(1,    6, 30, 4,  8,  8)
#define SIO_1_PA7            SIWX91X_GPIO(1, 0xFF,  2, 0,  7,  0)
#define SIO_1_PB10           SIWX91X_GPIO(1, 0xFF,  0, 1, 10,  0)
#define SIO_1_ULP1           SIWX91X_GPIO(1,    6, 23, 4,  1,  1)
#define SIO_1_ULP9           SIWX91X_GPIO(1,    6, 31, 4,  9,  9)
#define SIO_2_PA8            SIWX91X_GPIO(1, 0xFF,  3, 0,  8,  0)
#define SIO_2_PB11           SIWX91X_GPIO(1, 0xFF,  0, 1, 11,  0)
#define SIO_2_ULP2           SIWX91X_GPIO(1,    6, 24, 4,  2,  2)
#define SIO_2_ULP10          SIWX91X_GPIO(1,    6, 32, 4, 10, 10)
#define SIO_3_PA9            SIWX91X_GPIO(1, 0xFF,  4, 0,  9,  0)
#define SIO_3_PB12           SIWX91X_GPIO(1, 0xFF,  0, 1, 12,  0)
#define SIO_3_ULP3           SIWX91X_GPIO(1,    6, 25, 4,  3,  3)
#define SIO_3_ULP11          SIWX91X_GPIO(1,    6, 33, 4, 11, 11)
#define SIO_4_PA10           SIWX91X_GPIO(1, 0xFF,  5, 0, 10,  0)
#define SIO_4_PB13           SIWX91X_GPIO(1, 0xFF,  0, 1, 13,  0)
#define SIO_4_ULP4           SIWX91X_GPIO(1,    6, 26, 4,  4,  4)
#define SIO_5_PA11           SIWX91X_GPIO(1, 0xFF,  6, 0, 11,  0)
#define SIO_5_PB14           SIWX91X_GPIO(1, 0xFF,  0, 1, 14,  0)
#define SIO_5_ULP5           SIWX91X_GPIO(1,    6, 27, 4,  5,  5)
#define SIO_6_ULP6           SIWX91X_GPIO(1,    6, 28, 4,  6,  6)
#define SIO_7_PA15           SIWX91X_GPIO(1, 0xFF,  8, 0, 15,  0)
#define SIO_7_ULP7           SIWX91X_GPIO(1,    6, 29, 4,  7,  7)

#define SSI_CLK_PA8          SIWX91X_GPIO(3,  0xFF,  3, 0,  8, 0)
#define SSI_CLK_PB9          SIWX91X_GPIO(3,  0xFF,  0, 1,  9, 0)
#define SSI_CLK_PD4          SIWX91X_GPIO(3,  0xFF, 16, 3,  4, 0)
#define SSI_CS0_PA9          SIWX91X_GPIO(3,  0xFF,  4, 0,  9, 0)
#define SSI_CS0_PB12         SIWX91X_GPIO(3,  0xFF,  0, 1, 12, 0)
#define SSI_CS0_PD5          SIWX91X_GPIO(3,  0xFF, 17, 3,  5, 0)
#define SSI_CS1_PA10         SIWX91X_GPIO(3,  0xFF,  5, 0, 10, 0)
#define SSI_CS2_PA15         SIWX91X_GPIO(3,  0xFF,  8, 0, 15, 0)
#define SSI_CS2_PD2          SIWX91X_GPIO(3,  0xFF, 14, 3,  2, 0)
#define SSI_CS3_PD3          SIWX91X_GPIO(3,  0xFF, 15, 3,  3, 0)
#define SSI_DATA0_PA11       SIWX91X_GPIO(3,  0xFF,  6, 0, 11, 0)
#define SSI_DATA0_PB10       SIWX91X_GPIO(3,  0xFF,  0, 1, 10, 0)
#define SSI_DATA0_PD8        SIWX91X_GPIO(3,  0xFF, 20, 3,  8, 0)
#define SSI_DATA1_PA10       SIWX91X_GPIO(12, 0xFF,  5, 0, 10, 0)
#define SSI_DATA1_PA12       SIWX91X_GPIO(3,  0xFF,  7, 0, 12, 0)
#define SSI_DATA1_PB11       SIWX91X_GPIO(3,  0xFF,  0, 1, 11, 0)
#define SSI_DATA1_PD9        SIWX91X_GPIO(3,  0xFF, 21, 3,  9, 0)
#define SSI_DATA2_PA6        SIWX91X_GPIO(3,  0xFF,  1, 0,  6, 0)
#define SSI_DATA2_PB13       SIWX91X_GPIO(3,  0xFF,  0, 1, 13, 0)
#define SSI_DATA2_PD6        SIWX91X_GPIO(3,  0xFF, 18, 3,  6, 0)
#define SSI_DATA3_PA7        SIWX91X_GPIO(3,  0xFF,  2, 0,  7, 0)
#define SSI_DATA3_PB14       SIWX91X_GPIO(3,  0xFF,  0, 1, 14, 0)
#define SSI_DATA3_PD7        SIWX91X_GPIO(3,  0xFF, 19, 3,  7, 0)

#define SSIS_CLK_PA8         SIWX91X_GPIO(8, 0xFF,  3, 0,  8, 0)
#define SSIS_CLK_PB10        SIWX91X_GPIO(8, 0xFF,  0, 1, 10, 0)
#define SSIS_CLK_PC15        SIWX91X_GPIO(8, 0xFF, 11, 2, 15, 0)
#define SSIS_CLK_PD4         SIWX91X_GPIO(8, 0xFF, 16, 3,  4, 0)
#define SSIS_CS_PA9          SIWX91X_GPIO(8, 0xFF,  4, 0,  9, 0)
#define SSIS_CS_PB9          SIWX91X_GPIO(8, 0xFF,  0, 1,  9, 0)
#define SSIS_CS_PC14         SIWX91X_GPIO(8, 0xFF, 10, 2, 14, 0)
#define SSIS_CS_PD5          SIWX91X_GPIO(8, 0xFF, 17, 3,  5, 0)
#define SSIS_MISO_PA11       SIWX91X_GPIO(8, 0xFF,  6, 0, 11, 0)
#define SSIS_MISO_PB12       SIWX91X_GPIO(8, 0xFF,  0, 1, 12, 0)
#define SSIS_MISO_PD1        SIWX91X_GPIO(8, 0xFF, 13, 3,  1, 0)
#define SSIS_MISO_PD9        SIWX91X_GPIO(8, 0xFF, 21, 3,  9, 0)
#define SSIS_MOSI_PA10       SIWX91X_GPIO(8, 0xFF,  5, 0, 10, 0)
#define SSIS_MOSI_PB11       SIWX91X_GPIO(8, 0xFF,  0, 1, 11, 0)
#define SSIS_MOSI_PD0        SIWX91X_GPIO(8, 0xFF, 12, 3,  0, 0)
#define SSIS_MOSI_PD8        SIWX91X_GPIO(8, 0xFF, 20, 3,  8, 0)

#define TIMER0_PA7           SIWX91X_GPIO(9,    5,    2, 0,  7, 1)
#define TIMER0_PB11          SIWX91X_GPIO(11,   5,    0, 1, 11, 8)
#define TIMER0_PC14          SIWX91X_GPIO(9,    5,   10, 2, 14, 8)
#define TIMER0_ULP4          SIWX91X_GPIO(0xFF, 9, 0xFF, 4,  0, 4)
#define TIMER0_ULP8          SIWX91X_GPIO(0xFF, 5, 0xFF, 4,  0, 8)

#define TIMER1_PA15          SIWX91X_GPIO(9,    5,    8, 0, 15, 7)
#define TIMER1_PB10          SIWX91X_GPIO(11,   5,    0, 1, 10, 7)
#define TIMER1_ULP5          SIWX91X_GPIO(0xFF, 9, 0xFF, 4,  0, 5)
#define TIMER1_ULP7          SIWX91X_GPIO(0xFF, 5, 0xFF, 4,  0, 7)

#define TIMER2_ULP1          SIWX91X_GPIO(0xFF, 5, 0xFF, 4, 0, 1)

#define TRACE_CLK_PA7        SIWX91X_GPIO(13, 0xFF,  2, 0,  7, 0)
#define TRACE_CLK_PC15       SIWX91X_GPIO(6,  0xFF, 11, 2, 15, 0)
#define TRACE_CLK_PD5        SIWX91X_GPIO(6,  0xFF, 17, 3,  5, 0)
#define TRACE_CLKIN_PA6      SIWX91X_GPIO(13, 0xFF,  1, 0,  6, 0)
#define TRACE_CLKIN_PA15     SIWX91X_GPIO(6,  0xFF,  8, 0, 15, 0)
#define TRACE_CLKIN_PC14     SIWX91X_GPIO(6,  0xFF, 10, 2, 14, 0)
#define TRACE_CLKIN_PD4      SIWX91X_GPIO(6,  0xFF, 16, 3,  4, 0)
#define TRACE_D0_PA8         SIWX91X_GPIO(13, 0xFF,  3, 0,  8, 0)
#define TRACE_D0_PD0         SIWX91X_GPIO(6,  0xFF, 12, 3,  0, 0)
#define TRACE_D0_PD6         SIWX91X_GPIO(6,  0xFF, 18, 3,  6, 0)
#define TRACE_D1_PA9         SIWX91X_GPIO(13, 0xFF,  4, 0,  9, 0)
#define TRACE_D1_PD1         SIWX91X_GPIO(6,  0xFF, 13, 3,  1, 0)
#define TRACE_D1_PD7         SIWX91X_GPIO(6,  0xFF, 19, 3,  7, 0)
#define TRACE_D2_PA10        SIWX91X_GPIO(13, 0xFF,  5, 0, 10, 0)
#define TRACE_D2_PD2         SIWX91X_GPIO(6,  0xFF, 14, 3,  2, 0)
#define TRACE_D2_PD8         SIWX91X_GPIO(6,  0xFF, 20, 3,  8, 0)
#define TRACE_D3_PA11        SIWX91X_GPIO(13, 0xFF,  6, 0, 11, 0)
#define TRACE_D3_PD3         SIWX91X_GPIO(6,  0xFF, 15, 3,  3, 0)
#define TRACE_D3_PD9         SIWX91X_GPIO(6,  0xFF, 21, 3,  9, 0)

#define UART1_CLK_PA8        SIWX91X_GPIO(2,  0xFF,  3, 0,  8,  0)
#define UART1_CLK_PB9        SIWX91X_GPIO(2,  0xFF,  0, 1,  9,  0)
#define UART1_CLK_PD4        SIWX91X_GPIO(2,  0xFF, 16, 3,  4,  0)
#define UART1_CLK_ULP0       SIWX91X_GPIO(2,     6, 22, 4,  0,  0)
#define UART1_CTS_PA6        SIWX91X_GPIO(2,  0xFF,  1, 0,  6,  0)
#define UART1_CTS_PB10       SIWX91X_GPIO(2,  0xFF,  0, 1, 10,  0)
#define UART1_CTS_PD8        SIWX91X_GPIO(2,  0xFF, 20, 3,  8,  0)
#define UART1_CTS_ULP6       SIWX91X_GPIO(2,     6, 28, 4,  6,  6)
#define UART1_DCD_PA12       SIWX91X_GPIO(2,  0xFF,  7, 0, 12,  0)
#define UART1_DCD_PB13       SIWX91X_GPIO(12, 0xFF,  0, 1, 13,  0)
#define UART1_DSR_PA11       SIWX91X_GPIO(2,  0xFF,  6, 0, 11,  0)
#define UART1_DSR_PD9        SIWX91X_GPIO(2,  0xFF, 21, 3,  9,  0)
#define UART1_DTR_PA7        SIWX91X_GPIO(2,  0xFF,  2, 0,  7,  0)
#define UART1_IRRX_PB9       SIWX91X_GPIO(13, 0xFF,  0, 1,  9,  0)
#define UART1_IRRX_PC15      SIWX91X_GPIO(2,  0xFF, 11, 2, 15,  0)
#define UART1_IRRX_ULP0      SIWX91X_GPIO(11,    6, 22, 4,  0,  0)
#define UART1_IRRX_ULP7      SIWX91X_GPIO(2,     6, 29, 4,  7,  7)
#define UART1_IRTX_PB10      SIWX91X_GPIO(13, 0xFF,  0, 1, 10,  0)
#define UART1_IRTX_PD0       SIWX91X_GPIO(2,  0xFF, 12, 3,  0,  0)
#define UART1_IRTX_ULP1      SIWX91X_GPIO(11,    6, 23, 4,  1,  1)
#define UART1_IRTX_ULP8      SIWX91X_GPIO(2,     6, 30, 4,  8,  8)
#define UART1_RI_PB11        SIWX91X_GPIO(2,  0xFF,  0, 1, 11,  0)
#define UART1_RI_PC14        SIWX91X_GPIO(2,  0xFF, 10, 2, 14,  0)
#define UART1_RI_ULP4        SIWX91X_GPIO(11,    6, 26, 4,  4,  4)
#define UART1_RS485DE_PB13   SIWX91X_GPIO(13, 0xFF,  0, 1, 13,  0)
#define UART1_RS485DE_PD3    SIWX91X_GPIO(2,  0xFF, 15, 3,  3,  0)
#define UART1_RS485DE_ULP7   SIWX91X_GPIO(11,    6, 29, 4,  7,  7)
#define UART1_RS485DE_ULP11  SIWX91X_GPIO(2,     6, 33, 4, 11, 11)
#define UART1_RS485EN_PB11   SIWX91X_GPIO(13, 0xFF,  0, 1, 11,  0)
#define UART1_RS485EN_PD1    SIWX91X_GPIO(2,  0xFF, 13, 3,  1,  0)
#define UART1_RS485EN_ULP5   SIWX91X_GPIO(11,    6, 27, 4,  5,  5)
#define UART1_RS485EN_ULP9   SIWX91X_GPIO(2,     6, 31, 4,  9,  9)
#define UART1_RS485RE_PB12   SIWX91X_GPIO(13, 0xFF,  0, 1, 12,  0)
#define UART1_RS485RE_PD2    SIWX91X_GPIO(2,  0xFF, 14, 3,  2,  0)
#define UART1_RS485RE_ULP6   SIWX91X_GPIO(11,    6, 28, 4,  6,  6)
#define UART1_RS485RE_ULP10  SIWX91X_GPIO(2,     6, 32, 4, 10, 10)
#define UART1_RTS_PA9        SIWX91X_GPIO(2,  0xFF,  4, 0,  9,  0)
#define UART1_RTS_PB12       SIWX91X_GPIO(2,  0xFF,  0, 1, 12,  0)
#define UART1_RTS_PD5        SIWX91X_GPIO(2,  0xFF, 17, 3,  5,  0)
#define UART1_RTS_ULP5       SIWX91X_GPIO(2,     6, 27, 4,  5,  5)
#define UART1_RX_PA10        SIWX91X_GPIO(2,  0xFF,  5, 0, 10,  0)
#define UART1_RX_PB13        SIWX91X_GPIO(2,  0xFF,  0, 1, 13,  0)
#define UART1_RX_PD7         SIWX91X_GPIO(2,  0xFF, 19, 3,  7,  0)
#define UART1_RX_ULP1        SIWX91X_GPIO(2,     6, 23, 4,  1,  1)
#define UART1_RX_ULP6        SIWX91X_GPIO(4,     6, 28, 4,  6,  6)
#define UART1_TX_PB14        SIWX91X_GPIO(2,  0xFF,  0, 1, 14,  0)
#define UART1_TX_PD6         SIWX91X_GPIO(2,  0xFF, 18, 3,  6,  0)
#define UART1_TX_ULP4        SIWX91X_GPIO(2,     6, 26, 4,  4,  4)
#define UART1_TX_ULP7        SIWX91X_GPIO(4,     6, 29, 4,  7,  7)

#define UART2_CTS_PA11       SIWX91X_GPIO(6,  0xFF,  6, 0, 11,  0)
#define UART2_CTS_PC0        SIWX91X_GPIO(12, 0xFF,  9, 2,  0,  0)
#define UART2_CTS_PD3        SIWX91X_GPIO(9,  0xFF, 15, 3,  3,  0)
#define UART2_CTS_ULP1       SIWX91X_GPIO(9,     6, 23, 4,  1,  1)
#define UART2_CTS_ULP7       SIWX91X_GPIO(6,     6, 29, 4,  7,  7)
#define UART2_CTS_ULP9       SIWX91X_GPIO(9,     6, 31, 4,  9,  9)
#define UART2_RS485DE_PA9    SIWX91X_GPIO(6,  0xFF,  4, 0,  9,  0)
#define UART2_RS485DE_ULP2   SIWX91X_GPIO(6,     6, 24, 4,  2,  2)
#define UART2_RS485DE_ULP11  SIWX91X_GPIO(6,     6, 33, 4, 11, 11)
#define UART2_RS485EN_PA12   SIWX91X_GPIO(6,  0xFF,  7, 0, 12,  0)
#define UART2_RS485EN_PB10   SIWX91X_GPIO(6,  0xFF,  0, 1, 10,  0)
#define UART2_RS485EN_ULP0   SIWX91X_GPIO(6,     6, 22, 4,  0,  0)
#define UART2_RS485RE_PA8    SIWX91X_GPIO(6,  0xFF,  3, 0,  8,  0)
#define UART2_RS485RE_ULP1   SIWX91X_GPIO(6,     6, 23, 4,  1,  1)
#define UART2_RS485RE_ULP10  SIWX91X_GPIO(6,     6, 32, 4, 10, 10)
#define UART2_RTS_PA10       SIWX91X_GPIO(6,  0xFF,  5, 0, 10,  0)
#define UART2_RTS_PB11       SIWX91X_GPIO(6,  0xFF,  0, 1, 11,  0)
#define UART2_RTS_PB12       SIWX91X_GPIO(6,  0xFF,  0, 1, 12,  0)
#define UART2_RTS_PB15       SIWX91X_GPIO(12, 0xFF,  9, 1, 15,  0)
#define UART2_RTS_PD2        SIWX91X_GPIO(9,  0xFF, 14, 3,  2,  0)
#define UART2_RTS_ULP0       SIWX91X_GPIO(9,     6, 22, 4,  0,  0)
#define UART2_RTS_ULP6       SIWX91X_GPIO(6,     6, 28, 4,  6,  6)
#define UART2_RTS_ULP8       SIWX91X_GPIO(9,     6, 30, 4,  8,  8)
#define UART2_RX_PA6         SIWX91X_GPIO(6,  0xFF,  1, 0,  6,  0)
#define UART2_RX_PB13        SIWX91X_GPIO(6,  0xFF,  0, 1, 13,  0)
#define UART2_RX_PC1         SIWX91X_GPIO(12, 0xFF,  9, 2,  1,  0)
#define UART2_RX_ULP2        SIWX91X_GPIO(9,     6, 24, 4,  1,  1)
#define UART2_RX_ULP4        SIWX91X_GPIO(6,     6, 26, 4,  4,  4)
#define UART2_RX_ULP8        SIWX91X_GPIO(6,     6, 30, 4,  8,  8)
#define UART2_RX_ULP10       SIWX91X_GPIO(9,     6, 32, 4, 10, 10)
#define UART2_TX_PA15        SIWX91X_GPIO(2,  0xFF,  8, 0, 15,  0)
#define UART2_TX_PA7         SIWX91X_GPIO(6,  0xFF,  2, 0,  7,  0)
#define UART2_TX_PB14        SIWX91X_GPIO(6,  0xFF,  0, 1, 14,  0)
#define UART2_TX_PC2         SIWX91X_GPIO(12, 0xFF,  9, 2,  2,  0)
#define UART2_TX_ULP3        SIWX91X_GPIO(9,     6, 25, 4,  1,  1)
#define UART2_TX_ULP5        SIWX91X_GPIO(6,     6, 27, 4,  5,  5)
#define UART2_TX_ULP9        SIWX91X_GPIO(6,     6, 31, 4,  9,  9)
#define UART2_TX_ULP11       SIWX91X_GPIO(9,     6, 33, 4, 11, 11)

#define ULPI2C_SCL_PA11      SIWX91X_GPIO(9,    4,    6, 0, 11,  5)
#define ULPI2C_SCL_PA15      SIWX91X_GPIO(9,    4,    8, 0, 15,  7)
#define ULPI2C_SCL_PA7       SIWX91X_GPIO(9,    4,    2, 0,  7,  1)
#define ULPI2C_SCL_PB10      SIWX91X_GPIO(11,   4,    0, 1, 10,  7)
#define ULPI2C_SCL_PB11      SIWX91X_GPIO(11,   4,    0, 1, 11,  8)
#define ULPI2C_SCL_PC14      SIWX91X_GPIO(9,    4,   10, 2, 14,  8)
#define ULPI2C_SCL_ULP1      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  1)
#define ULPI2C_SCL_ULP5      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  5)
#define ULPI2C_SCL_ULP7      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  7)
#define ULPI2C_SCL_ULP8      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  8)
#define ULPI2C_SDA_PA6       SIWX91X_GPIO(9,    4,    1, 0,  6,  0)
#define ULPI2C_SDA_PA10      SIWX91X_GPIO(9,    4,    5, 0, 10,  4)
#define ULPI2C_SDA_PA12      SIWX91X_GPIO(9,    4,    7, 0, 12,  6)
#define ULPI2C_SDA_PB9       SIWX91X_GPIO(11,   4,    0, 1,  9,  6)
#define ULPI2C_SDA_PB12      SIWX91X_GPIO(11,   4,    0, 1, 12,  9)
#define ULPI2C_SDA_PB14      SIWX91X_GPIO(11,   4,    0, 1, 14, 11)
#define ULPI2C_SDA_PC15      SIWX91X_GPIO(9,    4,   11, 2, 15,  9)
#define ULPI2C_SDA_PD1       SIWX91X_GPIO(9,    4,   13, 3,  1, 11)
#define ULPI2C_SDA_ULP0      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  0)
#define ULPI2C_SDA_ULP4      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  4)
#define ULPI2C_SDA_ULP6      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  6)
#define ULPI2C_SDA_ULP9      SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0,  9)
#define ULPI2C_SDA_ULP11     SIWX91X_GPIO(0xFF, 4, 0xFF, 4,  0, 11)

#define ULPI2S_CLK_PA15      SIWX91X_GPIO(9,    2,    8, 0, 15,  7)
#define ULPI2S_CLK_PB10      SIWX91X_GPIO(11,   2,    0, 1, 10,  7)
#define ULPI2S_CLK_PB11      SIWX91X_GPIO(11,   2,    0, 1, 11,  8)
#define ULPI2S_CLK_PC14      SIWX91X_GPIO(9,    2,   10, 2, 14,  8)
#define ULPI2S_CLK_ULP7      SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  7)
#define ULPI2S_CLK_ULP8      SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  8)
#define ULPI2S_DIN_PA12      SIWX91X_GPIO(9,    2,    7, 0, 12,  6)
#define ULPI2S_DIN_PA6       SIWX91X_GPIO(9,    2,    1, 0,  6,  0)
#define ULPI2S_DIN_PB9       SIWX91X_GPIO(11,   2,    0, 1,  9,  6)
#define ULPI2S_DIN_PB12      SIWX91X_GPIO(11,   2,    0, 1, 12,  9)
#define ULPI2S_DIN_PC15      SIWX91X_GPIO(9,    2,   11, 2, 15,  9)
#define ULPI2S_DIN_ULP0      SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  0)
#define ULPI2S_DIN_ULP6      SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  6)
#define ULPI2S_DIN_ULP9      SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  9)
#define ULPI2S_DOUT_PA7      SIWX91X_GPIO(9,    2,    2, 0,  7,  1)
#define ULPI2S_DOUT_PA11     SIWX91X_GPIO(9,    2,    6, 0, 11,  5)
#define ULPI2S_DOUT_PB14     SIWX91X_GPIO(11,   2,    0, 1, 14, 11)
#define ULPI2S_DOUT_PD1      SIWX91X_GPIO(9,    2,   13, 3,  1, 11)
#define ULPI2S_DOUT_ULP1     SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  1)
#define ULPI2S_DOUT_ULP5     SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  5)
#define ULPI2S_DOUT_ULP11    SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0, 11)
#define ULPI2S_WS_PA8        SIWX91X_GPIO(9,    2,    3, 0,  8,  2)
#define ULPI2S_WS_PA10       SIWX91X_GPIO(9,    2,    5, 0, 10,  4)
#define ULPI2S_WS_PB13       SIWX91X_GPIO(11,   2,    0, 1, 13, 10)
#define ULPI2S_WS_PD0        SIWX91X_GPIO(9,    2,   12, 3,  0, 10)
#define ULPI2S_WS_ULP2       SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  2)
#define ULPI2S_WS_ULP4       SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0,  4)
#define ULPI2S_WS_ULP10      SIWX91X_GPIO(0xFF, 2, 0xFF, 4,  0, 10)

#define ULPSSI_CLK_PA6       SIWX91X_GPIO(9,    1,    1, 0,  6,  0)
#define ULPSSI_CLK_PB11      SIWX91X_GPIO(11,   1,    0, 1, 11,  8)
#define ULPSSI_CLK_PC14      SIWX91X_GPIO(9,    1,   10, 2, 14,  8)
#define ULPSSI_CLK_ULP0      SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0,  0)
#define ULPSSI_CLK_ULP4      SIWX91X_GPIO(0xFF, 8, 0xFF, 4,  0,  4)
#define ULPSSI_CLK_ULP8      SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0,  8)
#define ULPSSI_CS0_PB13      SIWX91X_GPIO(11,   1,    0, 1, 13, 10)
#define ULPSSI_CS0_PD0       SIWX91X_GPIO(9,    1,   12, 3,  0, 10)
#define ULPSSI_CS0_ULP7      SIWX91X_GPIO(0xFF, 8, 0xFF, 4,  0,  7)
#define ULPSSI_CS0_ULP10     SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0, 10)
#define ULPSSI_CS1_PA10      SIWX91X_GPIO(9,    1,    5, 0, 10,  4)
#define ULPSSI_CS1_ULP4      SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0,  4)
#define ULPSSI_CS2_PA12      SIWX91X_GPIO(9,    1,    7, 0, 12,  6)
#define ULPSSI_CS2_PB9       SIWX91X_GPIO(11,   1,    0, 1,  9,  6)
#define ULPSSI_CS2_ULP6      SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0,  6)
#define ULPSSI_DIN_PA8       SIWX91X_GPIO(9,    1,    3, 0,  8,  2)
#define ULPSSI_DIN_PB12      SIWX91X_GPIO(11,   1,    0, 1, 12,  9)
#define ULPSSI_DIN_PC15      SIWX91X_GPIO(9,    1,   11, 2, 15,  9)
#define ULPSSI_DIN_ULP2      SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0,  2)
#define ULPSSI_DIN_ULP6      SIWX91X_GPIO(0xFF, 8, 0xFF, 4,  0,  6)
#define ULPSSI_DIN_ULP9      SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0,  9)
#define ULPSSI_DOUT_PA7      SIWX91X_GPIO(9,    1,    2, 0,  7,  1)
#define ULPSSI_DOUT_PB14     SIWX91X_GPIO(11,   1,    0, 1, 14, 11)
#define ULPSSI_DOUT_PD1      SIWX91X_GPIO(9,    1,   13, 3,  1, 11)
#define ULPSSI_DOUT_ULP1     SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0,  1)
#define ULPSSI_DOUT_ULP5     SIWX91X_GPIO(0xFF, 8, 0xFF, 4,  0,  5)
#define ULPSSI_DOUT_ULP11    SIWX91X_GPIO(0xFF, 1, 0xFF, 4,  0, 11)

#define ULPUART_CTS_PA7      SIWX91X_GPIO(9,    3,    2, 0,  7,  1)
#define ULPUART_CTS_PA11     SIWX91X_GPIO(9,    3,    6, 0, 11,  5)
#define ULPUART_CTS_PB11     SIWX91X_GPIO(11,   3,    0, 1, 11,  8)
#define ULPUART_CTS_PC14     SIWX91X_GPIO(9,    3,   10, 2, 14,  8)
#define ULPUART_CTS_ULP1     SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  1)
#define ULPUART_CTS_ULP5     SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  5)
#define ULPUART_CTS_ULP8     SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  8)
#define ULPUART_RTS_PA6      SIWX91X_GPIO(9,    3,    1, 0,  6,  0)
#define ULPUART_RTS_PA10     SIWX91X_GPIO(9,    3,    5, 0, 10,  4)
#define ULPUART_RTS_PB13     SIWX91X_GPIO(11,   3,    0, 1, 13, 10)
#define ULPUART_RTS_PD0      SIWX91X_GPIO(9,    3,   12, 3,  0, 10)
#define ULPUART_RTS_ULP0     SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  0)
#define ULPUART_RTS_ULP4     SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  4)
#define ULPUART_RTS_ULP10    SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0, 10)
#define ULPUART_RX_PA8       SIWX91X_GPIO(9,    3,    3, 0,  8,  2)
#define ULPUART_RX_PA12      SIWX91X_GPIO(9,    3,    7, 0, 12,  6)
#define ULPUART_RX_PB9       SIWX91X_GPIO(11,   3,    0, 1,  9,  6)
#define ULPUART_RX_PB12      SIWX91X_GPIO(11,   3,    0, 1, 12,  9)
#define ULPUART_RX_PC15      SIWX91X_GPIO(9,    3,   11, 2, 15,  9)
#define ULPUART_RX_ULP2      SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  2)
#define ULPUART_RX_ULP6      SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  6)
#define ULPUART_RX_ULP9      SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  9)
#define ULPUART_TX_PA15      SIWX91X_GPIO(9,    3,    8, 0, 15,  7)
#define ULPUART_TX_PB10      SIWX91X_GPIO(11,   3,    0, 1, 10,  7)
#define ULPUART_TX_PB14      SIWX91X_GPIO(11,   3,    0, 1, 14, 11)
#define ULPUART_TX_PD1       SIWX91X_GPIO(9,    3,   13, 3,  1, 11)
#define ULPUART_TX_ULP7      SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0,  7)
#define ULPUART_TX_ULP11     SIWX91X_GPIO(0xFF, 3, 0xFF, 4,  0, 11)

#define UULP_GPIO4_ULP2      SIWX91X_GPIO(0xFF,  4, 0xFF, 4, 0, 2)
#define UULP_TESTMODE0_ULP7  SIWX91X_GPIO(0xFF, 11, 0xFF, 4, 0, 7)
#define UULP_TESTMODE0_ULP9  SIWX91X_GPIO(0xFF,  5, 0xFF, 4, 0, 9)

/* clang-format on */

/* The following definitions are duplicates of signals that are also
 * available on the same pins using other GPIO modes.
 * #define IR_OUTPUT_ULP5       SIWX91X_GPIO(0xFF, 10, 0xFF, 4, 0, 5)
 * #define PMU_TEST2_PB14       SIWX91X_GPIO(13, 0xFF, 0, 1, 14, 0)
 * #define PWM_1H_ULP1          SIWX91X_GPIO(8, 6, 23, 4, 1, 1)
 * #define PWM_1L_ULP0          SIWX91X_GPIO(8, 6, 22, 4, 0, 0)
 */

#endif /* INCLUDE_ZEPHYR_DT_BINDINGS_PINCTRL_SILABS_SIWX91X_PINCTRL_H_ */
