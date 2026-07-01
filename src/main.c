/*==================================================================================================
* Project : RTD AUTOSAR 4.9
* Platform : CORTEXM
* Peripheral : S32K3XX
* Dependencies : none
*
* Autosar Version : 4.9.0
* Autosar Revision : ASR_REL_4_9_REV_0000
* Autosar Conf.Variant :
* SW Version : 7.0.0
* Build Version : S32K3_RTD_7_0_0_QLP03_D2512_ASR_REL_4_9_REV_0000_20251210
*
* Copyright 2020 - 2026 NXP
*
*   NXP Proprietary. This software is owned or controlled by NXP and may only be
*   used strictly in accordance with the applicable license terms. By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms. If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

/**
*   @file main.c
*
*   @addtogroup main_module main module documentation
*   @{
*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "Clock_Ip.h"

/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/

/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

/*==================================================================================================
* DWT (Data Watchpoint and Trace) Registers
==================================================================================================*/
#define DWT_CTRL        (*(volatile uint32*)0xE0001000UL)  /* DWT Control Register */
#define DWT_CYCCNT      (*(volatile uint32*)0xE0001004UL)  /* DWT Cycle Count Register */
#define SCB_DEMCR       (*(volatile uint32*)0xE000EDFCUL)  /* Debug Exception and Monitor Control */

/* Control bits */
#define SCB_DEMCR_TRCENA    (1UL << 24)  /* Trace enable */
#define DWT_CTRL_CYCCNTENA  (1UL << 0)   /* Cycle counter enable */

/**
 * @brief Initialize DWT Cycle Counter
 */
void DWT_Init(void)
{
    /* Enable DWT and ITM blocks */
    SCB_DEMCR |= SCB_DEMCR_TRCENA;

    /* Reset cycle counter */
    DWT_CYCCNT = 0UL;

    /* Enable cycle counter */
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

/**
 * @brief Get current cycle count
 * @return Current cycle count (32-bit, counts UP)
 */
uint32 DWT_GetCycles(void)
{
    return DWT_CYCCNT;
}

/**
 * @brief Wait for specified time in milliseconds
 * @param ms - delay in milliseconds
 * @note Uses hardware cycle counter, no for loops!
 */
void delay_ms(uint32 ms)
{
    uint32 coreFreq;
    uint32 cyclesNeeded;
    uint32 startCycles;

    /* Get actual core clock frequency */
    coreFreq = Clock_Ip_GetClockFrequency(CORE_CLK);

    /* Calculate cycles needed for desired delay */
    cyclesNeeded = (coreFreq / 1000UL) * ms;

    /* Get starting cycle count */
    startCycles = DWT_GetCycles();

    /* Wait until elapsed cycles >= cyclesNeeded */
    /* DWT counts UP and automatically wraps at 32-bit, but subtraction handles it */
    while ((DWT_GetCycles() - startCycles) < cyclesNeeded)
    {
        /* Just reading hardware counter - no for loop! */
    }
}

/**
 * @brief Main function - LED blink with accurate clock-based timing
 * @details Initializes clock, then blinks LEDs every 500ms
 */
int main(void)
{
    Clock_Ip_StatusType clockStatus;

    /* ===== STEP 1: Initialize Clock===== */
    clockStatus = Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);

    if (clockStatus != CLOCK_IP_SUCCESS)
    {
        /* Clock initialization failed - indicate with fast blink */
        Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
        		g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

        while (1)
        {
            /* Fast blink = error */
            Siul2_Dio_Ip_WritePin(LED_PTA29_PORT, LED_PTA29_PIN, 1U);
            delay_ms(100);

            Siul2_Dio_Ip_WritePin(LED_PTA29_PORT, LED_PTA29_PIN, 0U);
            delay_ms(100);
        }
    }

    /* Initialize DWT cycle counter */
    DWT_Init();

    /* ===== STEP 2: Initialize GPIO (from blinky_pemicro) ===== */
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
    		g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    /* ===== STEP 3: Blink LED every 500ms using clock-based delay ===== */
    while (1)
    {
        /* LED ON */
        Siul2_Dio_Ip_WritePin(LED_PTA29_PORT, LED_PTA29_PIN, 1U);
        delay_ms(500);

        /* LED OFF */
        Siul2_Dio_Ip_WritePin(LED_PTA29_PORT, LED_PTA29_PIN, 0U);
        delay_ms(500);
    }

    return (0U);
}

#ifdef __cplusplus
}
#endif
