/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for TCPWM Capture Mode Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Debug UART context */
cy_stc_scb_uart_context_t  DEBUG_UART_context;
/* Debug UART HAL object */
mtb_hal_uart_t DEBUG_UART_hal_obj;

/* configure capture interrupt instance (capture or overflow)  */
const cy_stc_sysint_t Capture_IRQ_cfg = {
    .intrSrc      =  CAPTURE_IRQ,
    .intrPriority = 3UL
};

uint32_t capture_period;
uint32_t capture_duty;
unsigned long capture_fre;
uint32_t CaptureSignal = 0;
float DutyCycle;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void ISR_TCPWM_Capture(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*    1. Configure a PWM: 100kHZ and 25% duty cycle
*    2. Configure a counter as Capture mode to measure the frequency and duty cycle
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);
    /* Initialize HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** \
    TCPWM Capture Example ... \
    ****************** \r\n\n");

    /*Init and start PWM: 100KHZ and 25% duty-cycle*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(PWM_HW, PWM_NUM, &PWM_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(PWM_HW, PWM_NUM);

    /* Then start the PWM */
    Cy_TCPWM_TriggerStart_Single(PWM_HW, PWM_NUM);

    /*TCPWM Capture Mode initial*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(CAPTURE_HW, CAPTURE_NUM, &CAPTURE_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized counter */
    Cy_TCPWM_Counter_Enable(CAPTURE_HW, CAPTURE_NUM);

    /* Configure and enable interrupt */
    Cy_SysInt_Init(&Capture_IRQ_cfg, ISR_TCPWM_Capture);
    NVIC_EnableIRQ(Capture_IRQ_cfg.intrSrc);

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        if (CaptureSignal)
        {
            /* Print captured value of input signal: frequency and duty cycle */
            printf("Frequency: %lu HZ\r\n", (long unsigned)capture_fre);
            printf("duty: %.3f %% \r\n", DutyCycle);

            CaptureSignal = 0;
        }

    }
}

void ISR_TCPWM_Capture(void)
{
    /* Read interrupt trigger source */
    uint32_t status = Cy_TCPWM_Counter_GetStatus(CAPTURE_HW, CAPTURE_NUM);

    if(CY_TCPWM_INT_ON_CC0 & status)
    {
        /* Fetch the values of period and duty*/
        capture_period = Cy_TCPWM_Counter_GetCapture0Val(CAPTURE_HW, CAPTURE_NUM);
        capture_duty = Cy_TCPWM_Counter_GetCapture1Val(CAPTURE_HW, CAPTURE_NUM);

        /*Calculate the frequency and duty cycle per the captured value, the peripheral clock is 240MHZ*/
        capture_fre = 240000000/capture_period;
        DutyCycle = ((float)capture_duty*100) / (float)capture_period;

        CaptureSignal = 1;

    }

    /* clear interrupt*/
    Cy_TCPWM_ClearInterrupt(CAPTURE_HW, CAPTURE_NUM, status);

}

/* [] END OF FILE */
