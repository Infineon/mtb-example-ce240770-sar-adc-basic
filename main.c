/******************************************************************************
* File Name:   main.c
*
* Description: In this code example, SAR ADC is configured to sample input 
*              voltage and display it on the terminal software.
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
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define INTERVAL_MS                 (200u)
#define VOLTAGE_SOURCE CY_CFG_PWR_VDDA_MV

/* Maximum resolution of 12 bit SAR ADC  */
#define MAX_RESOLUTION              (0xFFF)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
/* Function to read input voltage from channel 0 */
void get_adc_result(void);

/* Function to print input voltage from channel 0 */
void print_adc_result(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
uint16_t g_adcResult;

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM7_0 CPU. It does...
*    1. Configure and initialize ADC.
*    2. Every 200ms read the input voltage and display input voltage on UART.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
    Cy_SCB_UART_Enable(UART_HW);
    result = cy_retarget_io_init(UART_HW);

    /* retarget-io init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    printf("retarget-io ver1.6 testing \r\n");

    /* Print message */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("-----------------------------------------------------------\r\n");
    printf("PDL: SAR ADC Basic\r\n");
    printf("-----------------------------------------------------------\r\n\n");

    /* Initialize SAR and Channel 0 */
    result = Cy_SAR2_Init(SAR_HW, &SAR_config);
    if(result != CY_SAR2_SUCCESS)
    {
        printf("ADC initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* Initialize SAR */
    Cy_SAR2_Enable(SAR_HW);

    for (;;)
    {
        /* Start SAR ADC */
        Cy_SAR2_Channel_SoftwareTrigger(SAR_HW, SAR_CH_0_IDX);

        /* Get ADC result */
        get_adc_result();

        /* print ADC result in millivolt */
        print_adc_result();

        /* 200 millisecond delay between scans */
        Cy_SysLib_Delay(INTERVAL_MS);
    }
}

void get_adc_result(void)
{
    /* Wait until SAR conversion is done */
    while(CY_SAR2_GRP_COMPLETE != Cy_SAR2_Channel_GetGroupStatus(SAR_HW, SAR_CH_0_IDX));

    /* Variable to store ADC conversion result from channel 0 */
    g_adcResult = Cy_SAR2_Channel_GetResult(SAR_HW, SAR_CH_0_IDX, NULL);
}

/*******************************************************************************
 * Function Name: print_adc_result
 *******************************************************************************
 *
 * Summary:
 *  ADC single channel process function. This function reads the input voltage
 *  and prints the input voltage on UART.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void print_adc_result(void)
{
    /* Convert input voltage to millivolt and print it */
    uint16_t resultInMv = VOLTAGE_SOURCE * g_adcResult / MAX_RESOLUTION;
    printf("Channel 0 input: %4umV\r\n", resultInMv);
}

/* [] END OF FILE */