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
* (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"

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

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    UART_context;           /** UART context */
static mtb_hal_uart_t               UART_hal_obj;           /** Debug UART HAL object  */

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

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&UART_hal_obj, &UART_hal_config, &UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

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