/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits,
    and allocates any necessary global system resources, such as the
    sysObj structure that contains the object handles to all the MPLAB Harmony
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "../../../TSIconfig.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_LEVEL_3
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_LEVEL_3
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    OFF
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_1
#pragma config FPLLRNG =    RANGE_5_10_MHZ
#pragma config FPLLICLK =   PLL_FRC
#pragma config FPLLMULT =   MUL_50
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     ON
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    ON
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xffff
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_I2C Initialization Data">
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USART Initialization Data">
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());

    /* Initialize Drivers */
    DRV_I2C0_Initialize();


    /* Initialize ADC */
    DRV_ADC_Initialize();

    /* Initialize CAN Driver 0 */
    DRV_CAN0_Initialize();

    sysObj.drvFlash0 = DRV_FLASH_Initialize(DRV_FLASH_INDEX_0, (SYS_MODULE_INIT *)NULL);
    /*Initialize TMR0 */
    DRV_TMR0_Initialize();
    /*Initialize TMR1 */
    DRV_TMR1_Initialize();
    /*Initialize TMR2 */
    DRV_TMR2_Initialize();

    sysObj.drvUsart0 = DRV_USART_Initialize(DRV_USART_INDEX_0, (SYS_MODULE_INIT *)NULL);
 
    /* Initialize System Services */
    SYS_PORTS_Initialize();

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();
    
    DRV_TMR0_PeriodValueSet(TIMER0_INT_PERIOD);
    DRV_TMR1_PeriodValueSet(TIMER1_INT_PERIOD);
    DRV_TMR2_PeriodValueSet(TIMER0_INT_PERIOD+2000);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_3);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_5);
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    
    /*Setup the INT_SOURCE_EXTERNAL_1 and Enable it*/
    SYS_INT_VectorPrioritySet(INT_VECTOR_INT1, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_INT1, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE1,INT_EDGE_TRIGGER_FALLING);
    SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_1);

    /* Initialize Middleware */

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_UART_Initialize();
    APP_CAN_Initialize();
    APP_DRIVER_STATE_FSM_Initialize();
    APP_I2C_Initialize();
    APP_DATAIO_Initialize();
    APP_FLASH_Initialize();
    APP_DRV_BTN_Initialize();
    
    TRISBbits.TRISB3 = 0; // LEDs
    ANSELBbits.ANSB3 = 0;
    TRISBbits.TRISB2 = 0;
    ANSELBbits.ANSB2 = 0;
    TRISBbits.TRISB4 = 0;
    ANSELBbits.ANSB4 = 0;
    
    TRISBbits.TRISB15 = 0; // Testing RB15 pin 30
    ANSELBbits.ANSB15 = 0;
    
    
    TRISGbits.TRISG6 = 1; // Safety loop input
    ANSELGbits.ANSG6 = 0;
//    CNPDGbits.CNPDG6 = 0;
//    CNPUGbits.CNPUG6 = 1;
    TRISEbits.TRISE7 = 1; // IMD input
    ANSELEbits.ANSE7 = 0;
    CNPDEbits.CNPDE7 = 1;
    TRISEbits.TRISE4 = 1; // BreakPressed input
    ANSELEbits.ANSE4 = 0;
    CNPUEbits.CNPUE4 = 1;
    TRISBbits.TRISB8 = 0; // Throttle_sel output
    ANSELBbits.ANSB8 = 0;
    TRISBbits.TRISB9 = 0; // Cooling relay control output
    ANSELBbits.ANSB9 = 0;
    
    TRISDbits.TRISD0 = 0; // Driver led output
//    ANSELDbits.ANSD0 = 0;
    
    TRISEbits.TRISE0 = 0; // RTDS CTRL output
//    ANSELEbits.ANSE0 = 0;
        
    TRISDbits.TRISD11 = 0; // Spare_LED output
//    ANSELEbits.ANSE0 = 0;
    
    TRISEbits.TRISE1 = 1; // Driver Button input
    CNPDEbits.CNPDE1 = 0;

//    TRISDbits.TRISD1 = 1;
//    CNPDDbits.CNPDD1 = 1;
    
//    TRISBbits.TRISB12 = 1; // app2s_b paddle voltage;
//    ANSELBbits.ANSB12 = 1;
//    TRISBbits.TRISB13 = 1; // app2s_b paddle voltage;
//    ANSELBbits.ANSB13 = 1;
    
//    TRISBbits.TRISB14 = 1; // AN9
//    ANSELBbits.ANSB14 = 1;
//    TRISBbits.TRISB12 = 1; // AN7
//    ANSELBbits.ANSB12 = 1;
//    TRISEbits.TRISE2 = 0;
    
}


/*******************************************************************************
 End of File
*/

