/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#include "system/common/sys_common.h"
#include "app_uart.h"
#include "app_can.h"
#include "app_driver_state_fsm.h"
#include "app_i2c.h"
#include "app_dataio.h"
#include "app_flash.h"
#include "app_drv_btn.h"
#include "system_definitions.h"
uint32_t count;
uint32_t timer1Count = 0;
uint32_t timer2Count = 0;
uint32_t timer3Count = 0;
uint32_t buzz_Count = 0;
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

 

void __ISR(_I2C4_MASTER_VECTOR, ipl2AUTO) _IntHandlerDrvI2CMasterInstance0(void)
{
    uint32_t A4 = PORTBbits.RB4;
    PORTBbits.RB4 = ~A4;
	DRV_I2C0_Tasks();
}


void __ISR(_I2C4_BUS_VECTOR, ipl2AUTO) _IntHandlerDrvI2CErrorInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_I2C_4_BUS);
}

void __ISR(_TIMER_1_VECTOR, ipl1AUTO) IntHandlerDrvTmrInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_1);
    timer1Count++;
    if (timer1Count == 2) {
        set_MCP23016_READ_flag(0);

    }
    if (timer1Count == 3) {
        set_NCD9830_READ_flag(0);
        set_NCD9830_READ_flag(1);
        set_NCD9830_READ_flag(2);
        set_NCD9830_READ_flag(3);
        set_NCD9830_READ_flag(4);
        set_NCD9830_READ_flag(5);
        set_NCD9830_READ_flag(6);
        set_NCD9830_READ_flag(7);
        timer1Count = 0;
    }
    
    
}
void __ISR(_TIMER_2_VECTOR, ipl3AUTO) IntHandlerDrvTmrInstance1(void)
{
//    if (timer3Count > 20) {
    send_condition_flag = 1;
    driver_btn_timer_flag = 1;
    buzz_timer_flag = 1;
//    can_send_bytes(0x300,1,1,2,2,3,3,4,rand() % 0xFF);
////
////        uint32_t B3 = PORTBbits.RB3;
////        PORTBbits.RB3 = ~B3;
//        timer3Count = 0;
//    }
//    if (states == 0x4 && buzz_flag == 0 ) {
//        if (buzz_Count < 100) {
//            buzz_Count ++;
//        }
//        else {
//            buzz_flag = 1;
//        }
//    }
//    if (states != 0x4) {
//        buzz_flag = 0;
//        buzz_Count = 0;
//    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
void __ISR(_TIMER_4_VECTOR, ipl3AUTO) IntHandlerDrvTmrInstance2(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}
 
void __ISR(_CAN1_VECTOR, IPL1AUTO) _IntHandlerDrvCANInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_CAN_1);
    
    if((PLIB_CAN_ModuleEventGet(CAN_ID_1) & (CAN_RX_EVENT)) != 0)
    {
       if(DRV_CAN0_ChannelMessageReceive(CAN_CHANNEL1, 0x201, 4, &canrecievemessage[0])) {
           can_send_bytes(0x300,canrecievemessage[0],canrecievemessage[1],2,2,3,3,canrecievemessage[0],rand() % 0xFF);
           overCurr_intr_flag = canrecievemessage[0];
       };
    }
    
}
void __ISR(_EXTERNAL_1_VECTOR, IPL1AUTO) _IntHandlerExternalInterruptInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
}


 /*******************************************************************************
 End of File
*/
