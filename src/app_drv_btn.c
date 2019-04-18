/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_drv_btn.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "app_drv_btn.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DRV_BTN_DATA app_drv_btnData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

uint32_t drive_btn_timer;
uint32_t drive_btn_releasing_timer;


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_DRV_BTN_Initialize ( void )

  Remarks:
    See prototype in app_drv_btn.h.
 */

void APP_DRV_BTN_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_drv_btnData.state = APP_DRV_BTN_STATE_INIT;
    drive_btn_timer = 0;
    drive_btn_pushing = 0;
    drive_btn_releasing_timer = 0;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_DRV_BTN_Tasks ( void )

  Remarks:
    See prototype in app_drv_btn.h.
 */

void APP_DRV_BTN_Tasks ( void )
{
//    PORTBbits.RB4 = drive_btn_pushing;
    /* Check the application's current state. */
    switch ( app_drv_btnData.state )
    {
        /* Application's initial state. */
        case APP_DRV_BTN_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                app_drv_btnData.state = RELEASED;
            }
            break;
        }

        case RELEASED:
        {
            drive_btn_pushing = 0;
            
            if (driver_btn_timer_flag && PORTEbits.RE1 == 0) {
                drive_btn_timer++;
                driver_btn_timer_flag = 0;
                if (drive_btn_timer > 2) {
                    app_drv_btnData.state = PUSHING;
//                    PORTBbits.RB4 = ~PORTBbits.RB4;
                    drive_btn_timer = 0;
                }
            }
            else if (PORTEbits.RE1 == 1) {
                drive_btn_pushing = 0;
                drive_btn_timer = 0;
            }
            
            break;
        }
        
        case PUSHING:
        {
            drive_btn_pushing = 1;
            
            if (PORTEbits.RE1 == 1) {
                app_drv_btnData.state = RELEASING;
            }
            
            break;
        }
        
        case RELEASING:
        {
            drive_btn_pushing = 0;
            
            if (driver_btn_timer_flag) {
                driver_btn_timer_flag = 0;
                drive_btn_releasing_timer++;
                if (drive_btn_releasing_timer > 10) {
                    app_drv_btnData.state = RELEASED;
                    drive_btn_releasing_timer = 0;
                } 
            }
            
            break;
        }
        
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
