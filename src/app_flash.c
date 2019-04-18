/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_flash.c

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

#include "app_flash.h"

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

APP_FLASH_DATA app_flashData;
uint8_t result8[4];

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
//
//uint32_t ToWrite = 0xBBAA;    
//uint32_t init_flagbytes= 0xAA; 

uint32_t storeflag; // read from flash address to see if the program already store values for overcurrent
uint8_t storeflagbyte;
uint8_t overwriteflagbyte;
/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_FLASH_Initialize ( void )

  Remarks:
    See prototype in app_flash.h.
 */

void APP_FLASH_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_flashData.state = APP_FLASH_STATE_INIT;
    DRV_HANDLE flashHandle = DRV_FLASH_Open(DRV_FLASH_INDEX_0, intent);
    overCurr_toWrite = 0xAACD;
    overCurr_intr_flag = 0;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_FLASH_Tasks ( void )

  Remarks:
    See prototype in app_flash.h.
 */

void APP_FLASH_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_flashData.state )
    {
        /* Application's initial state. */
        case APP_FLASH_STATE_INIT:
        {
            bool appInitialized = true;
            update_flag_byte_from_flash();
            
            if (appInitialized)
            {
//                if (storeflagbyte != 0xAA) {
//                    app_flashData.state = APP_FLASH_WRITE_INIT_VALUE;
//                }
//                else {
//                    app_flashData.state = APP_FLASH_LISTENING;
//                }
                app_flashData.state = APP_FLASH_LISTENING;
            }
            break;
        }
        
        case APP_FLASH_WRITE_INIT_VALUE: {
            update_flag_byte_from_flash();
            if(!DRV_FLASH_IsBusy(flashHandle)){
                DRV_FLASH_WriteWord(flashHandle, APP_PROGRAM_FLASH_BASE_ADDRESS,overCurr_toWrite);
                 app_flashData.state = APP_FLASH_LISTENING;
            }
            break;
        }
        
        case APP_FLASH_LISTENING: {
            if (overCurr_intr_flag == 1) {
                overCurr_intr_flag = 0;
                DRV_FLASH0_ErasePage(APP_PROGRAM_FLASH_BASE_ADDRESS);
            }
            else if (overCurr_intr_flag == 2) {
                overCurr_intr_flag = 0;
//                overCurr_toWrite = (rand() % 0xFF);
                overCurr_toWrite = canrecievemessage[1];
                if(!DRV_FLASH_IsBusy(flashHandle)){
                    DRV_FLASH_WriteWord(flashHandle, APP_PROGRAM_FLASH_BASE_ADDRESS,overCurr_toWrite);
                }
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

int update_flag_byte_from_flash() {
    storeflag = *(uint32_t*)(APP_PROGRAM_FLASH_BASE_ADDRESS); // read from flash address to see if the program already store values for overcurrent
    storeflagbyte = (storeflag & 0x0000ff00) >> 8;
    overCurr_val = (storeflag & 0x000000ff);
}
 

/*******************************************************************************
 End of File
 */
