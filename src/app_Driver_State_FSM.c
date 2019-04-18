/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_driver_state_fsm.c

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

#include "app_driver_state_fsm.h"

APP_DRIVER_STATE_FSM_DATA app_driver_state_fsmData;
int reading_mc_11; // MC+
int reading_mc_22; // TSV
uint32_t send_cond_timer; // timer for sending conditions
uint32_t buzz_timer;
uint32_t buzz_sound_count;


void APP_DRIVER_STATE_FSM_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_driver_state_fsmData.state = APP_DRIVER_STATE_FSM_STATE_INIT;
    send_cond_timer = 0;
    buzz_timer = 0;
    buzz_sound_count = 0;
    DriveSetUpDriveBtnRelease = 0;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


void APP_DRIVER_STATE_FSM_Tasks ( void )
{
    update_value();
    
    if (send_condition_flag) {
        send_condition_flag = 0;
        send_cond_timer++;
//        buzz_timer++;
        if (send_cond_timer == 10) {
            send_conditions();
            send_cond_timer = 0;
            PORTBbits.RB2 = ~PORTBbits.RB2;
        }
//        if (buzz_timer > 10) {
////            uint8_t a = PORTBbits.RB15;
////            PORTBbits.RB15 = ~PORTBbits.RB15;
//            buzz_timer = 0;
//            if (app_driver_state_fsmData.state == DRIVE_RTDS && buzz_sound_count < 6) {
//                buzz_sound_count++;
//                PORTBbits.RB15 = ~PORTBbits.RB15;
//            }
//        }
        
    }
    
    /* Check the application's current state. */
    switch ( app_driver_state_fsmData.state )
    {
        /* Application's initial state. */
        case APP_DRIVER_STATE_FSM_STATE_INIT:
        {
            states = 0x0;
            bool appInitialized = true;
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;   
            if (appInitialized)
            {            
                app_driver_state_fsmData.state = IDLE;
            }
            break;
        }

        case IDLE:
        {
            states = 0x1;
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;
            if(saftyloop){
                app_driver_state_fsmData.state = PRECHARGE;
            }
            break;
        }
        
        case PRECHARGE:
        {
            states = 0x2;
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=0;
            Spare_LED_CTRL = 0;
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(MC_Activate){
                app_driver_state_fsmData.state = DRIVE_SETUP;
            }
            break;
        }
        
        case DRIVE_SETUP: 
        {
            states = 0x3;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=1;
            D_LED_CTRL = 0;
            Spare_LED_CTRL = 0;
//            buzz_sound_count = 0; // reset buzzer sound time counter
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(drive_btn_pushing & !throttleImplausibility & brakePressed & MC_Activate){
                app_driver_state_fsmData.state = DRIVE_RTDS;
            }
            
            break;
        }
        
        case DRIVE_RTDS:
        {
            states = 0x4;
            PORTEbits.RE0 = 1;
            if (buzz_timer_flag) {
                buzz_timer_flag = 0;
                buzz_timer++;
                if (buzz_timer > 10) {
                    buzz_timer = 0;
                    if (buzz_sound_count < 10) {
                        buzz_sound_count++;
                        PORTEbits.RE0 = 1;
                    }
                }
            }
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            }
            else if (buzz_sound_count >= 10) {
                PORTEbits.RE0 = 0;
                if (!drive_btn_pushing && brakePressed) {
                    app_driver_state_fsmData.state = DRIVE;
                }
            }
            break;
        }
        
        case DRIVE:
        {
            buzz_sound_count = 0;
            states = 0x5;
            D_LED_CTRL=1;
            Throttle_SEL=1;
//            Cooling_Relay_CTRL=1;
//            BUZZER_CTRL=~buzz_flag;
            Spare_LED_CTRL = 1;
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(throttleImplausibility | (throttlepressed & brakePressed) | !MC_Activate | (drive_btn_pushing & brakePressed)){
                app_driver_state_fsmData.state = DRIVE_RETURN;
            } else if(overCurr){
                app_driver_state_fsmData.state = OVERCURRENT;
            }
            
            break;
        }
        
        case OVERCURRENT:
        {
            states = 0x6;
            //D_LED_CTRL flash
//            BUZZER_CTRL=0;
            D_LED_CTRL=0;
            Throttle_SEL=0;
            Cooling_Relay_CTRL=1;
            Spare_LED_CTRL = 0;
            if(saftyloop == 0){
                app_driver_state_fsmData.state = IDLE;
            } else if(!overCurr & throttlegreaterthan){
                app_driver_state_fsmData.state = DRIVE;
            }
            break;
        }
        
        case DRIVE_RETURN: 
        {
            states = 0x8;
            if (!drive_btn_pushing) {
                app_driver_state_fsmData.state = DRIVE_SETUP;
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


void update_value() {
    int MCP23016B_reading = get_MCP23016B();
    
    PC_Ready = ((MCP23016B_reading & 0x80) == 0x80); 
    PC_Ready = 1;
    
//    throttleImplausibility = !((MCP23016B_reading & 0x3F) == 0x3F);
    throttleImplausibility = ((MCP23016B_reading & 0b01000000) == 0b01000000);
    
    overCurr = over_current();
    
    saftyloop = !PORTGbits.RG6;
//    saftyloop = get_ADCCh(45)/0x10;
    
//    DriverButton_Pushed = PORTDbits.RD1;
    
    brakePressed = !PORTEbits.RE4;
    
//    MC_Activate = mc_active();
    MC_Activate = 1;
    
    reading_mc_11 = get_NCD9830_reading(7);
    reading_mc_22 = get_NCD9830_reading(5);
    
    PORTDbits.RD0 = D_LED_CTRL;
    PORTBbits.RB8 = Throttle_SEL;
    PORTBbits.RB9 = Cooling_Relay_CTRL;
    PORTDbits.RD11 = Spare_LED_CTRL;
    
    throttle = get_ADCCh(0)/0x10;

//    FlowRate = get_NCD9830_reading(0);
//    throttle = get_ADCCh(14)/0x10;
    
    throttlegreaterthan = throttle > 0x26; // 0.8v
    throttlepressed = throttle > 0x50; // 1.2v
    
    a1 = get_NCD9830_reading(3);
    a2 = get_NCD9830_reading(4);
    
    // for testing
//    PORTBbits.RB3 = DriverButton_Pushed;
}

int over_current() {
    int reading_1 = get_NCD9830_reading(3);
    int reading_2 = get_NCD9830_reading(4);
    return 0;
}

int mc_active() {
    int reading_mc_1 = get_NCD9830_reading(7); // MC+
    int reading_mc_2 = get_NCD9830_reading(5); // TSV
    
    if ((reading_mc_1 > (reading_mc_2 - 10) )) return 1;
    else return 0;
}

int get_pad_voltage() {
    return get_ADCCh(8);
    
}

int return_Throttp() {
    return (int)throttleImplausibility;
}

void send_conditions() {
    uint8_t conditions = throttlepressed + (throttleImplausibility*2) + (overCurr*4) + (saftyloop*8) + (DriverButton_Pushed*16) + 
                         (brakePressed*32) + (MC_Activate*64) + (throttlegreaterthan*128);
//    boardtemp = get_ADCCh(9);
//    CoolTemp_1 = get_ADCCh(1)/0x10;
//    CoolTemp_2 = get_ADCCh(2)/0x10;
    can_send_bytes(0x300,get_ADCCh(0)/0x10,get_ADCCh(1)/0x10,get_ADCCh(2)/0x10,get_ADCCh(4)/0x10,3,get_ADCCh(9),rand() % 0xFF,rand() % 0xFF);
//    can_send_bytes(0x300,1,1,2,2,3,3,4,4);
//    can_send_bytes(0x300,states,throttleImplausibility,throttlepressed,saftyloop,drive_btn_pushing,brakePressed,PORTEbits.RE0,throttlegreaterthan);
//    can_send_bytes(0x400,conditions,reading_mc_11,reading_mc_22,CoolTemp_1,CoolTemp_2,FlowRate,states,(a2-a1));
//    rand() % 0xFF
//    can_send_bytes(0x400,a1,a2,reading_mc_22,CoolTemp_1,CoolTemp_2,FlowRate,states,rand() % 0xFF);
}
/*******************************************************************************
 End of File
 */
