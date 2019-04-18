//DOM-IGNORE-END

#ifndef _APP_DRIVER_STATE_FSM_H
#define _APP_DRIVER_STATE_FSM_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

typedef enum
{
	/* Application's state machine's initial state. */
	APP_DRIVER_STATE_FSM_STATE_INIT=0,
	IDLE,
    PRECHARGE,
    DRIVE_SETUP,
    OVERCURRENT,
    DRIVE,
    DRIVE_RTDS,
    DRIVE_RETURN,

} APP_DRIVER_STATE_FSM_STATES;

typedef struct
{
    /* The application's current state */
    APP_DRIVER_STATE_FSM_STATES state;

} APP_DRIVER_STATE_FSM_DATA;

/*FSM conditions*/
//typedef struct {
//    uint8_t buttonPushed;
//    uint8_t brakePressed;
//    uint8_t safetyLoopClosed;
//    uint8_t throttleImplausibility;
//    uint8_t throttleLessThan; // throttle < 0.5v
//    uint8_t prechargeComplete;
//    uint8_t overCurrent;
//    uint8_t airsOpen;
//    uint8_t throttleControl;
//} driver_state_fsm_conditions;

void APP_DRIVER_STATE_FSM_Initialize ( void );

void APP_DRIVER_STATE_FSM_Tasks( void );

int saftyLoopClosed,PC_Ready, DriverButton_Pushed, throttleImplausibility, brakePressed, overCurr, airsOpen, MC_Activate, saftyloop, throttlegreaterthan, throttlepressed;
int BUZZER_CTRL, D_LED_CTRL, Throttle_SEL, Cooling_Relay_CTRL, Spare_LED_CTRL;
int throttle, boardtemp,CoolTemp_1,CoolTemp_2,FlowRate,a1,a2;
void update_value();
int states;
int buzz_flag;
volatile uint32_t send_condition_flag; // triggered by interrupt when timer count reach. Send current state and condition to VSCADA
volatile uint32_t buzz_timer_flag;
int DriveSetUpDriveBtnRelease;


#endif /* _APP_DRIVER_STATE_FSM_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */