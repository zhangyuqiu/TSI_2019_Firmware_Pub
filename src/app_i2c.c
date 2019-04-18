/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_i2c.c

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
#include "app_i2c.h"
#include "TSIconfig.h"


APP_I2C_DATA app_i2cData;
// This is the string to write to the slave device.  Be aware that the double quotes adds a null byte at the end of the string.
// So, writing "Hello World!" actually transmits 13 bytes.
uint8_t app_i2cWriteString[] = "Hello World";
const I2C_SLAVE_ADDRESS_VALUE app_i2cSlaveAddress = 0x40;

uint8_t appWriteString[1] = {2};
uint8_t appReadString[4];

SINGLE_I2C_TASK task_array[(I2C_MESSAGE_SEND_ARRAY_LENGTH)];
uint8_t array_count_i2c; // from 0 to CAN_MESSAGE_SEND_ARRAY_LENGTH;
uint8_t current_ptr_i2c; // from 0 to CAN_MESSAGE_SEND_ARRAY_LENGTH;


/* Application's i2c Setup Function */
static void I2C_Setup( void )
{
    app_i2cData.i2cStates = APP_I2C_I2C_START;
}

 /*Globe functions */

/*  Send array in bytes form
 *  1 will be returned if can message is added successfully into send array;
 *  0 will be returned if can message is not added.
 */
int i2c_do_task(uint8_t addr, uint8_t type, uint8_t senddatalength, uint8_t recievedatalength, 
                uint8_t * recieve_message, uint8_t * message_array) {
    
    if (array_count_i2c < I2C_MESSAGE_SEND_ARRAY_LENGTH) { // available place in can send array
        
        SINGLE_I2C_TASK thisTask;
        thisTask.addr = addr;
        thisTask.type = type;
        thisTask.senddatalength = senddatalength;
        thisTask.recievedatalength = recievedatalength;
        thisTask.recieve_message = recieve_message;
        thisTask.message_array = message_array;
        
        task_array[array_count_i2c] = thisTask; // store array value
        array_count_i2c++; // increment to be sent array
        return 1;
    }
    else {
        return 0;
    }
}
/******************************************************************************
  Function:
    static void APP_I2C_I2C_Task (void)
    
   Remarks:
    Allows a polled state machine to manage i2c testing.

*/
static void APP_I2C_I2C_Task (void)
{		
	switch(app_i2cData.i2cStates)
	{
		default:
		
		case APP_I2C_I2C_START:
        {
            // initalize pointers
            // Switch to the Transmit State.
            app_i2cData.i2cStates = APP_I2C_I2C_WAIT;
                      
//            char towrite[5];
//            sprintf(towrite,"%d",current_ptr_i2c);
//            uart_write_string(towrite);
        
			break;
		}	
        case APP_I2C_I2C_WAIT:
        {
            
            
            
            if (current_ptr_i2c < array_count_i2c) {// messages are added to send array
//                char towrite[5];
//                sprintf(towrite,"%d",array_count_i2c);
//                uart_write_string(towrite);
                
                app_i2cData.i2cStates = APP_I2C_I2C_TRANSMIT;

            }
            else {
                array_count_i2c = 0;
                current_ptr_i2c = 0;
                
            }
            break;
        }
		case APP_I2C_I2C_TRANSMIT:
        {
                
                switch (task_array[current_ptr_i2c].type) {
                    case I2C_WRITE: {
                        app_i2cData.I2CBufferHandle = DRV_I2C_Transmit (app_i2cData.handleI2C0,
                                                                        task_array[current_ptr_i2c].addr,
                                                                        task_array[current_ptr_i2c].message_array,
                                                                        task_array[current_ptr_i2c].senddatalength, 
                                                                        NULL);
                        app_i2cData.i2cStates = APP_I2C_I2C_PROCESS;
                        current_ptr_i2c++;
                        break;
                    }
                    case I2C_READ: {
                        
                        app_i2cData.I2CBufferHandle = DRV_I2C_Receive (app_i2cData.handleI2C0,
                                                task_array[current_ptr_i2c].addr,
                                                task_array[current_ptr_i2c].recieve_message,
                                                task_array[current_ptr_i2c].recievedatalength,
                                                NULL);
                        app_i2cData.i2cStates = APP_I2C_I2C_PROCESS;
                        current_ptr_i2c++;
                        break;
                    }
                    case I2C_WRITE_READ: {
                        app_i2cData.I2CBufferHandle = DRV_I2C_TransmitThenReceive (
                                                app_i2cData.handleI2C0,
                                                task_array[current_ptr_i2c].addr,
                                                task_array[current_ptr_i2c].message_array,
                                                task_array[current_ptr_i2c].senddatalength, 
                                                task_array[current_ptr_i2c].recieve_message,
                                                task_array[current_ptr_i2c].recievedatalength,
                                                NULL);
                        app_i2cData.i2cStates = APP_I2C_I2C_PROCESS;
                        current_ptr_i2c++;
                        break;
                    }
                    default: {
                        break;
                    }
                }
                
                
            break;
		}
        case APP_I2C_I2C_PROCESS: {
            app_i2cData.I2CBufferEvent = DRV_I2C_TransferStatusGet ( app_i2cData.handleI2C0,
                     app_i2cData.I2CBufferHandle );
            if(app_i2cData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_COMPLETE) 
                {
                    //BSP_LEDToggle(BSP_LED_3);
                    app_i2cData.i2cStates = APP_I2C_I2C_DONE;
                }
            if(app_i2cData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_ERROR)
                {
                    app_i2cData.i2cStates = APP_I2C_I2C_ERROR;
                }
            break;
        }
		case APP_I2C_I2C_DONE:
		{
            app_i2cData.i2cStates = APP_I2C_I2C_WAIT;
			break;
        }
        case APP_I2C_I2C_ERROR:
		{
            app_i2cData.i2cStates = APP_I2C_I2C_WAIT;
            break;
        }

    }
}

void APP_I2C_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_i2cData.state = APP_I2C_STATE_INIT;

    app_i2cData.handleI2C0 = DRV_HANDLE_INVALID;
    array_count_i2c = 0;
    current_ptr_i2c = 0;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


void APP_I2C_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_i2cData.state )
    {
        /* Application's initial state. */
        case APP_I2C_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (app_i2cData.handleI2C0 == DRV_HANDLE_INVALID)
            {
                app_i2cData.handleI2C0 = DRV_I2C_Open(APP_I2C_DRV_I2C_INDEX, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != app_i2cData.handleI2C0 );
            }
        
            if (appInitialized)
            {
                I2C_Setup();
                // do i2c task here
//                i2c_do_task(0x20, I2C_READ, 0, 4, NULL, NULL);
                
                app_i2cData.state = APP_I2C_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_I2C_STATE_SERVICE_TASKS:
        {
		    /* Run the state machine for servicing I2C */
            APP_I2C_I2C_Task();
        
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



//void i2c_send(uint8_t addr, uint8_t * tosend, uint8_t size) {
//    DRV_I2C_BUFFER_HANDLE bf = DRV_I2C_Transmit (app_i2cData.handleI2C0,
//                        addr,
//                        tosend,
//                        size,
//                        NULL);
//    if(DRV_I2C_TransferStatusGet(app_i2cData.handleI2C0,bf) == DRV_I2C_BUFFER_EVENT_COMPLETE) 
//       {
//           BSP_LEDToggle(BSP_LED_3);
//       }
//}
//
//void i2c_send_recieve(uint8_t addr, uint8_t * tosend, uint8_t size) {
//    DRV_I2C_BUFFER_HANDLE bf = DRV_I2C_TransmitThenReceive (app_i2cData.handleI2C0,
//                            addr,
//                            tosend,
//                            size, 
//                            appReadString,
//                            sizeof(appReadString),
//                            NULL);
//}
/*******************************************************************************
 End of File
 */
