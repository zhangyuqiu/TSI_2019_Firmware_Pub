/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _TSI_CONFIG    /* Guard against multiple inclusion */
#define _TSI_CONFIG

/*CAN bus TSI parameter*/
#define CAN_MESSAGE_SEND_ARRAY_LENGTH 10 // numbers of maximum CAN message to be transmitted that wait in queue
#define CAN_MESSAGE_SEND_BYTES 8 // Length of sending can message. Minimum 1, Maximum 8;

/*timer period*/
#define TIMER0_INT_PERIOD 30963 // int time = (TIMER0_INT_PERIOD/clk_feq) * prescaler. 78125 for 0.1s
#define TIMER0_INT_MULT 30 // final interrupt time = int time * TIMER0_INT_MULT.
#define TIMER1_INT_PERIOD 15625 // 0.04s * 2
#define TIMER1_INT_MULT 50

/*I2C TSI parameter*/
#define I2C_MESSAGE_SEND_ARRAY_LENGTH 10 // numbers of maximum I2C message to be transmitted that wait in queue
#define I2C_MESSAGE_SEND_BYTES 8 // Length of sending I2C message. Minimum 1, Maximum 8;
#define I2C_WRITE 0
#define I2C_READ 1
#define I2C_WRITE_READ 2

/*Flash memory config*/
/* Row size for MZ device is 2Kbytes */
//#define APP_DEVICE_ROW_SIZE_DIVIDED_BY_4        (DRV_FLASH_ROW_SIZE/4)
///* Page size for MZ device is 16Kbytes */
//#define APP_DEVICE_PAGE_SIZE_DIVIDED_BY_4       (DRV_FLASH_PAGE_SIZE/4)
#define APP_PROGRAM_FLASH_BASE_ADDRESS          (unsigned int) 0x9D008000
#define APP_PROGRAM_FLASH_BASE_ADDRESS_VALUE    (KVA0_TO_KVA1((unsigned int *) APP_PROGRAM_FLASH_BASE_ADDRESS))
#define CURRENT_LIMIT_ADDRESS 0x9D008000+4

/*Sampling frequency defines here*/ // All sampling are goes to TIMER1
uint8_t NCD9830CH0_freq = 2; // flowrate, 25 = 1s
uint8_t NCD9830CH1_freq = 2; // Cooling temp2
uint8_t NCD9830CH2_freq = 2; // Cooling temp1
uint8_t NCD9830CH3_freq = 2; // A1_HV
uint8_t NCD9830CH4_freq = 2; // A2_HV
uint8_t NCD9830CH5_freq = 2; // TSV voltage / HV +
uint8_t NCD9830CH6_freq = 2; // HV -
uint8_t NCD9830CH7_freq = 2; // MC_Voltage / MC+

#define FLOWRATE_CH 0;
#define COOLTEMP_2_CH 1;
#define COOLTEMP_1_CH 2;
#define A1_HV_CH 3;
#define A2_HV_CH 4;
#define TSV_VOLTAGE_CH 5;
#define HV_RTN_CH 6;
#define MC_VOLTAGE_CH 7;

uint8_t MCP23016GPB_freq = 30; // ThrottleP signal overall freq

uint8_t MCP23016GPB0_freq = 50; // ThrottleP signal
uint8_t MCP23016GPB1_freq = 50; // ThrottleP signal
uint8_t MCP23016GPB2_freq = 50; // ThrottleP signal
uint8_t MCP23016GPB3_freq = 50; // ThrottleP signal
uint8_t MCP23016GPB4_freq = 50; // ThrottleP signal
uint8_t MCP23016GPB5_freq = 50; // ThrottleP signal
uint8_t MCP23016GPB6_freq = 50; // ThrottleP signal
uint8_t MCP23016GPB7_freq = 50; // PC_ready, Prechare ready

/*Driver's state*/
#define OVER_CURR 10; // Overcurrent threshood



#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
