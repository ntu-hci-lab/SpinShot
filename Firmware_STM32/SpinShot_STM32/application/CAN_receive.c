/**
  ******************************************************************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0          
  *
  @verbatim
  ==============================================================================
	* @attention
  *
	* The program is referred from :
	* https://github.com/RoboMaster/Development-Board-C-Examples/tree/master/14.CAN
  *
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "CAN_receive.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern enum Status status;
extern enum Impact_phases impact_phases;

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/* motor data of the rm 2006 motor*/
static motor_measure_t motor_data_point;
		
static CAN_TxHeaderTypeDef  tx_message;
static uint8_t              can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		if(rx_header.StdId == CAN_M1_ID) {
			get_motor_measure(&motor_data_point, rx_data);
		}
		else {
			uint8_t err_msg[] = "please make sure your ESC's ID is 1";
			//CDC_Transmit_FS((uint8_t*)&err_msg, sizeof(err_msg));
    }
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_reset_ID(void)
{
    uint32_t send_mail_box;
    tx_message.StdId = 0x700;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx_message, can_send_data, &send_mail_box);
}

void CAN_cmd_current(int16_t current) {
    uint32_t send_mail_box;
    tx_message.StdId = CAN_CHASSIS_ALL_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = current >> 8;
    can_send_data[1] = current;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
const motor_measure_t *get_motor_measure_point(void)
{
    return &motor_data_point;
}

