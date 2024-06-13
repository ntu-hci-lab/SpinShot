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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_M1_ID = 0x201,
    CAN_M2_ID = 0x202,
    CAN_M3_ID = 0x203,
    CAN_M4_ID = 0x204,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/**
  * @brief          send CAN packet of ID 0x700, it will set motor 2006 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_reset_ID(void);

/**
  * @brief          send control current of motor (0x201)
  * @param[in]      motor 0x201: 2006 motor control current, range [-16384,16384] 
  * @retval         none
  */
extern void CAN_cmd_current(int16_t current);

/**
  * @brief          return the 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
extern const motor_measure_t *get_motor_measure_point(void);

extern const bool_t get_round_flip_flop_value(void);
#endif
