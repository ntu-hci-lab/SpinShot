/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  * SpinShot: Optimizing Both Physical and Perceived Force Feedback of Flywheel-Based, Directional Impact Handheld Devices
  * Author: Chia-An Fan et al.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "struct_typedef.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum Status{
    FREE=0, STABLE=1, ACCELERATE=2, WAIT=3, CALIBRATE=4, IMPACT=5, IMPACT_AFTER=6, TEST=7, NOT_CALIBRATED=8, ERR=9
};
enum Impact_phases{
    IMPACT_STANDBY=0, IMPACT_PHASE_1=1, IMPACT_PHASE_2=2, IMPACT_END=3
};
enum Command_Type {
    C_NOP, C_FREE, C_WAIT, C_ACCELERATE, C_IMPACT, C_CALIBRATE
};
enum ERR_TYPE {
    ENCODER_ERR, CONTROL_ERR, NO_ERR
};
typedef struct Command {
    enum Command_Type type;
    int values[3];
} Command;

typedef struct Command_Queue {
    int size;
    int capacity;
    int head;
    int tail;
    int temp_size;  // for cancellable enqueue
    int temp_tail;  // for cancellable enqueue
    Command * elements;
    Command null_command;
} Command_Queue;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
Command_Queue * Command_Queue_Init(void);
bool_t Enqueue_Command(Command_Queue *, enum Command_Type, int[]);
void Confirm_Enqueue(Command_Queue *);
void Rollback_Enqueue(Command_Queue *);
Command Peek_Command(Command_Queue *);
Command Dequeue_Command(Command_Queue *);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
