/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for controlling a motor with various states
  ******************************************************************************
  * Project Name: SpinShot 
  * Author: Chia-An Fan et al.
  * Conference: UIST2024
  ******************************************************************************
  * Supplementary Information:
  * - Upon receiving messages from the serial interface, commands are parsed in the
  *   usbd_cdc_if.c file and enqueued in the command queue for execution.
  * - Definitions for Status, Command_Type, and other key configurations are provided
  *   in main.h, centralizing important data types and enums.
  * - PID control operations are handled through functions defined in pid.c, ensuring
  *   precise control over motor behavior based on dynamic feedback.
  * - Motor status updates are managed via the CAN bus, with detailed handling and
  *   parsing implemented in CAN_receive.c to ensure timely updates to system states.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "usbd_cdc_if.h"
#include "pid.h"
#include <stdlib.h>

//#include "commend_processor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5600_RAW_ADDR 0x36
#define AS5600_ADDR (AS5600_RAW_ADDR << 1)
#define AS5600_RAW_ANGLE_REGISTER 0x0C

#define VALUE_ANGLE_15 170
#define VALUE_ANGLE_180 2048

#define MINIM_LATENCY_FACTOR 0.24

void Print_Rpm(void);
void Print_Angle(void);
void Print(void);
void Reset_Angle(void);
void Read_And_Calculate_Angle(void);
void Process_Commend(void);
void Brake_Event_Start(void);
bool_t Brake_Event(void);
bool_t IsReadyToBrake(void);
bool_t IsAboutToCollide(void);
bool_t IsCollided(void);
void Calibrate_Angle();
uint16_t Read_Raw_Angle(void);
//tool
void get_dec_str(uint8_t*, size_t, int32_t, bool_t);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Command_Queue *command_queue;      // Queue for managing incoming commands

/* Enumerated states for different phases of operation */
enum Status status = NOT_CALIBRATED;
enum Impact_phases impact_phases = IMPACT_STANDBY;
enum ERR_TYPE err_type = NO_ERR;

/* Variables to store angles and their base calibration */
static float angle = 0;
static float base_raw_angle = 0;
uint8_t angle_err_counter = 0;

/* Timer variables */
static uint32_t brake_start_time;
static uint32_t last_print_time;

/* Control variables */
int16_t impact_mode;
int16_t target_rpm;
int16_t accelerate_current;
int16_t shift_angle;
PID_TypeDef motor_pid;

/* Message and logging variables */
int16_t message_type = 0;
float last_angle = 0;

/* Runtime operational parameters */
static int16_t current_rpm = 0;
static int16_t current_torque = 0;
bool_t impact_direction = 0;

/* Configuration parameters for device operation */
uint32_t print_freq = 10; // Frequency of status prints in milliseconds
uint32_t minimum_impact_rpm = 150;  // Minimum RPM for impact

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USB_DEVICE_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_I2C2_Init();
    
    /* USER CODE BEGIN 2 */
	// Initial setup for timers and PWM
	last_print_time = HAL_GetTick();
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	can_filter_init();
	pid_init(&motor_pid);
	//motor_pid.f_param_init(&motor_pid,PID_Speed,16384,5000,10,0,8000,0,1.5,0.1,0); // another PID parameter set
	motor_pid.f_param_init(&motor_pid,PID_Speed,16384,5000,10,0,8000,0,3,0.06,0.06); 
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	command_queue = Command_Queue_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Read and process motor speed and commands
		current_rpm = (get_motor_measure_point())->speed_rpm / 36;
        current_torque = (get_motor_measure_point())->given_current;
		Process_Commend();
        
		Read_And_Calculate_Angle();
        
        // Logic for different operational states
		if(status == STABLE || (status == IMPACT && impact_phases == IMPACT_PHASE_1)) {
			motor_pid.target = target_rpm * 36;
			motor_pid.f_cal_pid(&motor_pid, (get_motor_measure_point())->speed_rpm);
			CAN_cmd_current(motor_pid.output);
		}
		if(status == FREE || status == NOT_CALIBRATED)
		{
			CAN_cmd_current(0);  /* no current */
		}
		else if(status == ACCELERATE){
			if(current_rpm < target_rpm - 10) CAN_cmd_current(accelerate_current);
			else if(current_rpm > target_rpm + 10) CAN_cmd_current(-accelerate_current);
			else {
				status = STABLE;
			}
		}
        else if(status == WAIT) {
            if((current_rpm > target_rpm - 10) || (current_rpm < target_rpm + 10)) 
                status = STABLE;
        }
		else if(status == IMPACT){
			if(((current_rpm > minimum_impact_rpm) || (-current_rpm > minimum_impact_rpm)) && Brake_Event()){
                status = FREE;
			}
		}
        else if(status == IMPACT_AFTER) {
            CAN_cmd_current(0);
            if(current_rpm < 10 || current_rpm > -10)
                status = FREE;
        }
		else if(status == CALIBRATE){
			Calibrate_Angle();
			status = FREE;
		}
        else if(status == ERR) {
            CAN_cmd_current(0); // Turn off the motor
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);  // Deactivate the solenoid      
        }
		Print();
        
		// Safety check for solenoid deactivation
		if(HAL_GetTick() - brake_start_time > 1000) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);  //turn off the solenoid
		
		HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
    Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
    Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Manages the progression of a braking event through its various phases.
 * This function handles the transition between different phases of a braking event based on certain conditions.
 * It starts with checking readiness to brake, then checks for imminent collision, and finally confirms a collision.
 * 
 * @return bool_t - Returns 1 (true) if a collision has occurred and the system has responded appropriately, otherwise 0 (false).
 */
bool_t Brake_Event(){
    // Check if the system is ready to initiate braking and if it's in standby phase
	if(impact_phases == IMPACT_STANDBY && IsReadyToBrake()) {
		impact_phases = IMPACT_PHASE_1; // Move to the first phase of the braking process
		Brake_Event_Start(); // Start the braking process by activating the solenoid
	}
    // Check if the system is in the first phase and a collision is about to happen
	if(impact_phases == IMPACT_PHASE_1 && IsAboutToCollide()){
		CAN_cmd_current(0); // Command the motor to stop to prevent motor stall
		impact_phases = IMPACT_PHASE_2; // Move to the second phase
	}
    // If the system is in the second phase and a collision has occurred
	else if(impact_phases == IMPACT_PHASE_2 && IsCollided()){
		impact_phases = IMPACT_END; // Move to the end phase indicating the impact is complete
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0); // Deactivate the solenoid
		return 1;
	}
	return 0;
}

/**
 * @brief Checks if conditions are suitable for extending the stopper based on the current flywheel angle and RPMs.
 * This function calculates a buffer angle based on the higher of the current or target RPMs adjusted by a latency factor.
 * It determines if the current mechanical angle is within the calculated safe range for braking.
 * @return bool_t - Returns 1 (true) if conditions are met for braking, otherwise 0 (false).
 */
bool_t IsReadyToBrake() {
    int16_t buffer_degree;
    // Check if motor is spinning in the positive direction
	if(current_rpm > 0 && target_rpm > 0) { 
        // Calculate buffer based on the maximum of current or target RPMs
        buffer_degree = (int16_t)(((current_rpm > target_rpm) ? current_rpm : target_rpm) * MINIM_LATENCY_FACTOR) + 1; 
        // Check if the angle is within the safe braking range
		if((angle > 15 && angle < (180 - buffer_degree)) || (angle > 195 && angle < (360 - buffer_degree))){
			impact_direction = 0;
			return 1;			
		}
	}
    // Check if motor is spinning in the negative direction
	else if(current_rpm < 0 && target_rpm < 0) {
        // Calculate buffer for negative RPMs
        buffer_degree = (int16_t)(((-current_rpm > -target_rpm) ? -current_rpm : -target_rpm) * MINIM_LATENCY_FACTOR) + 1;
        // Check if the angle is within the safe braking range for reverse direction
		if((angle < 345 && angle > (180 + buffer_degree)) || (angle < 165 && angle > buffer_degree)){
			impact_direction = 1;
			return 1;
		}
	}
	return 0;
}

/**
 * @brief Initializes the brake event by activating the solenoid and recording the start time.
 * The solenoid is activated to a specific comparison value to start the braking process.
 */
void Brake_Event_Start() {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 20000); // Activate solenoid
	brake_start_time = HAL_GetTick(); // Record the start time to manage braking duration
}

/**
 * @brief Determines if the mechanical system is about to collide based on the current angle and impact direction.
 * This function checks predefined critical angle ranges to predict an imminent collision.
 * @return bool_t - Returns 1 (true) if a collision is imminent, otherwise 0 (false).
 */
bool_t IsAboutToCollide(){
    // Check angles for potential collisions based on current impact direction
	if(impact_direction == 0 && ((angle > 145 && angle < 175) || (angle > 325 && angle < 355))) return 1;
	else if(impact_direction == 1 && ((angle > 185 && angle < 215) || (angle > 5 && angle < 35))) return 1;
	return 0;
}

/**
 * @brief Checks if a collision has occurred based on the motor's RPM.
 * The function evaluates the motor speed to determine if it has effectively stopped moving due to an impact.
 * @return bool_t - Returns 1 (true) if a collision is detected, otherwise 0 (false).
 */
bool_t IsCollided() {
    // Evaluate motor speed to check if it has stopped due to collision
	if ((impact_direction == 0 && (get_motor_measure_point())->speed_rpm <= 0) ||
        (impact_direction == 1 && (get_motor_measure_point())->speed_rpm >= 0)) {
        return 1;
    }
	return 0;
}

/**
 * @brief Calibrates the angle sensor by reading the raw angle and setting it as the base reference point.
 * This function is typically called to reset the angle measurement to zero based on current position.
 */
void Calibrate_Angle(){
	base_raw_angle = (float)Read_Raw_Angle();
}

/**
 * @brief Reads the raw angle value from an AS5600 sensor using I2C communication.
 * This function transmits the register address and then reads the 2-byte raw angle value from the sensor.
 * If the read operation is successful and no previous encoder error was detected, it resets any error conditions.
 * In case of repeated failed reads, it flags an encoder error after a threshold is exceeded.
 *
 * @return uint16_t - The raw angle if read successfully, 0 otherwise (also indicates an error condition).
 */
uint16_t Read_Raw_Angle(){
	HAL_StatusTypeDef hal_status;
	uint8_t buffer[2] = {0};  // Buffer to hold the 2-byte angle data
	uint16_t raw_angle = 0;   // Variable to store the combined raw angle value
	uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;  // I2C register address for the angle
    
    // Transmit the address of the angle register to the sensor
	hal_status = HAL_I2C_Master_Transmit(&hi2c2, AS5600_ADDR, &raw_angle_register, 1, 11);
    // Receive the 2-byte raw angle value from the sensor
	hal_status = HAL_I2C_Master_Receive(&hi2c2, (AS5600_ADDR | 1), buffer, 2, 12);
    
    if(hal_status == HAL_OK) {  // Check if the I2C operations were successful
        if(status == ERR && err_type == ENCODER_ERR){  // Reset error status if previously in error
            status = FREE;
            err_type = NO_ERR;
        }
        raw_angle = buffer[0] << 8 | buffer[1];  // Combine the two bytes into one raw angle value
        angle_err_counter = 0;  // Reset the error counter on successful read
        return raw_angle;
    }
	else {
        angle_err_counter++;  // Increment the error counter on failure
        if(angle_err_counter > 100){  // Check if error count exceeds threshold
            status = ERR;  // Set system status to error
            err_type = ENCODER_ERR;  // Specify encoder error
        }
        return 0;
    }
}

/**
 * @brief Calculates the processed angle from the raw sensor data.
 * This function reads the raw angle from the sensor and adjusts it based on the base calibration angle.
 * It then converts this adjusted raw value into a 360-degree scale format to represent the actual angle.
 */
void Read_And_Calculate_Angle(){
	float raw_angle = (float)Read_Raw_Angle();  // Get the raw angle from the sensor
    if(raw_angle < 4096) {  // Validate that the raw angle is within the expected range
        // Calculate the actual angle by adjusting from the base angle and converting to degrees
    	angle = raw_angle >= base_raw_angle ? raw_angle - base_raw_angle : raw_angle + 4096 - base_raw_angle;
        angle = angle * 360 / 4096;  // Scale the adjusted angle to a 0-360 degree range
    }
}

/**
 * @brief Initializes and allocates memory for a new command queue.
 * This function sets up a command queue with a predefined capacity, initializing all parameters
 * and allocating memory for storing commands.
 * 
 * @return Command_Queue* - Pointer to the newly allocated command queue; NULL if memory allocation fails.
 */
Command_Queue * Command_Queue_Init(){
	Command_Queue *q = malloc(sizeof(Command_Queue));
    if(!q) {
        return q;
    }
    q->null_command.type = C_NOP;
    q->size = 0;
    q->capacity = 15;
    q->head = 0;
    q->tail = 0;
    q->elements = malloc(q->capacity * sizeof(Command));
    if(!(q->elements)) {
        free(q);
        q = NULL;
        return q;
    }
    return q;
}

/**
 * @brief Adds a temporary command to the command queue.
 * This function enqueues a temporary command at the tail of the queue if there is space available.
 * 
 * @param self - Pointer to the command queue where the command will be enqueued.
 * @param type - The type of command to enqueue.
 * @param values - An array of integers containing the command values.
 * 
 * @return bool_t - Returns 1 if the command is successfully enqueued, otherwise 0.
 */
bool_t Enqueue_Command(Command_Queue * self, enum Command_Type type, int values[]){
    if(self->temp_size == self->capacity) return 0;
    else{
        if (self->temp_size != 0) self->temp_tail = (self->temp_tail + 1) % self->capacity;
        (self->elements[self->temp_tail]).type = type;
        for(int i=0; i<3; ++i){
            (self->elements[self->temp_tail]).values[i] = values[i];
        }
        self->temp_size = self->temp_size + 1;
        return 1;
    }
}

/**
 * @brief Confirms the enqueuing of commands in the temporary state to the main queue.
 * This function applies the temporary size and tail updates to the actual queue state.
 * 
 * @param self - Pointer to the command queue being modified.
 */
void Confirm_Enqueue(Command_Queue * self) {
    self->size = self->temp_size;
    self->tail = self->temp_tail;
}

/**
 * @brief Reverts any changes made during a temporary enqueue operation.
 * This function resets the temporary queue size and tail to the last confirmed state.
 * 
 * @param self - Pointer to the command queue being modified.
 */
void Rollback_Enqueue(Command_Queue * self) {
    self->temp_size = self->size;
    self->temp_tail = self->tail;
}

/**
 * @brief Retrieves the command at the front of the queue without removing it.
 * This function peeks at the first command in the queue if available.
 * 
 * @param self - Pointer to the command queue.
 * 
 * @return Command - The command at the front of the queue; returns a null command if the queue is empty.
 */
Command Peek_Command(Command_Queue * self) {
    if(self->size == 0) return self->null_command;
    else{
        return self->elements[self->head];
    }
}

/**
 * @brief Removes and returns the command at the front of the queue.
 * This function dequeues the first command in the queue, adjusting the head pointer accordingly.
 * 
 * @param self - Pointer to the command queue.
 * 
 * @return Command - The dequeued command; returns a null command if the queue is empty.
 */
Command Dequeue_Command(Command_Queue * self) {
    if(self->size == 0) return self->null_command;
    else{
        Command popped = self->elements[self->head];
        if(self->size != 1) self->head = (self->head + 1) % self->capacity;
        self->size = self->size - 1;
        self->temp_size = self->temp_size - 1;
        return popped;
    }
}

/**
 * @brief Processes the next command in the command queue and updates system state based on the command type.
 * This function peeks at the front command in the queue and performs actions based on its type,
 * adjusting the motor control parameters and system state accordingly.
 */
void Process_Commend() {
    Command processing_command = Peek_Command(command_queue);  // Peek at the first command in the queue without removing it
    
    // Handle different types of commands based on their type field
	if(processing_command.type == C_ACCELERATE){
		if((status == FREE || status == STABLE || status == ACCELERATE)){
			status = ACCELERATE;
			target_rpm = processing_command.values[0];
			accelerate_current = processing_command.values[1];
			Dequeue_Command(command_queue);
		}
        else if(status == IMPACT_AFTER){ //TODO
            if((impact_direction == 0 && processing_command.values[0] < 0) || (impact_direction == 1 && processing_command.values[0] > 0)) {
                status = ACCELERATE;
                target_rpm = processing_command.values[0];
                accelerate_current = processing_command.values[1];
                Dequeue_Command(command_queue);
            }
            else{
                status = ERR;
                target_rpm = 0;
            }
        }
	}
	else if(processing_command.type == C_FREE){
		if(status == ACCELERATE || status == STABLE || status == FREE) {
			status = FREE;
            target_rpm = 0;
            Dequeue_Command(command_queue);
		}
	}
    else if(processing_command.type == C_WAIT){
		if(status == ACCELERATE);
        else if(status == FREE) {
            status = WAIT;
            Dequeue_Command(command_queue);
        } 
        else if(status == STABLE) {
            Dequeue_Command(command_queue);
        }
	}
	else if(processing_command.type == C_IMPACT) {
		if((status == STABLE || status == ACCELERATE) && (target_rpm > minimum_impact_rpm || -target_rpm > minimum_impact_rpm)){
            status = IMPACT;
            impact_phases = IMPACT_STANDBY;
            Dequeue_Command(command_queue);
		}
		else {
			status = ERR;
		}
	}
	else if(processing_command.type == C_CALIBRATE) {
		int16_t angle = processing_command.values[0];
		if((status == FREE || status == NOT_CALIBRATED)) {
			status = CALIBRATE;
			shift_angle = angle;
			Dequeue_Command(command_queue);
		}
	}
	else if(processing_command.type == C_NOP){
	}
}

/**
 * @brief Periodically sends system status updates to the computer via the USB serial interface.
 * This function formats the current RPM, angle, torque, system status, and impact phase into a message
 * string and sends it over the serial connection.
 * 
 * Message Format:
 * - Bytes 0-3: Current RPM (signed)
 * - Bytes 4-6: Current angle
 * - Bytes 7-11: Current torque
 * - Byte 12: System status
 * - Byte 13: Impact phase (unsigned integer if in impact status, '0' otherwise)
 */
void Print(){
    int time = HAL_GetTick();
    if(time - last_print_time >= print_freq){
        last_print_time = HAL_GetTick();
        uint8_t message_string[15];
        get_dec_str(message_string, 4, (uint32_t)current_rpm, 1);
        get_dec_str(message_string + 4, 3, (uint32_t)angle, 0);
        get_dec_str(message_string + 7, 5, (uint32_t)current_torque, 0);
        get_dec_str(message_string + 12, 1, (uint32_t)status, 0);
        if(status == IMPACT) get_dec_str(message_string + 13, 1, (uint32_t)impact_phases, 0);
        else message_string[13] = '0';
        message_string[14] = '\0';
        CDC_Transmit_FS((uint8_t*)&message_string, 15);
    }
}

/**
 * @brief Converts an integer value to a decimal string representation.
 * This function formats an integer value into a string of a specified length. If the integer is negative
 * and the hasSign flag is set, the string will include a negative sign. Otherwise, the string is filled with
 * the integer's digits and leading zeros.
 * 
 * @param str Pointer to the string buffer where the resulting string will be stored.
 * @param len The length of the resulting string including the sign (if any) but excluding the null terminator.
 * @param val The integer value to convert into a string.
 * @param hasSign A boolean flag indicating whether a sign should be included for negative values.
 * 
 * Functionality:
 * - If the value is negative and hasSign is true, the function places a '-' in the string; it converts the value to positive.
 * - If the value is positive and hasSign is true, the function places a '+' in the string;
 * - It then fills the string from the end to the beginning with the least significant digits of the value.
 * - Any remaining positions in the string up to 'len' are filled with '0'.
 * - The string is null-terminated after the last digit.
 */
void get_dec_str (uint8_t* str, size_t len, int32_t val, bool_t hasSign)
{
    uint8_t i = 1;
    bool_t sign = '+';  
    if(val < 0) {
        val = -val;
        sign = '-';
    }
    for(; i<=len && val > 0; ++i)
    {
        str[len-i] = (uint8_t) ((val % 10UL) + '0');
        val/=10;
    }
    for(; i<= len; ++i){
        str[len-i] = '0';
    }
    if(hasSign && str[0] == '0') {
        str[0] = sign;
    }
    
    str[i-1] = '\0';  // Null-terminate the string
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
