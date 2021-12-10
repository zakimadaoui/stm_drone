#include "main.h"
#include "stdio.h"
#include "string.h"
#include "micros.h"
#include "I2CIMU.hpp"
#include "ESC.hpp"
#include "PID.hpp"
#include "ProtoHelper.h"

#define DEBUG 1

#if DEBUG == 1
#define LOG(args...) printf(args)
#else
#define LOG(args...)
#endif

// ================================== My defines ====================================

// #define BT_STAT_PIN GPIO_PIN_8
#define BT_STAT_PIN GPIOA, GPIO_PIN_8
#define BUTTON_PIN GPIOB, GPIO_PIN_11
#define BLUE_LED GPIOB, GPIO_PIN_8
#define RED_LED GPIOB, GPIO_PIN_9
#define SAFE_GUARD_ANGLE 9990.0f

// ============================= prototypes declaration =============================
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void commandCallback(Command *cmd);
void debugPrinfAndUartInterrupts();
int constrainMotorVal(float val);
void calibrateESCS();
void read_battery_and_compensate(float& v1, float& v2, float& v3, float& v4);
void init_battery();
// =============================== Global variables =================================

// variables
uint8_t uart_data = 46;
uint32_t time = 0;
char cmd;
int cmd_val;
uint32_t loop_time = 0;
volatile int flag = 0;
bool init_drone = false;
float thrust_cmd = PWM_MIN;
int pwm_operating = 0;
float battery_voltage = 11.1;
enum DroneState
{
  NONE,
  DRONE_FLY,
  DRONE_STOP,
  DRONE_CONNECTION_LOST,
  DRONE_CALIBRATE_ESCS
};
DroneState state = NONE;

// Handlers
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc1;

// Helper classes objs
I2CIMU &imu = I2CIMU::getInstance();
ESC &escs = ESC::getInstance();
ProtoHelper &phelper = ProtoHelper::getInstance();

//PID params and objs
// PIDs for rates (deg/sec) [First Control Stage]
float pitch_setpoint = 0.0f, roll_setpoint = 0.0f, yaw_setpoint = 0.0f;
float pid1_pitch_out, pid1_roll_out, pid1_yaw_out;
PID   pid1_pitch_rate(3.0f, 0.02f, 5.0f, 1800); 	 // 1800 is MAX motor output
PID   pid1_roll_rate(3.0f, 0.02f, 5.0f, 1800); 	 // 1800 is MAX motor output
PID   pid1_yaw_rate(10.8f, 0.08f, 0, 1800); // 1800 is MAX motor output
//PID   pid1_pitch_rate(1, 0, 0, 1800); 	 // 1800 is MAX motor output
//PID   pid1_roll_rate(1, 0, 0, 1800); 	 // 1800 is MAX motor output
//PID   pid1_yaw_rate(1, 0, 0, 1800); // 1800 is MAX motor output

// PIDs for angles [Second Control Stage]
float pid2_pitch_cmd, pid2_roll_cmd;  
PID   pid2_pitch(1, 0, 0, 1640);     // 164°/sec is the max rate_command output
PID   pid2_roll(1, 0, 0, 1640);		 // 164°/sec is the max rate_command output

// Motor PWM command outputs
float m1_out, m2_out, m3_out, m4_out;

//DEBUG vars
float debug_raw_pitch = 0;
float debug_raw_roll = 0;
float debug_pid_pitch = 0;
float debug_pid_roll = 0;

// ===================================== setup ======================================
void setup(){
   //init and config periphereals
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  micros_init();
  HAL_UART_Receive_IT(&huart1,&uart_data, 1);
  //set Proto+ callback to recieve the next command (init_drone) and all others
  phelper.setOnCommandRecievedCallabck(commandCallback);
  
  
  // Wait for the bluetooth module to connect to Proto+
  while (HAL_GPIO_ReadPin(BT_STAT_PIN) != GPIO_PIN_SET){
    HAL_GPIO_TogglePin(BLUE_LED);
    HAL_Delay(300);
    HAL_GPIO_TogglePin(BLUE_LED);
    HAL_GPIO_TogglePin(RED_LED);
    HAL_Delay(300);
    HAL_GPIO_TogglePin(RED_LED);
  };


  //wait for init button to be clicked in Proto+
  while (!init_drone){
     HAL_GPIO_TogglePin(BLUE_LED);
     HAL_GPIO_TogglePin(RED_LED);
     HAL_Delay(300);
     HAL_GPIO_TogglePin(BLUE_LED);
     HAL_GPIO_TogglePin(RED_LED);
     HAL_Delay(300);
  }

  //start motors
  escs.init(PWM_MIN);

  //init GYRO + Calibrate it
  HAL_GPIO_WritePin(RED_LED, GPIO_PIN_SET); //turn ON RED_LED ONLY to indicate calibration start
  HAL_GPIO_WritePin(BLUE_LED, GPIO_PIN_RESET);
  imu.init(250.0f);
  imu.calibrateSensor(200);
  //stabilize angle
  for(int i = 0; i < 100; i++) {
	  imu.updateData();
	  HAL_Delay(10);
  }
  HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET); //turn OFF RED_LED ONLY to indicate calibration end

  //initialize the battery_voltage variable to be able to filter it later
  init_battery();

//  calibrateESCS();
}

// ===================================== Main =======================================
int main(void)
{
  
  setup();
  loop_time = micros();
  while (1)
  {
    //---------------------------------- DRONE_FLY ----------------------------------
    if (state == DRONE_FLY)
    {
      //------------------------------- update angles -------------------------------
      imu.updateData();
      //-----------------------------------------------------------------------------

      //------------------------ PID stage 2 (auto-leveling) ------------------------
      //TODO: need to limit this to a certain range from PROTO+ directly (0 -> 30)
      //setpint from remote controller
      pid2_pitch.setSetpoint(-pitch_setpoint);
      pid2_roll.setSetpoint(-roll_setpoint);
      //Close outer-feedback loops and get PID rate outputs
      pid2_pitch_cmd = pid2_pitch.feedback(-imu.angle_pitch); //command in °/sec
      pid2_roll_cmd =  pid2_roll.feedback(-imu.angle_roll);
      //-----------------------------------------------------------------------------

      //--------------------- PID stage 1 (rate-stabilization) ----------------------
      //update PID rate setpoints (deg/s)
      pid1_pitch_rate.setSetpoint(pid2_pitch_cmd); //feed out PID2 into setpoint of PID1
      pid1_roll_rate.setSetpoint(pid2_roll_cmd); //feed out PID2 into setpoint of PID1
      pid1_yaw_rate.setSetpoint(yaw_setpoint); // TODO: yaw rate is given directly from the remote controller 
      //Close inner-feedback loops and get PID rate outputs
      pid1_pitch_out = pid1_pitch_rate.feedback(imu.gyro_pitch_rate_pid);
      pid1_roll_out = pid1_roll_rate.feedback(imu.gyro_roll_rate_pid);
      pid1_yaw_out = pid1_yaw_rate.feedback(imu.gyro_yaw_rate_pid);
      //-----------------------------------------------------------------------------
      

      //-------------------- calculate and constrain motor output -------------------
      m1_out = constrainMotorVal(thrust_cmd + pid1_pitch_out + pid1_roll_out - pid1_yaw_out); //Calculate the pulse for esc 1 (front-right - CCW)
      m2_out = constrainMotorVal(thrust_cmd + pid1_pitch_out - pid1_roll_out + pid1_yaw_out); //Calculate the pulse for esc 2 (rear-right - CW)
      m3_out = constrainMotorVal(thrust_cmd - pid1_pitch_out - pid1_roll_out - pid1_yaw_out); //Calculate the pulse for esc 3 (rear-left - CCW)
      m4_out = constrainMotorVal(thrust_cmd - pid1_pitch_out + pid1_roll_out + pid1_yaw_out); //Calculate the pulse for esc 4 (front-left - CW)
      //-----------------------------------------------------------------------------


      //drive motors but add safe angle protection
      if(abs(imu.angle_pitch) >= SAFE_GUARD_ANGLE || abs(imu.angle_roll) >= SAFE_GUARD_ANGLE)
      {
    	escs.drive(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
        state = DRONE_STOP;
        //stop untill start btn is clicked
        while(state != DRONE_FLY){
        HAL_GPIO_TogglePin(RED_LED);
        HAL_Delay(1000);
        }
        // update the angle a few times to stabilize it
        for(int i = 0; i < 100; i++) imu.updateData();
        // turn off the red led
        HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET);

      }
      else
        read_battery_and_compensate(m1_out, m2_out, m3_out, m4_out);
    	  escs.drive(m1_out, m2_out, m3_out, m4_out);

    }

    //--------------------------- DRONE_CONNECTION_LOST -----------------------------
    else if (state == DRONE_CONNECTION_LOST)
    {
      /*In case connection is lost, decrement motor speeds slowly within 5 secs max */

      float delay = (thrust_cmd * 0.007); //get MAX 50ms delay from a 7200 thrust
      float dec_speed = thrust_cmd *0.01 ;
      for (int i = 0; i < 100; i++)
      {
        thrust_cmd -= dec_speed;
        if (thrust_cmd < PWM_MIN) thrust_cmd = PWM_MIN;
        escs.drive(thrust_cmd,thrust_cmd,thrust_cmd,thrust_cmd);
        HAL_Delay((uint32_t) thrust_cmd);
      }
      state = NONE; //do not enter in any state after this
    }

    //--------------------------------- DRONE_STOP ----------------------------------
    else if (state == DRONE_STOP)
    {
      //turn off the motors
      escs.drive(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
      //reset everything
      pid1_pitch_rate.reset();
      pid1_roll_rate.reset();
      pid1_yaw_rate.reset();
      pid2_pitch.reset();
      pid2_roll.reset();
      pitch_setpoint = 0.0f;
      roll_setpoint = 0.0f;
      yaw_setpoint = 0.0f;
      thrust_cmd = 0.0f;
      state = NONE; //do not enter in any state after this
    }

    //-------------------------- DRONE_CALIBRATE_ESCS -------------------------------
    //TODO
    else if (state == DRONE_CALIBRATE_ESCS)
    {
      calibrateESCS();
      state = NONE; //do not enter in any state after this
    }
    
    //-------------------------------------------------------------------------------
    // Controller frequency to 250HZ
    while ((micros() - loop_time) < 4000);
    loop_time = micros();
  }
}



//================================ Config Proceadures ===============================
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

// UART interrupt handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  // LOG("IT: %c\n",(char) uart_data);
  phelper.loadByte((char)uart_data);
  HAL_UART_Receive_IT(&huart1,&uart_data, 1);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLUE_LED, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 (BLUE_LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 (RED_LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 (BTN) */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	state = DRONE_CONNECTION_LOST;
}


static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  /** Configure Regular Channel*/
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)Error_Handler();
  
}

void Error_Handler(void)
{
  state = DRONE_STOP;
}

//================================== My Proceadures =================================

void calibrateESCS(){
   // ESC CALIBRATION
   // KEEP MOTORS DISCONNECTED FROM BATTERY
   // CLICK THE BUTTON
   HAL_GPIO_WritePin(BLUE_LED, GPIO_PIN_SET);// indicate calibration start
   while(HAL_GPIO_ReadPin(BUTTON_PIN) != GPIO_PIN_SET);
   escs.drive(PWM_MAX, PWM_MAX, PWM_MAX, PWM_MAX);
   HAL_GPIO_WritePin(RED_LED, GPIO_PIN_SET);// indicate to click the btn again to continue calibration
   HAL_Delay(1000);

   // NOW CONNECT MOTORS TO BATTERY
   // WAIT FOR THE SPECIAL BEEPS
   // CLICK THE BUTTON
   while(HAL_GPIO_ReadPin(BUTTON_PIN) != GPIO_PIN_SET);
   escs.drive(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
   HAL_Delay(1000);
  
   // indicate calibration end
   HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(BLUE_LED, GPIO_PIN_RESET);
}

int constrainMotorVal(float val)
{
  if (val > PWM_MAX)
    val = PWM_MAX;
  else if (val <= PWM_MIN + pwm_operating)
    val = PWM_MIN + pwm_operating; // add 180 or other val to not stop motors while in flight
  return (int) val;
}

void init_battery()
{
  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 1252 = 11.1 V ..... 4095 = 36.3V(not needed)
  //11.1 / 1252 =  1/ 112.81
  battery_voltage = (float)HAL_ADC_GetValue(&hadc1) / 112.81;
}

void read_battery_and_compensate(float& v1, float& v2, float& v3, float& v4)
{
  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 1252 = 11.1 V ..... 4095 = 36.3V(not needed)
  //11.1 / 1252 =  1/ 112.81.
  //410.1 = 112.81 / 0.08
  battery_voltage = battery_voltage * 0.82 + (float) HAL_ADC_GetValue(&hadc1) / 1410.1;
  float scale = (11.1f - battery_voltage)/35.0f;
  //TODO: enable this later
  // v1 += v1* scale;
  // v2 += v2* scale;
  // v3 += v3* scale;
  // v4 += v4* scale;
}

//Proto+ callback
void commandCallback(Command* command){
  
  cmd = command->getNameAsChar();

  switch (cmd)
  {

  case 'i': //handshake from Proto+ that UART connection is well established
    init_drone = true;
    break;

  case 'S': //start the drone
    state = DRONE_FLY;
    // pitch_setpoint = imu.angle_pitch;
    // pitch_roll = imu.angle_roll;
    break;    

  case 'X': //stop the drone
    state = DRONE_STOP;
    break;
    
  case 'T': //Thrust command
    thrust_cmd = command->getParam(0)->getIntValue();
    // escs.drive(cmd_val, cmd_val, cmd_val, cmd_val);
    break;

  case 'R': //Roll command (IN: degrees)
    /*CODE*/
    break;

  case 'P': //Pitch command (IN: degrees)
    /*CODE*/
    break;

  case 'Z': //yaw command (IN: degrees/sec)
    /*CODE*/
    break;

  case 'A': //kp (PITCH & ROll) setting 
    pid1_pitch_rate.kp = pid1_roll_rate.kp = command->getParam(0)->getIntValue()/100.0f;
    break;

  case 'B': //ki (PITCH & ROll) setting
    pid1_pitch_rate.ki = pid1_roll_rate.ki = command->getParam(0)->getIntValue()/1000.0f;
    break;

  case 'C': //kd (PITCH & ROll) setting
    pid1_pitch_rate.kd = pid1_roll_rate.kd = command->getParam(0)->getIntValue()/10.0f;
    break;

  case 'D': //kp (YAW) setting 
    pid1_yaw_rate.kp = command->getParam(0)->getIntValue()/100.0f;
    break;

  case 'E': //ki (YAW) setting
    pid1_yaw_rate.ki = command->getParam(0)->getIntValue()/1000.0f;
    break;

  case 'F': //kd (YAW) setting
    pid1_yaw_rate.kd = command->getParam(0)->getIntValue()/100.0f;
    break;

  case 'O':
	pwm_operating = command->getParam(0)->getIntValue();
    break;

  }

  
}



