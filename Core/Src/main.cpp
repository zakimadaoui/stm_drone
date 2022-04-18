#include "main.h"
#include "stdio.h"
#include "string.h"
#include "micros.h"
#include "I2CIMU.hpp"
#include "ESC.hpp"
#include "PID.hpp"
#include "ProtoHelper.h"

#define DEBUG 0

#if DEBUG == 1
#define LOG(args...) printf(args)
#else
#define LOG(args...)
#endif

// ================================== My defines ====================================

#define BT_STAT_PIN GPIOA, GPIO_PIN_8
#define BUTTON_PIN GPIOB, GPIO_PIN_11
#define BLUE_LED GPIOB, GPIO_PIN_8
#define RED_LED GPIOB, GPIO_PIN_9
#define SAFE_GUARD_ANGLE 27.0f
#define ADC_TIMOUT 1000

// ============================= prototypes declaration =============================
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void commandCallback(Command *cmd);
int constrainMotorVal(float val);
void calibrateESCS();
void read_battery_and_compensate(float &v1, float &v2, float &v3, float &v4);
void ADC_init();
uint32_t ADC_read();
void EscErrorHandler(); //TODO: these souldn't be called handlers as they are not in the original class
void I2cErrorHandler();
void AdcErrorHandler();
// =============================== Global variables =================================

// variables
uint8_t uart_data = 46;
uint32_t time = 0;
char cmd;
int cmd_val;
uint64_t loop_time = 0;
volatile int flag = 0;
bool init_drone = false;
float thrust_cmd = PWM_MIN;
int pwm_operating = 0;
float battery_voltage = 11.1;
bool stop_due_to_angle = false;
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

// Helper classes objs
I2CIMU &imu = I2CIMU::getInstance();
ESC &escs = ESC::getInstance();
ProtoHelper &phelper = ProtoHelper::getInstance();

//PID params and objs
// PIDs for rates (deg/sec) [First Control Stage]
float pitch_setpoint = 0.0f, roll_setpoint = 0.0f, yaw_setpoint = 0.0f;
float pid1_pitch_out, pid1_roll_out, pid1_yaw_out;

// PID1 this controller is going to make the drone resilient to rate change only !
// PID   pid1_pitch_rate(3.0f, 0.02f, 5.0f, 1800); 	 // 1800 is MAX/2 motor output
// PID   pid1_roll_rate(3.0f, 0.02f, 5.0f, 1800); 	 // 1800 is MAX/2 motor output
// PID   pid1_yaw_rate(10.8f, 0.08f, 0, 1800); // 1800 is MAX/2 motor output
// PID pid1_pitch_rate(2.5f,0, 0, 1800); // 1800 is MAX/2 motor output
// PID pid1_roll_rate(2.5f, 0, 0, 1800);  // 1800 is MAX/2 motor output
// PID pid1_yaw_rate(15.0f, 0.1f, 0, 1800);   // 1800 is MAX/2 motor output

PID pid_pitch(0,0, 0, 1800, PID_DIRECTION_NEGATIVE); // 1800 is MAX/2 motor output
PID pid_roll(0, 0, 0, 1800, PID_DIRECTION_NEGATIVE);  // 1800 is MAX/2 motor output
PID pid_yaw(0, 0, 0, 1800, PID_DIRECTION_POSITIVE);   // 1800 is MAX/2 motor output
//PID pid_yaw(5.0f, 0.1f, 0, 1800, PID_DIRECTION_POSITIVE);   // 1800 is MAX/2 motor output
float pid_pitch_out, pid_roll_out, pid_yaw_out;

// PID2 for angles [Second Control Stage]
// this controller is for stabilizing on an angle and could only be "P"or "PI" controller
// float pid2_pitch_out, pid2_roll_out; //angle PID output in Deg/s
// PID pid2_pitch(0, 0, 0, 1640);       // 164°/sec is the max rate_command output
// PID pid2_roll(0, 0, 0, 1640);        // 164°/sec is the max rate_command output

// Motor PWM command outputs
float m1_out, m2_out, m3_out, m4_out;

//DEBUG vars
float debug_pitch = 0;
float debug_roll = 0;
float debug_pid_pitch = 0;
float debug_pid_roll = 0;



// ===================================== setup ======================================
void setup()
{
  //init and config periphereals
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  micros_init();
  HAL_UART_Receive_IT(&huart1, &uart_data, 1);
  ADC_init();
  //set Proto+ callback to recieve the next command (init_drone) and all others
  phelper.setOnCommandRecievedCallabck(commandCallback);

  // Wait for the bluetooth module to connect to Proto+
  #if DEBUG != 1
  while (HAL_GPIO_ReadPin(BT_STAT_PIN) != GPIO_PIN_SET)
  {
    HAL_GPIO_TogglePin(BLUE_LED);
    HAL_Delay(300);
    HAL_GPIO_TogglePin(BLUE_LED);
    HAL_GPIO_TogglePin(RED_LED);
    HAL_Delay(300);
    HAL_GPIO_TogglePin(RED_LED);
  };
  #endif

  //wait for init button to be clicked in Proto+
  #if DEBUG != 1
  while (!init_drone)
  {
    HAL_GPIO_TogglePin(BLUE_LED);
    HAL_GPIO_TogglePin(RED_LED);
    HAL_Delay(300);
    HAL_GPIO_TogglePin(BLUE_LED);
    HAL_GPIO_TogglePin(RED_LED);
    HAL_Delay(300);
  }
  #endif

  //start motors
  escs.setEscErrorHandler(EscErrorHandler);
  escs.init(PWM_MIN);

  //init GYRO + Calibrate it
  HAL_GPIO_WritePin(RED_LED, GPIO_PIN_SET); //turn ON RED_LED ONLY to indicate calibration start
  HAL_GPIO_WritePin(BLUE_LED, GPIO_PIN_RESET);

  //imu init and calibration (please keep both calibrations faulty and accurate)
  imu.setI2cErrorHandler(I2cErrorHandler);
  imu.init(250.0f);
  imu.calibrateSensor(200);  //first calibration is faulty
  imu.calibrateSensor(2000); // second calibration is much more accurate

  HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET); //turn OFF RED_LED ONLY to indicate calibration end
  //  calibrateESCS();

  //TODO: you should mention that the user should leave the angle to stab a little before use
}

// ===================================== Main =======================================
int main(void)
{
  setup();
  loop_time = micros();
  while (1)
  {
    //---------------------------------- DRONE_FLY ----------------------------------
    #if DEBUG == 1
    state = DRONE_FLY;
    #endif
    if (state == DRONE_FLY)
    {
      //------------------------------- update angles -------------------------------
      imu.updateData();
      debug_pitch = imu.angle_pitch;
      debug_roll = imu.angle_roll;
      //-----------------------------------------------------------------------------

      //------------------------ PID controller (traditional)------------------------
      //TODO: Range Limited from PROTO+ directly (0 -> 30 degrees)
      pid_pitch.setSetpoint(pitch_setpoint);
      pid_roll.setSetpoint(roll_setpoint);
      pid_yaw.setSetpoint(yaw_setpoint);

      pid_pitch_out = pid_pitch.feedback(imu.angle_pitch);
      pid_roll_out = pid_roll.feedback(imu.angle_roll);
      pid_yaw_out = pid_yaw.feedback(imu.gyro_yaw_rate_pid);


      //-------------------- calculate and constrain motor output -------------------
      m1_out = constrainMotorVal(thrust_cmd + pid_pitch_out + pid_roll_out - pid_yaw_out); //Calculate the pulse for esc 1 (front-right - CCW)
      m2_out = constrainMotorVal(thrust_cmd + pid_pitch_out - pid_roll_out + pid_yaw_out); //Calculate the pulse for esc 2 (rear-right - CW)
      m3_out = constrainMotorVal(thrust_cmd - pid_pitch_out - pid_roll_out - pid_yaw_out); //Calculate the pulse for esc 3 (rear-left - CCW)
      m4_out = constrainMotorVal(thrust_cmd - pid_pitch_out + pid_roll_out + pid_yaw_out); //Calculate the pulse for esc 4 (front-left - CW)
      //-----------------------------------------------------------------------------

      //drive motors but add safe angle protection
      if (abs(imu.angle_pitch) >= SAFE_GUARD_ANGLE || abs(imu.angle_roll) >= SAFE_GUARD_ANGLE)
      {
        escs.drive(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN); //STOP motors
        stop_due_to_angle = true;// flag bad angle
        state = DRONE_STOP;//switch to stop state
      }
      else
      {
        read_battery_and_compensate(m1_out, m2_out, m3_out, m4_out);
        escs.drive(m1_out, m2_out, m3_out, m4_out);
      }
    }

    //--------------------------- DRONE_CONNECTION_LOST -----------------------------
    else if (state == DRONE_CONNECTION_LOST)
    {
      /*In case connection is lost, decrement motor speeds slowly within 5 secs max */

//      float delay = (thrust_cmd * 0.007); //get MAX 50ms delay from a 7200 thrust
//      float dec_speed = thrust_cmd * 0.01;
//      for (int i = 0; i < 100; i++)
//      {
//        thrust_cmd -= dec_speed;
//        if (thrust_cmd < PWM_MIN)
//          thrust_cmd = PWM_MIN;
//        escs.drive(thrust_cmd, thrust_cmd, thrust_cmd, thrust_cmd);
//        HAL_Delay((uint32_t)thrust_cmd);
//      }
    	//TODO: think about something better
      state = DRONE_STOP; //stop the drone
    }

    //--------------------------------- DRONE_STOP ----------------------------------
    else if (state == DRONE_STOP)
    {
      //turn off the motors
      escs.drive(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
      // TODO: reset everything
      // pid1_pitch_rate.reset();
      // pid1_roll_rate.reset();
      // pid1_yaw_rate.reset();
      // pid2_pitch.reset();
      // pid2_roll.reset();

      pid_yaw.reset();
      pid_pitch.reset();
      pid_roll.reset();

      pitch_setpoint = 0.0f;
      roll_setpoint = 0.0f;
      yaw_setpoint = 0.0f;
      imu.reset(); //reset sensor vals to zero to start from ground again
      thrust_cmd = PWM_MIN;

      //TODO: (add this in the drone manual)stop untill start btn is clicked
      if(stop_due_to_angle){
			while (state != DRONE_FLY)
			{
			  HAL_GPIO_TogglePin(RED_LED);
			  HAL_Delay(1000);
			}
			stop_due_to_angle = false; //reset flag
			HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET);// turn off the red led
      }

      state = NONE; //do not enter in any state after this
    }

    //-------------------------- DRONE_CALIBRATE_ESCS -------------------------------
    //TODO: add this btn to app ?
    else if (state == DRONE_CALIBRATE_ESCS)
    {
      calibrateESCS();
      state = NONE; //do not enter in any state after this
    }

    //-------------------------------------------------------------------------------
    // Controller frequency to 250HZ
    // Current loop code delay is around 857us!!!
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
//    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
//    Error_Handler();TODO
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
//    Error_Handler();TODO
  }
}

// UART interrupt handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // LOG("IT: %c\n",(char) uart_data);
  phelper.loadByte((char)uart_data);
  HAL_UART_Receive_IT(&huart1, &uart_data, 1);
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

//================================== My Proceadures =================================

void calibrateESCS()
{
  // ESC CALIBRATION
  // KEEP MOTORS DISCONNECTED FROM BATTERY
  // CLICK THE BUTTON
  HAL_GPIO_WritePin(BLUE_LED, GPIO_PIN_SET); // indicate calibration start
  while (HAL_GPIO_ReadPin(BUTTON_PIN) != GPIO_PIN_SET)
    ;
  escs.drive(PWM_MAX, PWM_MAX, PWM_MAX, PWM_MAX);
  HAL_GPIO_WritePin(RED_LED, GPIO_PIN_SET); // indicate to click the btn again to continue calibration
  HAL_Delay(1000);

  // NOW CONNECT MOTORS TO BATTERY
  // WAIT FOR THE SPECIAL BEEPS
  // CLICK THE BUTTON
  while (HAL_GPIO_ReadPin(BUTTON_PIN) != GPIO_PIN_SET)
    ;
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
  else if (val < PWM_MIN + pwm_operating)
    val = PWM_MIN + pwm_operating; // add 180 or other val to not stop motors while in flight
  return (int)val;
}

void init_battery()
{
  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 1252 = 11.1 V ..... 4095 = 36.3V(not needed)
  //11.1 / 1252 =  1/ 112.81
  battery_voltage = (float)(ADC_read()) / 112.81f;
}

void ADC_init()
{
  // CLK config for ADC1
  // ADC clock can be 14 Mhz at max. So we need to divide the
  //   APB2 by 6 to make it 12 Mhz
  // Ensure ADCPRE is zero
  RCC->CFGR &= ~(0b11 << 14);
  // Set prescaler to 6
  RCC->CFGR |= (0b10 << 14);

  // Enable alternate function clock. Bit 0 in RCC APB2ENR register
  RCC->APB2ENR |= (1 << 0);
  // Enable GPIOD clock. Bit 5 in RCC APB2ENR register
  RCC->APB2ENR |= (1 << 5);
  // Enable GPIOC clock. Bit 4 in RCC APB2ENR register
  RCC->APB2ENR |= (1 << 4);
  // Enable clock for ADC1 clock. Bit 9 in RCC APB2ENR register
  RCC->APB2ENR |= (1 << 9);

  //LL ADC
  ADC1->CR2 &= ~ADC_CR2_ALIGN_Msk; // Enable right alignment
  ADC1->CR2 &= ~ADC_CR2_CONT_Msk;  // Enable single conversion
  ADC1->CR2 |= ADC_CR2_EXTSEL_Msk; // enable software trigger SWSRART

  //config sample time
  ADC1->SMPR2 = 0; //1.5 ADC cycles

  //configre regular channel to 1 conversion for CH9
  ADC1->SQR1 = 0U; // set L[3:0] to make 1 coversion
  ADC1->SQR3 = 9U; // set CH9 as first in the sequence (CHN9 = PB1)

  ADC1->CR2 |= ADC_CR2_ADON_Msk; // Power on ADC

  //Reset calibration and calibrate the ADC for better measurements
  //+ gain few ADC clk cycles to satisfy T_stab after powering on the ADC
  uint32_t init_time = micros();
  ADC1->CR2 |= ADC_CR2_RSTCAL_Msk;               // reset calibration
  while ((ADC1->CR2 & ADC_CR2_RSTCAL_Msk) != 0U) //wait for RSTCAL bit to reset
  {
    if ((micros() - init_time) > ADC_TIMOUT)
      AdcErrorHandler();
  }
  ADC1->CR2 |= ADC_CR2_CAL_Msk; //calibrate
  while ((ADC1->CR2 & ADC_CR2_CAL_Msk) != 0)
    ; //wait for CAL bit to reset
  {
    if ((micros() - init_time) > ADC_TIMOUT)
      AdcErrorHandler();
  }

  init_battery();
}

uint32_t ADC_read()
{
  uint32_t init_time = micros();

  ADC1->CR2 |= ADC_CR2_ADON_Msk; // start conversion
  while ((ADC1->SR & ADC_SR_EOC) != ADC_SR_EOC)
  { // waite for conversion flag
    if ((micros() - init_time) > ADC_TIMOUT)
      AdcErrorHandler();
  }

  return ADC1->DR; // read data (clears EOC auto)
}

void read_battery_and_compensate(float &v1, float &v2, float &v3, float &v4)
{
  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 1252 = 11.1 V ..... 4095 = 36.3V(not needed)
  //11.1 / 1252 =  1/ 112.81.
  //1410.1 = 112.81 / 0.08
  battery_voltage = battery_voltage * 0.92 + ((float)(ADC_read() / 1410.1f)); //read and filter

  // TODO: Appearently the battery reading is inverse proportional to the
  // Thrust command, the higher the trhust the low is the battery reading
  // So this is causing a huge noise that makes battery reading very incorrect
  //  battery_voltage = (float)(ADC_read()) / 112.81f;

//  if(battery_voltage <10.0f){ //LOW battery
//	 state =DRONE_STOP;
//	 HAL_GPIO_WritePin(BLUE_LED, GPIO_PIN_SET);
//  }
//  else if(battery_voltage <13.0f){ //compensate PWM
//   float scale = (13.0f - battery_voltage)/13.0f;
//   v1 += v1* scale;
//   v2 += v2* scale;
//   v3 += v3* scale;
//   v4 += v4* scale;
//  }
}

void EscErrorHandler()
{
  //TODO: if use LCD display a message error for PWM
}

void I2cErrorHandler()
{
  //TODO: if use LCD display a message error for I2C
}
void AdcErrorHandler()
{
  //TODO: if use LCD display a message error for ADC
}

//Proto+ callback
void commandCallback(Command *command)
{

  cmd = command->getNameAsChar();

  switch (cmd)
  {

  case 'i': //handshake from Proto+ that UART connection is well established
    init_drone = true;
    break;

  case 'S': //start the drone
    state = DRONE_FLY;
    break;

  case 'X': //stop the drone
    state = DRONE_STOP;
    break;

  case 'T': //Thrust command
    thrust_cmd = command->getParam(0)->getIntValue();
    // escs.drive(cmd_val, cmd_val, cmd_val, cmd_val);
    break;

  case 'R': //Roll command (IN: degrees)
    // roll_setpoint = command->getParam(0)->getFloatValue();
    break;

  case 'P': //Pitch command (IN: degrees)
    /*CODE*/
    break;

  case 'Y': //yaw command (IN: degrees/sec)
    /*CODE*/
    break;

  case 'A': //kp_pitch setting
    pid_pitch.setProportional(command->getParam(0)->getIntValue() / 100.0f);
    break;

  case 'B': //ki_pitch setting
    pid_pitch.setIntegral(command->getParam(0)->getIntValue() / 1000.0f);
    break;

  case 'C': //kd_pitch setting
    pid_pitch.setDerivative(command->getParam(0)->getIntValue() / 10.0f);
    break;

  case 'D': //kp (YAW rate) setting
    pid_yaw.setProportional(command->getParam(0)->getIntValue() / 100.0f);
    break;

  case 'E': //ki (YAW rate) setting
    pid_yaw.setIntegral(command->getParam(0)->getIntValue() / 1000.0f);
    break;

  case 'F': //kd (YAW rate) setting
    pid_yaw.setDerivative(command->getParam(0)->getIntValue() / 10.0f);
    break;

  case 'G': //kp_roll 
    pid_roll.setProportional(command->getParam(0)->getIntValue() / 100.0f);
    break;

  case 'H': //ki_roll 
    pid_roll.setIntegral(command->getParam(0)->getIntValue() / 1000.0f);
    break;

  case 'I': //kd_roll 
    pid_roll.setDerivative(command->getParam(0)->getIntValue() / 10.0f);
    break;

  case 'O':
    pwm_operating = command->getParam(0)->getIntValue();
    break;
  
  case 'Q': //for debugging
    char pData[60];
    int pitch = imu.angle_pitch *100;
    int roll  = imu.angle_roll *100 ;
    int yaw   = imu.gyro_yaw_rate_pid *100;
    sprintf(pData, "p: %d | r: %d | y: %d \n\0", pitch, roll, yaw);
    uint16_t size = strlen(pData);
    HAL_UART_Transmit(&huart1, (uint8_t*) pData, size, 1000);
	  break;
  }
}
