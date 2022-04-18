#if !defined(ESC_H_)
#define ESC_H_
#include "stm32f1xx_hal.h"

#define PWM_MAX 7199
#define PWM_MIN 3599

class ESC
{

private:
TIM_HandleTypeDef htim2;
void (*err_handler)(void);// a fct pointer for err_handler that takes/returns no params

private:
                ESC();
public:
static ESC&     getInstance();
void            init(int init_speed);
void            drive(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
ESC(ESC& other) = delete; //Singletons should not be cloneable
void setEscErrorHandler(void (*err_handler)(void));
};

ESC::ESC(){
//empty
}

ESC& ESC::getInstance() {
    static ESC _instance;
    return _instance;
}

void  ESC::setEscErrorHandler(void (*err_handler)(void)){
    this->err_handler = err_handler;
}

void ESC::init(int init_speed){

    //init TIM2 and PWM channels

    //=========================== HAL TIM2 CONFIG ==============================
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 19; //from 0 to 19 ie /20
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 59999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) err_handler();
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) err_handler();
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) err_handler();
    
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) err_handler();
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) err_handler();
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_MIN;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) err_handler();

    sConfigOC.Pulse = init_speed; //PWM_MIN
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) err_handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) err_handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) err_handler();
    
    HAL_TIM_MspPostInit(&htim2);
    //=========================== HAL PWM CONFIG ==============================
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}


void ESC::drive(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4){
    TIM2->CCR1 = m4;
    TIM2->CCR2 = m3;
    TIM2->CCR3 = m2;
    TIM2->CCR4 = m1;
}



#endif
