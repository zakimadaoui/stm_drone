#ifndef PID_H_
#define PID_H_


class PID{

public:
float   kp;
float   ki;
float   kd;
    
private:
float   setpoint = 0.0f;
int     max_value;
float   error = 0.0f;
float   derivative;
float   prev_error = 0.0f;
float   i_sum = 0.0f; //sum of errors for the integral part
float   output;


public:
        PID(float kp,float ki,float kd, int max);
void    setSetpoint(float setpoint);
float   feedback(float val);
void    reset();
};


PID::PID(float kp,float ki,float kd, int max){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    max_value = max;
}

void PID::setSetpoint(float setpoint){
    this->setpoint = setpoint;
}

float PID::feedback(float val){
    
    //calculate errors, derivative and sum
    prev_error = error;
    error = val - setpoint; 
    derivative = prev_error - error; //we want this to in reverse signe to the proportional_output as it corrects over compensation 
    i_sum += ki * error;
    
    //integral windup (this is not redundant since it may be reduced by the derivative or proportional)
    if(i_sum > max_value)i_sum = max_value;
    else if(i_sum < max_value * -1)i_sum = max_value * -1;
    
    //calculate PID output
    output = kp*error + i_sum + kd*derivative;
    
    //output windup
    if(output > max_value)output = max_value;
    else if(output < max_value * -1)output = max_value * -1;
    return output;
}

void PID::reset(){
    setpoint = 0.0f;
    error = 0.0f;
    derivative;
    prev_error = 0.0f;
    i_sum = 0.0f; 
    output = 0.0f;
}


#endif
