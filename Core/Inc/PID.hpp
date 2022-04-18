#ifndef PID_H_
#define PID_H_


class PID{
    
private:
float   kp;
float   ki;
float   kd;
float   setpoint = 0.0f;
int     max_value;
float   error = 0.0f;
float   derivative;
float   prev_error = 0.0f;
float   i_sum = 0.0f; //sum of errors for the integral part
float   output;
float   last_input = 0.0f;

#define   PID_DIRECTION_POSITIVE 1.0f
#define   PID_DIRECTION_NEGATIVE -1.0f
float   controller_direction = 1.0f;



public:
        PID(float kp,float ki,float kd, int max, float direction);
void    setSetpoint(float setpoint);
float   feedback(float val);
void    reset();
void    setProportional(float kp);
void    setIntegral(float ki);
void    setDerivative(float kd);
void    setTunings(float kp, float ki, float kd);
};


PID::PID(float kp,float ki,float kd, int max, float direction){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    max_value = max;
    this->controller_direction = direction;
}

void PID::setSetpoint(float setpoint){
    this->setpoint = setpoint; 
}

void PID::setProportional(float kp){
    this->kp = controller_direction * kp ;
}
void PID::setIntegral(float ki){
    this->ki = controller_direction * ki ;
}
void PID::setDerivative(float kd){
    this->kd = controller_direction * kd;
}
void PID::setTunings(float kp, float ki, float kd){
    this->kp = controller_direction * kp ;
    this->ki = controller_direction * ki ;
    this->kd = controller_direction * kd;
}

float PID::feedback(float val){
    //TODO: need to investigate this more by trial and error
    //calculate errors, derivative and sum
    prev_error = error;
    error = setpoint - val; 
    
    //calculate derivative:
    // derivative = error- prev_error; //we want this to in reverse signe to the proportional_output as it corrects over compensation 
    /* this is better than the derivative above, this is called: derivative on measurment 
        and it eliminated derivative spikes (derivative kick): check this for more info: 
        http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
    */
    derivative = val - last_input;
    last_input = val;
    
    //calculate integral sum
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
    derivative = 0.0f;
    prev_error = 0.0f;
    i_sum = 0.0f; 
    output = 0.0f;
}


#endif
