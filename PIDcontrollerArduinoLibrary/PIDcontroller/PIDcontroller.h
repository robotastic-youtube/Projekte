// PID-code based on: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
// Also added functionality to smooth sensor signals

#ifndef PIDcontroller_h
#define PIDcontroller_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "stdlib.h"
#endif

class PIDcontroller {

  public:

    // function to set PID gain
    void setPIDgain(float kp, float ki, float kd);

    // function to set setpoint
    void setSetpoint(float setPoint);

    // function to set max. allowed output value
    void setMaxOutput(float maxOutput);

    // function to set min. allowed output value
    void setMinOutput(float minOutput);

    // function to set interval at which the output is updated
    void setSampleTime(float dt);

    // function to get output of pid controller
    float getOutput();

    // function to get setpoint
    float getSetpoint();

    // function to turn off pid
    void turnOff();

    // function to turn on pid
    void turnOn(float input);

    // function to reverse PID controller -> output gets smaller if input gets bigger 
    void reversePID();
    
    // function to run pid controller
    float runPID(float input);

    // function to smooth data to improve derivative 
    float smoothSignal(float input, int n);

  private:

    // pid-controller gain
    float _kp = 0.0;
    float _ki = 0.0;
    float _kd = 0.0;

    // input and ouput variable
    float _input = 0.0;
    float _lastInput = 0.0;
    float _output = 0.0;
    float _maxOutput = 0.0;
    float _minOutput = 0.0;

    // setpoint
    float _setPoint = 0.0;

    // error
    float _error = 0.0;

    // previous error
    float _prevError = 0.0;

    // sum of error
    float _errorSum = 0.0;

    // time interval at which the output is updated, default: 100 ms
    float _dt = 100;

    // last time when output was updated
    unsigned long _lastTimeUpdated = 0.0;

    // sum of integral term
    float _iTerm = 0.0;

    // bool to turn off the pid
    bool _turnedOff = false;

    // max allowed error in time diff
    float _dtError = 1;

    // previous stored data and number of stored data
    float _prevData[10];
    int _numStoredData = 0;

};

#endif