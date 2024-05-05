#include "PIDcontroller.h"

void PIDcontroller::setPIDgain(float kp, float ki, float kd)
{

  // set new pid-gains
  float dt = _dt / 1000; // sample time in seconds
  _kp = kp;
  _ki = ki * dt;
  _kd = kd / dt;

}

void PIDcontroller::setSetpoint(float setPoint)
{

  // set setPoint
  _setPoint = setPoint;

}

void PIDcontroller::setMaxOutput(float maxOutput)
{

  // set max. output allowed
  _maxOutput = maxOutput;

}

void PIDcontroller::setMinOutput(float minOutput)
{

  // set min. output allowed 
  _minOutput = minOutput;

}

void PIDcontroller::setSampleTime(float dt)
{

  // set new sample time
  float sampleTimeRatio = dt / _dt;
  _dt = dt;
  _ki *= sampleTimeRatio;
  _kd /= sampleTimeRatio; 

}

float PIDcontroller::getOutput()
{

  // return current output of pid
  return _output;

}

float PIDcontroller::getSetpoint()
{

  // set setpoint
  return _setPoint;

}

void PIDcontroller::turnOff()
{

  // turn off pid-controller
  _turnedOff = true;

}

void PIDcontroller::turnOn(float input)
{

  // turn on pid-controller
  _turnedOff = false;
  _iTerm = _output;
  _lastInput = input;

}

void PIDcontroller::reversePID()
{
  // change sign of pid gains
  _ki *= (-1);
  _kd *= (-1);
  _kp *= (-1);

}

float PIDcontroller::smoothSignal(float input, int n)
{

  // check if n exceeds the max. number of points
  if(n > 10) n = 10;

  // update _numStoredData for the calculation of smoothed value
  if (_numStoredData > n){
    _numStoredData = n;
  }
  else if (_numStoredData < n){
    _prevData[_numStoredData] = input;
    _numStoredData ++;
  }
  else {
    for(int i=0; i < n - 1; i++){
      _prevData[i] = _prevData[i + 1];
    }
    _prevData[n-1] = input;
  }

  // calc sum of last n values
  float sum = 0.0;

  for (int i = 0; i < _numStoredData; i++){
    sum += _prevData[i];
  }

  // calc mean of last n values
  float smoothedValue = sum / _numStoredData;

  return smoothedValue;
}

float PIDcontroller::runPID(float input)
{

  if (_turnedOff == true) return _output;
  
  unsigned long curTime = millis();
  unsigned long timeDiff = (curTime - _lastTimeUpdated);

  if(timeDiff >= _dt)
  {

    _input = input;

    // calc error and input diff
    _error = _setPoint - _input;
    float inputDiff = input - _lastInput;

    // calc iTerm, dTerm and pTerm
    _iTerm += (_ki * _error);
    float dTerm = _kd * inputDiff;
    float pTerm = _kp * _error;

    // anti wind up: check if iTerm exceeds max / min output
    if (_iTerm > _maxOutput) _iTerm = _maxOutput;
    else if (_iTerm < _minOutput) _iTerm = _minOutput;

    // calc output
    float output = _iTerm - dTerm + pTerm;

    // anti wind up: check if output exceed max / min output
    if (output > _maxOutput) output = _maxOutput;
    else if(output < _minOutput) output = _minOutput;

    // set output, update last input and last time updated
    _output = output;
    _lastInput = input;
    _lastTimeUpdated = curTime;

  }

  return _output;

}