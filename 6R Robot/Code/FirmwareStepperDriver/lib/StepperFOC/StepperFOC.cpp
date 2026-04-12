/*
================================================================================
File: StepperFOC.cpp
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Implementation file for the StepperFOC class. Handles the instantiation, 
initialization, and configuration of all hardware-specific objects (motor, 
driver, sensors, ADC) and exposes a clean interface for parameter tuning 
and state control.

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#include "StepperFOC.h"

/**
 * @brief Constructor implementation. Sets initial motor and driver parameters.
 * @param polePairs The number of pole pairs for the stepper motor.
 * @param voltagePowerSupply The main supply voltage connected to the driver.
 * @param voltageLimit The maximum voltage the driver is allowed to output.
 * @param voltageSensorAlign The voltage applied during the FOC alignment phase.
 */
StepperFOC::StepperFOC(uint8_t polePairs, float_t voltagePowerSupply, float_t voltageLimit, float_t voltageSensorAlign) {
    // Initialize driver configuration from constructor arguments.
    _driverConfig.voltagePowerSupply = voltagePowerSupply;
    _driverConfig.voltageLimit = voltageLimit;

    // Initialize motor configuration from constructor arguments.
    _motorConfig.polePairs = polePairs;
    _motorConfig.voltageSensorAlign = voltageSensorAlign;
    // Default motor voltage limit is set to half of the driver's limit.
    _motorConfig.voltageLimit = _driverConfig.voltageLimit / 2;
}

/**
 * @brief Destructor implementation. Frees all dynamically allocated memory.
 */
StepperFOC::~StepperFOC() {
    delete _spiEncoder;
    delete _magneticSensor;
    delete _driver;
    delete _motor;
    delete _currentSense;
}

/**
 * @brief Main initialization function. Creates and configures all hardware objects.
 * @param instance Pointer to the STM32 ADC peripheral instance (defaults to ADC1).
 */
void StepperFOC::initStepperFOC(ADC_TypeDef* instance) {
    // Create instances of all required hardware objects.
    _spiEncoder = new SPIClass(_sensorConfig.posSensorMOSIPin, _sensorConfig.posSensorMISOPin, _sensorConfig.posSensorSCKPin);
    _motor = new StepperMotor(_motorConfig.polePairs);
    _driver = new StepperDriver4PWM(_driverConfig.phaseAPin1, _driverConfig.phaseBPin1, _driverConfig.phaseAPin2, _driverConfig.phaseBPin2, _driverConfig.enableAPin, _driverConfig.enableBPin);
    _magneticSensor = new MagneticSensorAS5047(_sensorConfig.posSensorCSpin, 1);
    _currentSense = new LowsideCurrentSense(_sensorConfig.currentSensorGain, _sensorConfig.currentSensorPhaseAPin, _sensorConfig.currentSensorPhaseBPin);
    
    // Initialize the ADC peripheral before using it for current sensing.
    initADC(instance);

    // Initialize the magnetic position sensor.
    _magneticSensor->init(_spiEncoder);

    // Configure and initialize the motor driver.
    _driver->voltage_power_supply = _driverConfig.voltagePowerSupply;
    _driver->voltage_limit = _driverConfig.voltageLimit;
    _driver->pwm_frequency = _driverConfig.pwmFrequency;
    if (_driver->init() == 0 && _debug) Serial.println("Driver Init failed.");

    // Link the driver to the current sense object and initialize it.
    _currentSense->linkDriver(_driver);

    // Link components (sensor, driver) to the main motor object.
    _motor->linkSensor(_magneticSensor);
    _motor->linkDriver(_driver);

    // Set fundamental motor control settings from the config struct.
    _motor->voltage_limit = _motorConfig.voltageLimit;
    _motor->voltage_sensor_align = _motorConfig.voltageSensorAlign;

    // Set current control PID parameters.
    _motor->PID_current_q.P = _motorConfig.pidCurrentQP;
    _motor->PID_current_q.I = _motorConfig.pidCurrentQI;
    _motor->PID_current_d.P = _motorConfig.pidCurrentDP;
    _motor->PID_current_d.I = _motorConfig.pidCurrentDI;
    _motor->LPF_current_q.Tf = _motorConfig.lpfCurrentQ;
    _motor->LPF_current_d.Tf = _motorConfig.lpfCurrentD;
    _motor->current_limit = _motorConfig.currentLimit;
    _motor->PID_current_d.output_ramp = _motorConfig.pidCurrentDoutputRamp;
    _motor->PID_current_q.output_ramp = _motorConfig.pidCurrentQoutputRamp;

    // Set velocity control PID parameters.
    _motor->PID_velocity.P = _motorConfig.pidVelocityP;
    _motor->PID_velocity.I = _motorConfig.pidVelocityI;
    _motor->PID_velocity.D = _motorConfig.pidVelocityD;
    _motor->LPF_velocity.Tf = _motorConfig.lpfVelocity;

    // Set angle control PID parameters.
    _motor->P_angle.P = _motorConfig.pidAngleP;
    _motor->P_angle.I = _motorConfig.pidAngleI;
    _motor->P_angle.D = _motorConfig.pidAngleD;
    _motor->P_angle.output_ramp = _motorConfig.angleOutputRamp;
    _motor->velocity_limit = _motorConfig.angleVelocityLimit;

    // Initialize the motor object itself.
    if (_motor->init() == 0 && _debug) Serial.println("Motor Init failed.");
    // Initialize current sense
    if (_currentSense->init() == 0 && _debug) Serial.println("Current Sense Init failed");
    // Link current sense to driver
    _motor->linkCurrentSense(_currentSense);
    // Initialize the FOC algorithm, which includes sensor alignment.
    if (_motor->initFOC() == 0 && _debug) Serial.println("FOC Init failed.");

    // Set a default control mode on startup.
    setControlType(MotionControlType::velocity);
}

/**
 * @brief Low-level ADC initialization using the STM32 HAL.
 * @param instance Pointer to the specific ADC peripheral (e.g., ADC1, ADC2).
 */
void StepperFOC::initADC(ADC_TypeDef* instance) {
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Configure the main ADC peripheral settings.
    _hadc.Instance = instance;
    _hadc.Init.ClockPrescaler = _adcConfig.ClockPrescaler;
    _hadc.Init.Resolution = _adcConfig.Resolution;
    _hadc.Init.DataAlign = _adcConfig.DataAlign;
    _hadc.Init.GainCompensation = _adcConfig.GainCompensation;
    _hadc.Init.ScanConvMode = _adcConfig.ScanConvMode;
    _hadc.Init.EOCSelection = _adcConfig.EOCSelection;
    _hadc.Init.LowPowerAutoWait = _adcConfig.LowPowerAutoWait;
    _hadc.Init.ContinuousConvMode = _adcConfig.ContinuousConvMode;
    _hadc.Init.NbrOfConversion = _adcConfig.NbrOfConversion;
    _hadc.Init.DiscontinuousConvMode = _adcConfig.DiscontinuousConvMode;
    _hadc.Init.ExternalTrigConv = _adcConfig.ExternalTrigConv;
    _hadc.Init.ExternalTrigConvEdge = _adcConfig.ExternalTrigConvEdge;
    _hadc.Init.DMAContinuousRequests = _adcConfig.DMAContinuousRequests;
    _hadc.Init.Overrun = _adcConfig.Overrun;
    _hadc.Init.OversamplingMode = _adcConfig.OversamplingMode;
    if (HAL_ADC_Init(&_hadc) != HAL_OK) {
        if (_debug) Serial.println("ADC Initialization failed.");
        while(true); // Halt on failure
    }

    // Configure multi-mode settings (not typically used for this setup).
    multimode.Mode = _adcConfig.Mode;
    if (HAL_ADCEx_MultiModeConfigChannel(&_hadc, &multimode) != HAL_OK) {
        if (_debug) Serial.println("Multi Mode Configuration Channel failed.");
        while(true); // Halt on failure
    }

    // Configure the specific channel for conversion.
    sConfig.Channel = _adcConfig.Channel;
    sConfig.Rank = _adcConfig.Rank;
    sConfig.SamplingTime = _adcConfig.SamplingTime;
    sConfig.SingleDiff = _adcConfig.SingleDiff;
    sConfig.OffsetNumber = _adcConfig.OffsetNumber;
    sConfig.Offset = _adcConfig.Offset;
    if (HAL_ADC_ConfigChannel(&_hadc, &sConfig) != HAL_OK) {
        if (_debug) Serial.println("ADC Config Channel failed.");
        while(true); // Halt on failure
    }
}

/**
 * @brief Activates debug prints to the serial monitor.
 */
void StepperFOC::activateDebug() {
    _debug = true;
}

/**
 * @brief Sets the primary motion control type and FOC modulation.
 * @param motionType The desired motion control mode (e.g., angle, velocity).
 * @param focModulation The FOC modulation technique (defaults to SinePWM).
 */
void StepperFOC::setControlType(MotionControlType motionType, FOCModulationType focModulation) {
    _motor->controller = motionType;
    _motor->foc_modulation = focModulation;
}

/**
 * @brief Overloaded function to set motion, torque, and FOC modulation types.
 * @param motionType The desired motion control mode.
 * @param torqueType The underlying torque control technique (e.g., voltage, foc_current).
 * @param focModulation The FOC modulation technique.
 */
void StepperFOC::setControlType(MotionControlType motionType, TorqueControlType torqueType, FOCModulationType focModulation) {
    _motor->controller = motionType;
    _motor->torque_controller = torqueType;
    _motor->foc_modulation = focModulation;
}

// --- Implementation of Getters ---

/**
 * @brief Retrieves the sensor configuration struct.
 * @return SensorConfig& Reference to the internal sensor config.
 */
SensorConfig& StepperFOC::getSensorConfig() { return _sensorConfig; }

/**
 * @brief Retrieves the driver configuration struct.
 * @return DriverConfig& Reference to the internal driver config.
 */
DriverConfig& StepperFOC::getDriverConfig() { return _driverConfig; }

/**
 * @brief Retrieves the motor configuration struct.
 * @return MotorConfig& Reference to the internal motor config.
 */
MotorConfig& StepperFOC::getMotorConfig() { return _motorConfig; }

/**
 * @brief Retrieves the ADC configuration struct.
 * @return ADCConfig& Reference to the internal ADC config.
 */
ADCConfig& StepperFOC::getADCConfig() { return _adcConfig; }

/**
 * @brief Retrieves the pointer to the SimpleFOC StepperMotor object.
 * @return StepperMotor* Pointer to the motor instance.
 */
StepperMotor* StepperFOC::getMotor() const { return _motor; }

/**
 * @brief Retrieves the pointer to the SimpleFOC Magnetic Sensor object.
 * @return MagneticSensorAS5047* Pointer to the sensor instance.
 */
MagneticSensorAS5047* StepperFOC::getMagneticSensor() const { return _magneticSensor; }

/**
 * @brief Retrieves the current target setpoint.
 * @return float_t The current target (angle, velocity, or torque).
 */
float StepperFOC::getTarget() { return _target; }

// --- Implementation of Setters ---

/**
 * @brief Overwrites the current sensor configuration.
 * @param config The new SensorConfig struct.
 */
void StepperFOC::setSensorConfig(const SensorConfig& config) { _sensorConfig = config; }

/**
 * @brief Overwrites the current driver configuration.
 * @param config The new DriverConfig struct.
 */
void StepperFOC::setDriverConfig(const DriverConfig& config) { _driverConfig = config; }

/**
 * @brief Overwrites the current motor configuration.
 * @param config The new MotorConfig struct.
 */
void StepperFOC::setMotorConfig(const MotorConfig& config) { _motorConfig = config; }

/**
 * @brief Overwrites the current ADC configuration.
 * @param config The new ADCConfig struct.
 */
void StepperFOC::setADCConfig(const ADCConfig& config) { _adcConfig = config; }

/**
 * @brief Updates the target variable for the motor control loop.
 * @param target The new target value.
 */
void StepperFOC::setTarget(float_t target) { _target = target; }

/**
 * @brief Enables the motor driver outputs.
 */
void StepperFOC::enableMotor() {
    if (_debug) Serial.println("Motor enabled.");
    if (_motor) _motor->enable();
}

/**
 * @brief Disables the motor driver outputs.
 */
void StepperFOC::disableMotor() {
    if (_debug) Serial.println("Motor disabled.");
    if (_motor) _motor->disable();
}

/**
 * @brief Sets the PID gains for the position (angle) controller.
 * @param p Proportional gain.
 * @param i Integral gain.
 * @param d Derivative gain.
 */
void StepperFOC::setPositionGains(float p, float i, float d) {
    // Update the configuration struct.
    _motorConfig.pidAngleP = p;
    _motorConfig.pidAngleI = i;
    _motorConfig.pidAngleD = d;
    // Apply the new gains to the live motor object.
    if (_motor) {
        _motor->P_angle.P = p;
        _motor->P_angle.I = i;
        _motor->P_angle.D = d;
    }
}

/**
 * @brief Sets the PID gains for the velocity controller.
 * @param p Proportional gain.
 * @param i Integral gain.
 * @param d Derivative gain.
 */
void StepperFOC::setVelocityGains(float p, float i, float d) {
    _motorConfig.pidVelocityP = p;
    _motorConfig.pidVelocityI = i;
    _motorConfig.pidVelocityD = d;
    if (_motor) {
        _motor->PID_velocity.P = p;
        _motor->PID_velocity.I = i;
        _motor->PID_velocity.D = d;
    }
}

/**
 * @brief Sets the PI gains for the current (d/q axis) controllers.
 * @param p Proportional gain (applied to both d and q axes).
 * @param i Integral gain (applied to both d and q axes).
 */
void StepperFOC::setCurrentGains(float p, float i) {
    _motorConfig.pidCurrentQP = p;
    _motorConfig.pidCurrentQI = i;
    _motorConfig.pidCurrentDP = p; // Use same values for D-axis
    _motorConfig.pidCurrentDI = i;
    if (_motor) {
        _motor->PID_current_q.P = p;
        _motor->PID_current_q.I = i;
        _motor->PID_current_d.P = p;
        _motor->PID_current_d.I = i;
    }
}

/**
 * @brief Sets the software velocity limit for the motor.
 * @param limit The maximum allowed velocity in rad/s.
 */
void StepperFOC::setVelocityLimit(float limit) {
    _motorConfig.angleVelocityLimit = limit;
    if (_motor) _motor->velocity_limit = limit;
}

/**
 * @brief Sets the software current limit for the motor.
 * @param limit The maximum allowed current in Amps.
 */
void StepperFOC::setCurrentLimit(float limit) {
    _motorConfig.currentLimit = limit;
    if (_motor) _motor->current_limit = limit;
}

/**
 * @brief Sets the number of pole pairs for the motor.
 * @param poles The number of pole pairs (e.g., 50 for a 1.8deg stepper).
 */
void StepperFOC::setPolePairs(uint8_t poles) {
    _motorConfig.polePairs = poles;
    if (_motor) {
        _motor->pole_pairs = poles;
    }
    if (_debug) {
        Serial.print("Set Pole Pairs to: ");
        Serial.println(poles);
    }
}

/**
 * @brief Sets the power supply voltage for the driver.
 * @param voltage The main supply voltage in Volts.
 */
void StepperFOC::setPowerSupplyVoltage(float voltage) {
    _driverConfig.voltagePowerSupply = voltage;
    if (_driver) {
        _driver->voltage_power_supply = voltage;
    }
    if (_debug) {
        Serial.print("Set Power Supply Voltage to: ");
        Serial.println(voltage);
    }
}

/**
 * @brief Sets the voltage limit for the driver and scales the motor limit accordingly.
 * @param voltage The maximum voltage the driver can output in Volts.
 */
void StepperFOC::setDriverVoltageLimit(float voltage) {
    // Set the driver's voltage limit.
    _driverConfig.voltageLimit = voltage;
    if (_driver) {
        _driver->voltage_limit = voltage;
    }

    // Automatically set the motor's voltage limit to 50% of the driver's limit.
    float motor_voltage_limit = voltage / 2.0f;
    _motorConfig.voltageLimit = motor_voltage_limit;
    if (_motor) {
        _motor->voltage_limit = motor_voltage_limit;
    }

    if (_debug) {
        Serial.print("Set Driver Voltage Limit to: ");
        Serial.println(voltage);
        Serial.print("Automatically set Motor Voltage Limit to: ");
        Serial.println(motor_voltage_limit);
    }
}

/**
 * @brief Sets the time constant for the velocity low-pass filter.
 * @param tf The low-pass filter time constant in seconds.
 */
void StepperFOC::setVelocityLPF(float tf) {
    _motorConfig.lpfVelocity = tf;

    if (_motor) {
        _motor->LPF_velocity.Tf = tf;
    }

    if (_debug) {
        Serial.print("Set Velocity LPF Tf to: ");
        Serial.println(tf, 4);
    }
}