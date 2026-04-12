/*
================================================================================
File: StepperFOC.h
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Header file for the StepperFOC class, which encapsulates all functionality for 
controlling a stepper motor using Field-Oriented Control (FOC) with the 
SimpleFOC library. It manages hardware configurations, motor parameters, 
and control objects.

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#pragma once

#include <SPI.h>
#include <optional>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5047/MagneticSensorAS5047.h"

/**
 * @struct SensorConfig
 * @brief Holds all configuration parameters related to sensors.
 */
struct SensorConfig {
    // -- Position Sensor (AS5047) --
    uint8_t posSensorCSpin = PA15;       //!< Chip select pin for the magnetic position sensor.
    uint8_t posSensorMOSIPin = PB5;      //!< SPI MOSI pin.
    uint8_t posSensorMISOPin = PB4;      //!< SPI MISO pin.
    uint8_t posSensorSCKPin = PB3;       //!< SPI Clock pin.

    // -- Current Sensor --
    float_t currentSensorGain = 910.0;   //!< Gain of the current sense amplifier (in V/A or mV/A, depending on library version/hardware).
    uint8_t currentSensorPhaseAPin = PB0;//!< Analog pin for measuring current of phase A.
    uint8_t currentSensorPhaseBPin = PB1;//!< Analog pin for measuring current of phase B.
};

/**
 * @struct DriverConfig
 * @brief Holds all configuration parameters for the motor driver hardware.
 */
struct DriverConfig {
    // -- Driver Pinout --
    uint8_t phaseAPin1 = PA3;            //!< PWM pin for phase A, output 1.
    uint8_t phaseBPin1 = PA2;            //!< PWM pin for phase B, output 1.
    uint8_t phaseAPin2 = PA1;            //!< PWM pin for phase A, output 2.
    uint8_t phaseBPin2 = PA0;            //!< PWM pin for phase B, output 2.
    uint8_t enableAPin = PA4;            //!< Enable pin for phase A driver bridge.
    uint8_t enableBPin = PA6;            //!< Enable pin for phase B driver bridge.

    // -- Driver Voltage Settings --
    float_t voltagePowerSupply = 24.0;   //!< The main power supply voltage provided to the driver.
    float_t voltageLimit = 24.0;         //!< The maximum voltage the driver is allowed to apply to the motor. Should not exceed power supply voltage.

    // -- PWM Settings --
    uint32_t pwmFrequency = 20000;       //!< PWM frequency for the motor driver, in Hz.
};

/**
 * @struct MotorConfig
 * @brief Holds all physical and control parameters for the stepper motor.
 */
struct MotorConfig {
    // -- Physical Properties --
    uint8_t polePairs = 50;                     //!< Number of pole pairs of the stepper motor. For a 1.8-degree stepper, this is 50.

    // -- Motor Voltage & Current Limits --
    float_t voltageLimit = 12.0;                //!< Per-motor voltage limit, often set to a fraction of the driver's limit.
    float_t voltageSensorAlign = 5.0;           //!< Voltage used during the sensor alignment phase in initFOC().
    float_t currentLimit = 2.0;                 //!< Maximum current (in Amps) the motor is allowed to draw.

    // -- PID Gains for Current Control (d/q axes) --
    // cutoff frequency roughly 1 kHz
    float_t pidCurrentQP = 17.0f;               //!< Proportional gain for the q-axis (torque) current controller.
    float_t pidCurrentQI = 9400.0f;             //!< Integral gain for the q-axis (torque) current controller.
    float_t pidCurrentDP = 17.0f;               //!< Proportional gain for the d-axis (flux) current controller.
    float_t pidCurrentDI = 9400.0f;             //!< Integral gain for the d-axis (flux) current controller.
    float_t pidCurrentQoutputRamp = 8000.0f;    //!< Ramp limit for the q-axis PID output.
    float_t pidCurrentDoutputRamp = 8000.0f;    //!< Ramp limit for the d-axis PID output.

    // -- PID Gains for Velocity Control --
    // float_t pidVelocityP = 0.5f;             //!< Proportional gain for the velocity controller. --> Nema 17
    // float_t pidVelocityP = 0.1f;             //!< Proportional gain for the velocity controller. --> Nema 23
    float_t pidVelocityP = 0.146f;              //!< roughly 100 Hz cutoff frequency
    // float_t pidVelocityI = 15.0f;            //!< Integral gain for the velocity controller. --> Nema 17
    // float_t pidVelocityI = 15.0f;            //!< Integral gain for the velocity controller --> Nema 23
    float_t pidVelocityI = 65.0f;               //!< roughly 100 Hz cutoff frequency
    // float_t pidVelocityD = 0.0000f;          //!< Derivative gain for the velocity controller -> Nema 23
    float_t pidVelocityD = 0.001f;              //!< Derivative gain for the velocity controller -> Nema17

    // -- PID Gains for Position/Angle Control --
    // float_t pidAngleP = 15.0f;               //!< Proportional gain for the angle controller.
    // float_t pidAngleI = 5.0f;                //!< Integral gain for the angle controller.
    float_t pidAngleP = 3940.0f;                //!< Roughly 10 Hz position controller
    float_t pidAngleI = 0.0f;
    // float_t pidAngleD = 0.000f;              //!< Derivative gain for the angle controller -> Nema 23
    float_t pidAngleD = 0.001f;                 //!< Derivative gain for the angle controller -> Nema 17
    
    // -- Low Pass Filter Time Constants (Tf in seconds) --
    float_t lpfCurrentQ = 0.001f;               //!< Time constant for the q-axis current measurement low-pass filter.
    float_t lpfCurrentD = 0.001f;               //!< Time constant for the d-axis current measurement low-pass filter.
    // float_t lpfVelocity = 0.005f;            //!< Time constant for the velocity calculation low-pass filter --> Nema 23
    float_t lpfVelocity = 0.04f;                //!< Time constant for the velocity calculation low-pass filter --> Nema 17

    // -- Position Control Limits --
    float_t angleOutputRamp = 10000.0;          //!< Ramp limit for the angle controller output (max velocity change).
    float_t angleVelocityLimit = 20.0;          //!< Maximum velocity allowed when in angle control mode.
};

/**
 * @struct ADCConfig
 * @brief Holds all configuration parameters for the STM32's ADC peripheral, based on HAL definitions.
 */
struct ADCConfig {
    // -- ADC Initialization Parameters --
    uint32_t ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    uint32_t Resolution = ADC_RESOLUTION_12B;
    uint32_t DataAlign = ADC_DATAALIGN_RIGHT;
    uint32_t GainCompensation = 0;
    uint32_t ScanConvMode = ADC_SCAN_DISABLE;
    uint32_t EOCSelection = ADC_EOC_SINGLE_CONV;
    uint32_t NbrOfConversion = 1;
    uint32_t ExternalTrigConv = ADC_SOFTWARE_START;
    uint32_t ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    uint32_t Overrun = ADC_OVR_DATA_PRESERVED;
    
    FunctionalState LowPowerAutoWait = DISABLE;
    FunctionalState ContinuousConvMode = DISABLE;
    FunctionalState DiscontinuousConvMode = DISABLE;
    FunctionalState DMAContinuousRequests = DISABLE;
    FunctionalState OversamplingMode = DISABLE;

    // -- ADC Channel Configuration Parameters --
    uint32_t Mode = ADC_MODE_INDEPENDENT;
    uint32_t Channel = ADC_CHANNEL_15;
    uint32_t Rank = ADC_REGULAR_RANK_1;
    uint32_t SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    uint32_t SingleDiff = ADC_SINGLE_ENDED;
    uint32_t OffsetNumber = ADC_OFFSET_NONE;
    uint32_t Offset = 0;
};

/**
 * @class StepperFOC
 * @brief A wrapper class to manage and control a stepper motor using SimpleFOC.
 */
class StepperFOC {
public:
    StepperFOC() = default;
    StepperFOC(uint8_t polePairs, float_t voltagePowerSupply, float_t voltageLimit, float_t voltageSensorAlign);
    ~StepperFOC();

    // --- Initialization & Setup ---
    void initStepperFOC(ADC_TypeDef* instance = ADC1);
    void activateDebug();

    // --- Control Type Setters ---
    void setControlType(MotionControlType motionType, FOCModulationType focModulation = FOCModulationType::SinePWM);
    void setControlType(MotionControlType motionType, TorqueControlType torqueType, FOCModulationType focModulation = FOCModulationType::SinePWM);

    // --- Setters ---
    void setSensorConfig(const SensorConfig& config);
    void setDriverConfig(const DriverConfig& config);
    void setMotorConfig(const MotorConfig& config);
    void setADCConfig(const ADCConfig& config);
    void setTarget(float_t target);
    
    void setPositionGains(float p, float i, float d);
    void setVelocityGains(float p, float i, float d);
    void setCurrentGains(float p, float i);
    void setVelocityLimit(float limit);
    void setCurrentLimit(float limit);
    void setVelocityLPF(float tf);
    void setPolePairs(uint8_t poles);
    void setPowerSupplyVoltage(float voltage);
    void setDriverVoltageLimit(float voltage);

    // --- Getters ---
    SensorConfig& getSensorConfig();
    DriverConfig& getDriverConfig();
    MotorConfig& getMotorConfig();
    ADCConfig& getADCConfig();
    StepperMotor* getMotor() const;
    MagneticSensorAS5047* getMagneticSensor() const;
    float_t getTarget();

    // --- Motor State ---
    void enableMotor();
    void disableMotor();

private:
    // --- Private Member Variables ---
    bool _debug = false;
    float_t _target = 0.0f;
    
    ADC_HandleTypeDef _hadc;

    SPIClass* _spiEncoder = nullptr;
    StepperDriver4PWM* _driver = nullptr;
    MagneticSensorAS5047* _magneticSensor = nullptr;
    LowsideCurrentSense* _currentSense = nullptr;
    StepperMotor* _motor = nullptr;

    SensorConfig _sensorConfig;
    DriverConfig _driverConfig;
    MotorConfig _motorConfig;
    ADCConfig _adcConfig;

    // --- Private Methods ---
    void initADC(ADC_TypeDef* instance);
};