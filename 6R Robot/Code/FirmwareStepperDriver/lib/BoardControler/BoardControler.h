/*
================================================================================
File: BoardControler.h
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Header file for the BoardControler class. This class acts as the main 
application logic, coordinating the Board (hardware abstraction), CANbus 
(communication), and StepperFOC (motor control) classes. It implements a 
state machine to handle different control modes and processes incoming 
CAN commands.

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#pragma once

#include <cstdint>

#include "Board.h"
#include "CANbus.h"
#include "StepperFOC.h"

/**
 * @enum ProcessStatus
 * @brief Defines status codes for asynchronous process completion messages.
 */
enum class ProcessStatus : uint8_t {
    SUCCESS = 0x00,
    FAILURE_TIMEOUT = 0x01,
    FAILURE_INVALID_STATE = 0x02  
};

/**
 * @enum CommandType
 * @brief Defines the command IDs used in the CAN communication protocol.
 * The first byte of every CAN message payload corresponds to one of these commands.
 */
enum class CommandType : uint8_t {
    // --- Motion Commands ---
    SET_POSITION = 0x01,
    SET_VELOCITY = 0x02,
    SET_TORQUE = 0x03,
    SET_TRAJECTORY_POINT = 0x05,
    SET_SYNCED_TRAJECTORY_POINT = 0x06,
    SET_POSITION_RAMP = 0x07,

    // --- Configuration Commands - Position Controller ---
    SET_POS_P_GAIN = 0x10,
    SET_POS_I_GAIN = 0x11,
    SET_POS_D_GAIN = 0x12,

    // --- Configuration Commands - Velocity Controller ---
    SET_VEL_P_GAIN = 0x13,
    SET_VEL_I_GAIN = 0x14,
    SET_VEL_D_GAIN = 0x15,

    // --- Configuration Commands - Current Controller ---
    SET_CUR_P_GAIN = 0x16, // Sets P for both Q and D axes
    SET_CUR_I_GAIN = 0x17, // Sets I for both Q and D axes

    // --- Configuration Commands - Limits ---
    SET_VELOCITY_LIMIT = 0x18,
    SET_CURRENT_LIMIT = 0x19,

    // --- Configure Velocity Low-Pass-Filter
    SET_VEL_FILTER_TF = 0x1A,

    // --- State Control Commands ---
    ENABLE_MOTOR = 0x20,
    DISABLE_MOTOR = 0x21,
    START_HOMING_SEQUENCE = 0x22,
    ENABLE_OPEN_LOOP_MODE = 0x23,
    DISABLE_OPEN_LOOP_MODE = 0x24,

    // --- Telemetry Commands ---
    REQUEST_TELEMETRY = 0x30,
    TELEMETRY_DATA = 0x31,

    // --- Endstop Feedback ---
    SEND_ENDSTOP_VAL = 0x40,
    ENDSTOP_VAL = 0x41,

    // --- Fundamental Motor/Driver configuration ---
    SET_POLE_PAIRS = 0x50,
    SET_POWER_SUPPLY_VOLTAGE = 0x51,
    SET_DRIVER_VOLTAGE_LIMIT = 0x52,

    // --- Process Completion Feedback ---
    PROCESS_COMPLETE = 0x60,

    // --- Synchronization ---
    SYNC_NOW = 0x90
};

/**
 * @struct RemoteState
 * @brief  Holds the last known state received from a remote motor controller.
 * This struct is updated when feedback messages like telemetry are received.
 */
struct RemoteState {
    // Telemetry Data
    float last_received_position = 0.0f;
    float last_received_velocity = 0.0f;
    uint32_t telemetryUpdateCounter = 0;

    // Endstop State
    bool endstop_triggered = false;

    // Process Completion State
    CommandType last_completed_process = CommandType::ENABLE_MOTOR; // Default value
    ProcessStatus last_process_status = ProcessStatus::SUCCESS;
};

/**
 * @class BoardControler
 * @brief The main application class that orchestrates motor control logic.
 * It links together the hardware, communication, and motor control layers,
 * processes CAN commands, and manages the motor's state machine.
 */
class BoardControler {
public:
    // --- Linking and Initialization ---
    void linkBoard(Board* board);
    void linkCANBus(CANbus* canBus);
    void linkStepperFOC(StepperFOC* stepper);
    void activateDebug();
    void activateCANInterrupt();
    void activateMoveStepperInterrupt(uint32_t intervalMicros);
    void initAll();
    void loop();

    // --- CAN Command Sending ---
    bool sendCommand(CommandType type);
    bool sendCommand(CommandType type, float value);
    bool sendCommand(CommandType type, uint8_t value);
    bool sendCommand(CommandType type, float position, float velocity);

    // --- Public Setters for Control Logic ---
    void setPosition();
    void setVelocity();
    void setTorque();
    void setTrajectoryPoint();
    void setTargetTorque(float targetTorque);
    void setTargetVelocity(float targetVelocity);
    void setTargetPosition(float targetPosition);

    // --- Public Setters for Motor settings
    void setMotorControlType(MotionControlType motionType, FOCModulationType focModulation = FOCModulationType::SinePWM);
    void setMotorControlType(MotionControlType motionType, TorqueControlType torqueType, FOCModulationType focModulation = FOCModulationType::SinePWM);

    // --- Public Getter methods ---
    const RemoteState& getRemoteState() const;

    // --- Public Helper methods ---
    void clearLastProcessFeedback();

    // --- Public methods for open-loop control
    void enableOpenLoop();
    void disableOpenLoop();
    bool isOpenLoopEnabled() const;

private:
    /**
     * @enum ControlMode
     * @brief Internal state machine for the main control loop.
     */
    enum class ControlMode {
        POSITION,
        VELOCITY,
        TORQUE,
        TRAJECTORY,
        HOMING,
        RAMP_PROFILE,
        IDLE
    };

    /**
     * @enum HomingState
     * @brief Internal sub-state machine for the multi-step homing process.
     */
    enum class HomingState {
        IDLE,
        SEEKING_TRIGGER,
        SEEKING_RELEASE
    };

    /**
     * @enum ProfilePhase
     * @brief States for the analytical trajectory generator
     */
    enum class ProfilePhase { IDLE, ACCELERATING, CRUISING, DECELERATING };

    /**
     * @struct BufferedCommand
     * @brief Holds the data of a comand waiting for a SYNC signal.
     */
    struct BufferedCommand {
        CommandType type = CommandType::DISABLE_MOTOR;
        float p1 = 0, p2 = 0; // Generic parameters (e.g., position, velocity)
    };
    
    bool _debug = false;                        //!< Flag to enable/disable debug prints.

    // --- Timing and Timeout Variables ---
    uint32_t _moveStepperInterruptInterval = 0; //!< Interval of the high-frequency motion interrupt in microseconds.
    uint32_t _homingTimeoutCounter = 0;         //!< Countdown timer for the homing sequence.
    uint32_t _profileMoveTimeoutCounter = 0;    //!< Timeout counter for the ramp move

    // --- Target Setpoints ---
    float _targetPosition = 0.0f;               //!< Target position in radians.
    float _targetVelocity = 0.0f;               //!< Target velocity in rad/s.
    float _targetTorque = 0.0f;                 //!< Target torque (voltage or current).
    float _homingVelocity = 0.0f;               //!< Velocity used during the homing sequence.
    float _homingPos1 = 0.0f;                   //!< Position where the endstop is first triggered.
    float _homingPos2 = 0.0f;                   //!< Position where the endstop is released.

    // --- Motion Profiler Variables ---
    float _profileDefaultMaxVelocity = 10.0f;   //!< Default max. velocity of motion profile
    float _profileDefaultAcceleration = 2.0f;   //!< Defailt acceleration of motion profile
    float _profileTargetPosition = 0.0f;        //!< Target position of motin profile
    float _profileAcceleration = 0.0f;          //!< Acceleration of motion profile
    float _profileMaxVelocity = 0.0f;           //!< Max. Velocity of motion profile
    float _angleIntegralError = 0.0f;           //!< Integration of error for controller
    int _profileDirection = 1;                  //!< Moving direction of profile-motion
    const float _profilePositionTolerance = 0.001f; //!< Tolerance for position profile, defines when motion is reported as successfully finished
    const float _profileVelocityTolerance = 0.01; //!< Tolerance for velocity profile, defines when motion is reported as successfully finished

    // --- Variables for the analytical trajectory generator ---
    unsigned long _profileStartTimeUs = 0;      //!< Time when the profile was started in us
    float _profileInitialPosition = 0.0f;       //!< Position when the profile was started
    float _profileTimeAccel = 0.0f;             //!< Duration of acceleration
    float _profileTimeCruise = 0.0f;            //!< Duration of constant velocity
    float _profileDistAccel = 0.0f;             //!< Distance travelt during acceleration
    float _profileDistCruise = 0.0f;            //!< Distance travelt during constant velocity
    float _profileVelPeak = 0.0f;               //!< Peak velocity of profile, needed when max. velocity can not be reached

    // --- Variable for open-loop control
    bool _openLoopEnabled = false;              //!< Flag to enable and disable open-loop mode
    float _openLoopMotorVoltageLimit = 5.0f;    //!< Voltage limit for open loop, should be set based on the max. current of the motor


    // --- Linked Object Pointers ---
    Board* _board = nullptr;
    CANbus* _canBus = nullptr;
    StepperFOC* _stepper = nullptr;
    TIM_TypeDef* _moveStepperTimer = TIM6;      //!< STM32 Timer peripheral used for the motion interrupt.

    // --- State Machine Variables ---
    ControlMode _currentControlMode = ControlMode::IDLE;
    HomingState _homingState = HomingState::IDLE;
    ProfilePhase _profilePhase = ProfilePhase::IDLE;

    // --- Sync & Command Buffering Variables ---
    bool _awaitingSync = false;
    BufferedCommand _bufferedCommand;

    // --- State Storage for Remote Device ---
    RemoteState _remoteState;

    // --- Private Methods ---
    void CANCallback(FDCAN_RxHeaderTypeDef rxHeader, uint8_t* rxData);
    void moveStepperInterrupt();
    void executeHomingStep();
    void sendEndstopVal();
    void sendTelemetry();
    void sendProcessComplete(CommandType type, ProcessStatus status);
    void startHoming(float velocity);
    void bufferMotionCommand(CommandType type, float p1, float p2);
    void executeBufferedCommand();
    void startProfiledMove(float targetPos, float maxVel, float accel);
    void executeProfileStep();
};