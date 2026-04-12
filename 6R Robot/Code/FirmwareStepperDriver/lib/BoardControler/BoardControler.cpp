/*
================================================================================
File: BoardControler.cpp
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Implementation file for the BoardControler class. Contains the core logic for 
processing CAN commands, executing motion control profiles (position, velocity, 
torque, trajectories), and handling homing and open-loop sequences.

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#include "BoardControler.h"

/**
 * @brief Links the Board object to this controller.
 * @param board A pointer to the global Board instance.
 */
void BoardControler::linkBoard(Board* board) {
    _board = board;
    if (_debug) Serial.println("Board linked.");
}

/**
 * @brief Links the CANbus object to this controller.
 * @param canBus A pointer to the global CANbus instance.
 */
void BoardControler::linkCANBus(CANbus* canBus) {
    _canBus = canBus;
    if (_debug) Serial.println("CAN bus linked.");
}

/**
 * @brief Links the StepperFOC object to this controller.
 * @param stepper A pointer to the global StepperFOC instance.
 */
void BoardControler::linkStepperFOC(StepperFOC* stepper) {
    _stepper = stepper;
    if (_debug) Serial.println("Stepper FOC linked");
}

/**
 * @brief Activates debug prints for this class.
 */
void BoardControler::activateDebug() {
    _debug = true;
}

/**
 * @brief Configures and enables the CAN receive interrupt.
 */
void BoardControler::activateCANInterrupt() {
    if (_canBus) {
        // Use a lambda function to bind the member function as a callback.
        _canBus->setReceiveCallback([this](FDCAN_RxHeaderTypeDef rxHeader, uint8_t* rxData) {
            this->CANCallback(rxHeader, rxData);
        });
        _canBus->enableInterrupt();
    }
}

/**
 * @brief The main callback function for processing incoming CAN messages.
 * This function is executed in an interrupt context.
 * @param rxHeader The header of the received CAN message.
 * @param rxData A pointer to the data payload of the message.
 */
void BoardControler::CANCallback(FDCAN_RxHeaderTypeDef rxHeader, uint8_t* rxData) {
    // Safety check for valid pointers and data.
    if (!_stepper || !rxData || rxHeader.DataLength == 0 ) {
        if (_debug) Serial.println("ERR: Invalid MSG");
        return;
    }

    // Extract the command from the first byte of the payload.
    CommandType command = static_cast<CommandType>(rxData[0]);
    float value;
    uint8_t received_len = _canBus->getLengthFromDLC(rxHeader.DataLength);

    // Process the command using a state machine.
    switch (command) {
        // --- Motion Commands ---
        case CommandType::SET_POSITION: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&_targetPosition, &rxData[1], sizeof(float));
            _currentControlMode = ControlMode::POSITION;
            if (_debug) { Serial.print("CMD: POS, TGT: "); Serial.println(_targetPosition); }
            break;
        }
        case CommandType::SET_VELOCITY: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&_targetVelocity, &rxData[1], sizeof(float));
            _currentControlMode = ControlMode::VELOCITY;
            if (_debug) { Serial.print("CMD: VEL, TGT: "); Serial.println(_targetVelocity); }
            break;
        }
        case CommandType::SET_TORQUE: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&_targetTorque, &rxData[1], sizeof(float));
            _currentControlMode = ControlMode::TORQUE;
            if (_debug) { Serial.print("CMD: TRQ, TGT: "); Serial.println(_targetTorque); }
            break;
        }
        case CommandType::SET_TRAJECTORY_POINT: {
            if (received_len < 8) break;
            memcpy(&_targetPosition, &rxData[1], sizeof(float));
            int32_t velocity_raw = (rxData[7] << 16) | (rxData[6] << 8) | rxData[5];
            if (velocity_raw & 0x00800000) velocity_raw |= 0xFF000000; // Sign extend
            _targetVelocity = static_cast<float>(velocity_raw) / 1000.0f;
            _currentControlMode = ControlMode::TRAJECTORY;
            // if (_debug) { // This print can be too slow for high-frequency trajectory updates
            //     Serial.print("CMD: TRAJ, P:"); Serial.print(_targetPosition);
            //     Serial.print(", V:"); Serial.println(_targetVelocity);
            // }
            break;
        }
        case CommandType::SEND_ENDSTOP_VAL: {
            if (_debug) Serial.println("CMD: GET_ESTOP");
            sendEndstopVal();
            break;
        }
        case CommandType::SET_SYNCED_TRAJECTORY_POINT: {
            if (received_len < 8) break;
            float pos, vel;
            memcpy(&pos, &rxData[1], sizeof(float));
            // Reconstruct the 24-bit signed velocity value
            int32_t velocity_raw = (rxData[7] << 16) | (rxData[6] << 8) | rxData[5];
            if (velocity_raw & 0x00800000) velocity_raw |= 0xFF000000; // Sign extend
            vel = static_cast<float>(velocity_raw) / 1000.0f;

            // Buffer this command and wait for a SYNC_NOW signal
            bufferMotionCommand(command, pos, vel);
            break;
        }
        case CommandType::SYNC_NOW: {
            // This print can be to slow if the synchronization is done at a high frequency
            // if (_debug) Serial.println("CMD: SYNC_NOW received");
            // Execute the previously buffered command
            executeBufferedCommand();
            break;
        }
        case CommandType::SET_POSITION_RAMP: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));

            if (_debug) {Serial.print("CMD: SET_POS_RAMP, TGT: "); Serial.println(value);}

            startProfiledMove(value, _profileDefaultMaxVelocity, _profileDefaultAcceleration);
            _currentControlMode = ControlMode::RAMP_PROFILE;
            break;
        }

        // --- Configuration Commands ---
        case CommandType::SET_POS_P_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setPositionGains(value, _stepper->getMotorConfig().pidAngleI, _stepper->getMotorConfig().pidAngleD); 
            if (_debug) { Serial.print("CMD: SET_POS_P, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_POS_I_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setPositionGains(_stepper->getMotorConfig().pidAngleP, value, _stepper->getMotorConfig().pidAngleD); 
            if (_debug) { Serial.print("CMD: SET_POS_I, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_POS_D_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setPositionGains(_stepper->getMotorConfig().pidAngleP, _stepper->getMotorConfig().pidAngleI, value); 
            if (_debug) { Serial.print("CMD: SET_POS_D, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_VEL_P_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setVelocityGains(value, _stepper->getMotorConfig().pidVelocityI, _stepper->getMotorConfig().pidVelocityD); 
            if (_debug) { Serial.print("CMD: SET_VEL_P, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_VEL_I_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setVelocityGains(_stepper->getMotorConfig().pidVelocityP, value, _stepper->getMotorConfig().pidVelocityD); 
            if (_debug) { Serial.print("CMD: SET_VEL_I, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_VEL_D_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setVelocityGains(_stepper->getMotorConfig().pidVelocityP, _stepper->getMotorConfig().pidVelocityI, value); 
            if (_debug) { Serial.print("CMD: SET_VEL_D, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_CUR_P_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setCurrentGains(value, _stepper->getMotorConfig().pidCurrentQI); 
            if (_debug) { Serial.print("CMD: SET_CUR_P, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_CUR_I_GAIN: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setCurrentGains(_stepper->getMotorConfig().pidCurrentQP, value); 
            if (_debug) { Serial.print("CMD: SET_CUR_I, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_VELOCITY_LIMIT: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setVelocityLimit(value); 
            if (_debug) { Serial.print("CMD: SET_VEL_LIM, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_CURRENT_LIMIT: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setCurrentLimit(value);
            if (_debug) { Serial.print("CMD: SET_CUR_LIM, V:"); Serial.println(value); }
            break;
        }

        case CommandType::SET_VEL_FILTER_TF: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));

            if (value < 0.0f) value = 0.0f;

            _stepper->setVelocityLPF(value);
            if (_debug) {
                Serial.print("CMD: SET_VEL_LPF, V:");
                Serial.println(value, 4);
            }
            break;
        }

        // --- State Commands ---
        case CommandType::ENABLE_MOTOR: {
            if (_debug) Serial.println("CMD: ENABLE");
            _stepper->enableMotor(); 
            break;
        }
        case CommandType::DISABLE_MOTOR: {
            if (_debug) Serial.println("CMD: DISABLE");
            _stepper->disableMotor();
            _currentControlMode = ControlMode::POSITION; // Revert to a safe mode.
            break;
        }
        case CommandType::START_HOMING_SEQUENCE:{
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            if (_debug) { Serial.print("CMD: HOMING, V:"); Serial.println(value); }
            startHoming(value);
            break;
        }

        case CommandType::ENABLE_OPEN_LOOP_MODE: {
            if (_debug) Serial.println("CMD: ENABLE OPEN LOOP");
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            if (value > _stepper->getDriverConfig().voltagePowerSupply) value = _stepper->getDriverConfig().voltagePowerSupply;
            if (value < 0) value = 0.0;
            _openLoopMotorVoltageLimit = value;
            enableOpenLoop();
            break;
        }

        case CommandType::DISABLE_OPEN_LOOP_MODE: {
            if (_debug) Serial.println("CMD: DISABLE OPEN LOOP");
            disableOpenLoop();
            break;
        }
        
        // --- Telemetry Commands ---
        case CommandType::REQUEST_TELEMETRY: {
            if (_debug) Serial.println("CMD: REQ_TLM");
            sendTelemetry(); 
            break;
        }

        // --- Handling for feedback messages ---
        case CommandType::TELEMETRY_DATA: {
            if (received_len < 8) break; // Expect 1 byte command + 4 pos + 3 vel

            // Unpack position (float) from bytes 1-4
            memcpy(&_remoteState.last_received_position, &rxData[1], sizeof(float));

            // Unpack velocity (int24) from bytes 5-7
            int32_t vel_raw = (rxData[7] << 16) | (rxData[6] << 8) | rxData[5];
            if (vel_raw & 0x00800000) vel_raw |= 0xFF000000; // Sign extend
            _remoteState.last_received_velocity = static_cast<float>(vel_raw) / 1000.0f;

            // Update counter
            _remoteState.telemetryUpdateCounter++;

            // if (_debug) {
            //     Serial.print("RX< TELEMETRY, P:"); Serial.print(_remoteState.last_received_position);
            //     Serial.print(", V:"); Serial.println(_remoteState.last_received_velocity);
            // }
            break;
        }

        case CommandType::ENDSTOP_VAL: {
            if (received_len < 2) break; // Expect 1 byte command + 1 byte state
            _remoteState.endstop_triggered = (rxData[1] == 0x01);
            if (_debug) {
                Serial.print("RX< ENDSTOP_VAL, State:");
                Serial.println(_remoteState.endstop_triggered ? "TRIGGERED" : "NOT TRIGGERED");
            }
            break;
        }

        case CommandType::PROCESS_COMPLETE: {
            if (received_len < 3) break; // Expect 1 cmd + 1 type + 1 status
            _remoteState.last_completed_process = static_cast<CommandType>(rxData[1]);
            _remoteState.last_process_status = static_cast<ProcessStatus>(rxData[2]);
            if (_debug) {
                Serial.print("RX< PROC_COMPLETE, Type: 0x"); Serial.print((uint8_t)_remoteState.last_completed_process, HEX);
                Serial.print(", Status:0x"); Serial.println((uint8_t)_remoteState.last_process_status, HEX);
            }
            break;
        }

        // --- Fundamental Config ---
        case CommandType::SET_POLE_PAIRS: {
            if (received_len < 2) break;
            uint8_t pole_pairs = rxData[1];
            _stepper->setPolePairs(pole_pairs);
            if (_debug) { Serial.print("CMD: SET_POLES, V:"); Serial.println(pole_pairs); }
            break;
        }
        case CommandType::SET_POWER_SUPPLY_VOLTAGE: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setPowerSupplyVoltage(value);
            if (_debug) { Serial.print("CMD: SET_PWR_V, V:"); Serial.println(value); }
            break;
        }
        case CommandType::SET_DRIVER_VOLTAGE_LIMIT: {
            if (received_len < 1 + sizeof(float)) break;
            memcpy(&value, &rxData[1], sizeof(float));
            _stepper->setDriverVoltageLimit(value);
            if (_debug) { Serial.print("CMD: SET_DRV_V_LIM, V:"); Serial.println(value); }
            break;
        }

        default:
            if (_debug) {
                // This handles all commands that are SENT but not received (like REQUEST_TELEMETRY)
                // and unknown commands.
                Serial.print("CMD: UNKNOWN/UNHANDLED, ID:0x");
                Serial.println(uint8_t(rxData[0]), HEX);
            }
            break;
    }
}

/**
 * @brief Prepares the motor target for position control mode.
 */
void BoardControler::setPosition() {

    MotionControlType controlMode;

    if (_openLoopEnabled) {
        controlMode = MotionControlType::angle_openloop;
    } else {
        controlMode = MotionControlType::angle;
    }

    // Ensure the motor is in the correct low-level control mode.
    if (_stepper->getMotor()->controller != controlMode) {
        if (_debug) {
            Serial.print("Switching to ");
            Serial.println(_openLoopEnabled ? "angle OPEN loop" : "angle CLOSED loop");
        }
        _stepper->setControlType(controlMode);
    }
    // Set the target for the main move() call.
    _stepper->setTarget(_targetPosition);
}

/**
 * @brief Prepares the motor target for velocity control mode.
 */
void BoardControler::setVelocity() {

    MotionControlType controlType;

    if (_openLoopEnabled) {
        controlType = MotionControlType::velocity_openloop;
    } else {
        controlType = MotionControlType::velocity;
    }

    if (_stepper->getMotor()->controller != controlType) {
        if (_debug) {
            Serial.print("Switching to ");
            Serial.println(_openLoopEnabled ? "velocity OPEN loop" : "velocity closed loop");
        }
        _stepper->setControlType(controlType);
    }
    _stepper->setTarget(_targetVelocity);
}

/**
 * @brief Prepares the motor target for torque control mode.
 */
void BoardControler::setTorque() {
    if (_stepper->getMotor()->controller != MotionControlType::torque) {
        if (_debug) Serial.println("Switching to torque control mode.");
        _stepper->setControlType(MotionControlType::torque, TorqueControlType::foc_current);
    }
    _stepper->setTarget(_targetTorque);
}

/**
 * @brief Implements the position controller with velocity feed-forward for trajectory following.
 */
void BoardControler::setTrajectoryPoint() {
    if (!_stepper) return;
    StepperMotor* motor = _stepper->getMotor();

    // --- Check if open-loop mode is avtivated
    if (_openLoopEnabled) {
        // check if MotionControlType is angle-openloop
        if (motor->controller != MotionControlType::angle_openloop) {
            _stepper->setControlType(MotionControlType::angle_openloop);
        }

        // set position
        _stepper->setTarget(_targetPosition);

        // open loop finished
        return;
    }
    
    if (motor->controller != MotionControlType::velocity) {
        _stepper->setControlType(MotionControlType::velocity);
    }
    
    // --- PD Controller Calculation ---

    // Position Controller Gains
    float pGain = _stepper->getMotorConfig().pidAngleP;
    float dGain = _stepper->getMotorConfig().pidAngleD;
    float iGain = _stepper->getMotorConfig().pidAngleI;

    // Proportional (P) part: Corrects based on the distance to the target position.
    // This provides the main "pull" towards the tharget
    float posError = _targetPosition - motor->shaftAngle();
    float currVelocity = motor->shaftVelocity();
    float velFromPosLoop = pGain * posError;

    // Integral (I) part: only activated if the planned velocity is equal to zero
    float dt = _moveStepperInterruptInterval / 1000000.0f;
    if (fabs(_targetVelocity) < _profileVelocityTolerance) {
        _angleIntegralError += posError * dt;
    } else {
        _angleIntegralError = 0;
    }

    float integralLimit = motor->velocity_limit;
    float velFromIntLoop = iGain * _angleIntegralError;
    velFromIntLoop = constrain(velFromIntLoop, -integralLimit, integralLimit);

    // Derivative (D) part: Corrects based on the current velocity of the motor.
    // This acts as a "damper" or "brake" to prevent the motor from overshooting the target.
    float velFromVelLoop = dGain * currVelocity;

    // The total corrective velocity is the P-correction minus the D-damping.
    float corrVel = velFromPosLoop + velFromIntLoop - velFromVelLoop;

    // Feed-forward part: The planned velocity from the trajectory is added proactively.
    // This makes the motor follow the trajectory without lag.
    float finalVelTarget = corrVel + _targetVelocity;

    // Constrain the final command to the motor's velocity limit.
    float velLimit = motor->velocity_limit;
    finalVelTarget = constrain(finalVelTarget, -velLimit, velLimit);
    _stepper->setTarget(finalVelTarget);
}

/**
 * @brief Getter for the remote state struct.
 * @return const RemoteState& Read-only reference to the current RemoteState.
 */
const RemoteState& BoardControler::getRemoteState() const {
    return _remoteState;
}

/**
 * @brief Configures and starts the high-frequency hardware timer for motion control.
 * @param intervalMicros The interval for the interrupt in microseconds.
 */
void BoardControler::activateMoveStepperInterrupt(uint32_t intervalMicros) {
    _moveStepperInterruptInterval = intervalMicros;
    
    if (!_stepper || !_board) {
        if (_debug) Serial.println("No StepperFOC or Board object linked.");
        while(true);
    }

    // Use the Board class to start the timer with a lambda function as a callback.
    _board->startHardwareTimer(_moveStepperTimer, intervalMicros, [this]() {
        this->moveStepperInterrupt();
    });
}

/**
 * @brief The main high-frequency motion control interrupt service routine.
 * @details This function is called by the hardware timer at a fixed interval.
 */
void BoardControler::moveStepperInterrupt() {
    // Run the state machine to determine the control action.
    switch (_currentControlMode) {
        case ControlMode::POSITION:
            setPosition();
            break;
        case ControlMode::VELOCITY:
            setVelocity();
            break;
        case ControlMode::TORQUE:
            setTorque();
            break;
        case ControlMode::TRAJECTORY:
            setTrajectoryPoint();
            break;
        case ControlMode::HOMING:
            executeHomingStep();
            return; // Homing has its own move() call, so we exit early.
        case ControlMode::RAMP_PROFILE:
            if (_profileMoveTimeoutCounter > 0) {
                _profileMoveTimeoutCounter--;
            } else {
                if (_debug) Serial.println("Ramp Move FAILED: Timeout reached.");
                _currentControlMode = ControlMode::POSITION;
                _targetPosition = _stepper->getMotor()->shaftAngle();
                sendProcessComplete(CommandType::SET_POSITION_RAMP, ProcessStatus::FAILURE_TIMEOUT);
                return;
            }
            // Step 1: Generate the next point on the trajectory porifle
            executeProfileStep();
            // Step 2: Use the trajectory follower to set the motor's target
            if (_currentControlMode == ControlMode::RAMP_PROFILE) setTrajectoryPoint();
            break;
        case ControlMode::IDLE:
            break;
    }

    // The final, common move command for standard modes.
    _stepper->getMotor()->move(_stepper->getTarget());
}

/**
 * @brief Initializes all linked components.
 */
void BoardControler::initAll() {
    // Activate debug modes if enabled.
    if (_debug) {
        if (_canBus) _canBus->activateDebug();
        if (_board) _board->activateDebug();
        if (_stepper) _stepper->activateDebug();
    }

    // Initialize the CAN bus with a filter for this board's ID.
    if (_canBus) {
        _canBus->addFilter(_board->getCANReceiveID(), 0x7FF); // Mask 0x7FF accepts only this specific ID.
        _canBus->initCANbus(_board->getCANBaudrate());
    }
   
    // Initialize the motor and FOC algorithm and set Position to zero.
    if (_stepper) _stepper->initStepperFOC();
}

/**
 * @brief Main loop function, called repeatedly from Arduino's loop().
 * @details This is used for non-interrupt-critical tasks and runs the FOC algorithm.
 */
void BoardControler::loop() {
    // This function is crucial as it calls the FOC algorithm's main processing loop.
    _stepper->getMotor()->loopFOC();
}

/**
 * @brief Sends a simple command over CAN without payload.
 * @param type The CommandType enum to send.
 * @return true If the message was successfully queued for transmission.
 * @return false If transmission failed or CAN is not linked.
 */
bool BoardControler::sendCommand(CommandType type) {
    if (!_canBus) return false;
    uint8_t commandByte = static_cast<uint8_t>(type);
    return _canBus->sendMessage(_board->getCANSendID(), &commandByte, 1);
}

/**
 * @brief Sends a command over CAN with a float payload.
 * @param type The CommandType enum to send.
 * @param value The float value to append as payload.
 * @return true If successful.
 * @return false If failed.
 */
bool BoardControler::sendCommand(CommandType type, float value) {
    if (!_canBus) return false;
    uint8_t data[5];
    data[0] = static_cast<uint8_t>(type);
    memcpy(&data[1], &value, sizeof(float));
    return _canBus->sendMessage(_board->getCANSendID(), data, sizeof(data));
}

/**
 * @brief Sends a command over CAN with a uint8_t payload.
 * @param type The CommandType enum to send.
 * @param value The 8-bit unsigned integer to append as payload.
 * @return true If successful.
 * @return false If failed.
 */
bool BoardControler::sendCommand(CommandType type, uint8_t value) {
    if (!_canBus) return false;
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(type);
    data[1] = value;
    return _canBus->sendMessage(_board->getCANSendID(), data, sizeof(data));
}

/**
 * @brief Sends a dual-payload command over CAN (Position and Velocity).
 * @param type The CommandType enum to send.
 * @param position The position target in float.
 * @param velocity The velocity target in float.
 * @return true If successful.
 * @return false If failed.
 */
bool BoardControler::sendCommand(CommandType type, float position, float velocity) {
    if (!_canBus) return false;
    uint8_t data[8];
    data[0] = static_cast<uint8_t>(type);

    memcpy(&data[1], &position, sizeof(float));

    int32_t vel_scaled = static_cast<int32_t>(velocity * 1000.0f);
    data[5] = vel_scaled & 0xFF;
    data[6] = (vel_scaled >> 8) & 0xFF;
    data[7] = (vel_scaled >> 16) & 0xFF;

    return _canBus->sendMessage(_board->getCANSendID(), data, sizeof(data));
}

/**
 * @brief Sends telemetry data (position and velocity) over CAN.
 */
void BoardControler::sendTelemetry() {
    if (!_canBus || !_stepper) return;

    uint8_t data[8];
    data[0] = static_cast<uint8_t>(CommandType::TELEMETRY_DATA);

    float currentPos = _stepper->getMotor()->shaftAngle(); 
    memcpy(&data[1], &currentPos, sizeof(float));

    float currentVel = _stepper->getMotor()->shaftVelocity();
    int32_t vel_scaled = static_cast<int32_t>(currentVel * 1000.0f);

    data[5] = vel_scaled & 0xFF;
    data[6] = (vel_scaled >> 8) & 0xFF;
    data[7] = (vel_scaled >> 16) & 0xFF;

    _canBus->sendMessage(_board->getCANSendID(), data, sizeof(data));
}

/**
 * @brief Executes one step of the homing state machine.
 * @details This is called repeatedly by the motion interrupt while in HOMING mode.
 */
void BoardControler::executeHomingStep() {
    if (!_stepper || !_board) return;
    StepperMotor* motor = _stepper->getMotor();

    // Handle timeout countdown.
    if (_homingTimeoutCounter > 0) {
        _homingTimeoutCounter--;
    } else {
        if (_debug) Serial.println("Homing FAILED: Timeout reached.");
        _targetPosition = motor->shaftAngle(); // Hold current position on failure.
        _homingState = HomingState::IDLE;
        _currentControlMode = ControlMode::POSITION;
        sendProcessComplete(CommandType::START_HOMING_SEQUENCE, ProcessStatus::FAILURE_TIMEOUT);
        return;
    }

    // Set motor to velocity mode and command the homing velocity.
    motor->controller = MotionControlType::velocity;
    motor->move(_homingVelocity);

    // Read endstop value (assuming 0 = triggered, 1 = not triggered).
    GPIOPin endstop = _board->getEndstopPin();
    bool endstopTriggered = (_board->readGPIOPin(endstop.port, endstop.pin) == 0);

    // Homing sub-state machine.
    switch (_homingState) {
        case HomingState::SEEKING_TRIGGER:
            if (endstopTriggered) {
                _homingPos1 = motor->shaftAngle();
                if (_debug) {
                    Serial.print("Homing: Trigger point found at ");
                    Serial.println(_homingPos1);
                }
                _homingState = HomingState::SEEKING_RELEASE; // Move to the next state.
            }
            break;

        case HomingState::SEEKING_RELEASE:
            if (!endstopTriggered) {
                _homingPos2 = motor->shaftAngle();
                if (_debug) {
                    Serial.print("Homing: Release point found at ");
                    Serial.println(_homingPos2);
                }
                
                motor->move(0); // Stop the motor.

                // Calculate the center point between trigger and release.
                float zeroPoint = (_homingPos1 + _homingPos2) / 2.0f;
                
                // Adjust the motor's internal angle to make the center point the new zero.
                motor->sensor_offset = zeroPoint;

                if (_debug) {
                    Serial.print("Homing SUCCESS: New zero point established. Current angle is now ");
                    Serial.println(motor->shaft_angle);
                }

                _targetPosition = 0; // Set target to the new zero.
                _homingState = HomingState::IDLE;
                _currentControlMode = ControlMode::POSITION; // Revert to a safe mode.
                sendProcessComplete(CommandType::START_HOMING_SEQUENCE, ProcessStatus::SUCCESS);
            }
            break;

        case HomingState::IDLE:
            _currentControlMode = ControlMode::POSITION;
            break;
    }
}

/**
 * @brief Reads the endstop state and sends it over CAN.
 */
void BoardControler::sendEndstopVal() {
    if (!_canBus || !_board) {
        if (_debug) Serial.println("Error: CAN bus or Board object not linked.");
        return;
    }
    
    GPIOPin endstopPin = _board->getEndstopPin();
    bool pinState = _board->readGPIOPin(endstopPin.port, endstopPin.pin);
    
    uint8_t data[2];
    const uint8_t length = 2;

    data[0] = static_cast<uint8_t>(CommandType::ENDSTOP_VAL);
    data[1] = pinState ? 0x01 : 0x00; // 1 for not triggered, 0 for triggered

    if (_debug) {
        Serial.print("Sending Endstop Value: ");
        Serial.println(pinState ? "NOT TRIGGERED" : "TRIGGERED");
    }
    _canBus->sendMessage(_board->getCANSendID(), data, length);
}

/**
 * @brief Sends a process completion message (e.g., for homing) over CAN.
 * @param type The type of process that was completed.
 * @param status The completion status (e.g., success, failure).
 */
void BoardControler::sendProcessComplete(CommandType type, ProcessStatus status) {
    if (!_canBus || !_board) return;

    uint8_t data[3];
    data[0] = static_cast<uint8_t>(CommandType::PROCESS_COMPLETE);
    data[1] = static_cast<uint8_t>(type); // NOTE: This should likely be a ProcessType enum.
    data[2] = static_cast<uint8_t>(status);

    _canBus->sendMessage(_board->getCANSendID(), data, sizeof(data));
}

/**
 * @brief Initiates the homing sequence.
 * @param velocity The velocity to use for the homing movement.
 */
void BoardControler::startHoming(float velocity) {
    if (velocity == 0.0f) {
        sendProcessComplete(CommandType::START_HOMING_SEQUENCE, ProcessStatus::FAILURE_INVALID_STATE);
        return;
    }

    if (_debug) {
        Serial.print("Homing sequence initiated with velocity: ");
        Serial.println(velocity);
    }
    
    _homingVelocity = velocity;

    // Calculate a dynamic timeout based on making 30 full revolutions.
    const float revolutionsForTimeout = 30.0;
    const float totalAngle = revolutionsForTimeout * 2.0 * PI; // in Radians
    const float timeForTimeout_s = totalAngle / fabs(velocity); // in Seconds
    const float interval_s = _moveStepperInterruptInterval / 1000000.0;
    _homingTimeoutCounter = static_cast<uint32_t>(timeForTimeout_s / interval_s);
    
    if (_debug) {
        Serial.print("Homing Timeout set to ");
        Serial.print(_homingTimeoutCounter);
        Serial.println(" interrupt cycles.");
    }
    
    _currentControlMode = ControlMode::HOMING;
    _homingState = HomingState::SEEKING_TRIGGER; // Set the initial sub-state.
}

/**
 * @brief Buffers a motion command to await a SYNC signal for execution.
 * @param type The command type to buffer (should be SET_SYNCED_TRAJECTORY_POINT).
 * @param p1 First parameter (position).
 * @param p2 Second parameter (velocity).
 */
void BoardControler::bufferMotionCommand(CommandType type, float p1, float p2) {
    _bufferedCommand.type = type;
    _bufferedCommand.p1 = p1;
    _bufferedCommand.p2 = p2;
    _awaitingSync = true;
    if (_debug) { Serial.print("CMD buffered for SYNC. Type: 0x"); Serial.println((uint8_t)type, HEX); }
}

/**
 * @brief Executes the command that was previously buffered.
 */
void BoardControler::executeBufferedCommand() {
    if (!_awaitingSync) return; // Do nothing if no command is waiting

    if (_debug) { Serial.print("Executing buffered CMD: 0x"); Serial.println((uint8_t)_bufferedCommand.type, HEX); }

    switch (_bufferedCommand.type) {
        case CommandType::SET_SYNCED_TRAJECTORY_POINT:
            // Set the target values from the buffer
            _targetPosition = _bufferedCommand.p1;
            _targetVelocity = _bufferedCommand.p2;
            // Switch the state machine to TRAJECTORY mode
            _currentControlMode = ControlMode::TRAJECTORY;
            break;
        // No other cases needed as only one command is syncable
        default:
            break;
    }
    // Clear the flag so the command is not executed again
    _awaitingSync = false;
}

/**
 * @brief The Planner: Pre-calculates all phases of a new ramp profile move.
 * @param targetPos Target position of trajectory
 * @param maxVel Maximum velocity for motion profile
 * @param accel Acceleration of motion profile
 */
void BoardControler::startProfiledMove(float targetPos, float maxVel, float accel) {
    if (accel == 0.0f) return;

    _profileInitialPosition = _stepper->getMotor()->shaftAngle();
    _profileTargetPosition = targetPos;
    _profileMaxVelocity = fabs(maxVel);
    _profileAcceleration = fabs(accel);

    float totalDistance = fabs(_profileTargetPosition - _profileInitialPosition);
    if (totalDistance < 0.001f) { // Movement is too small to plan
        _profilePhase = ProfilePhase::IDLE;
        return;
    }
    _profileDirection = (_profileTargetPosition > _profileInitialPosition) ? 1 : -1;

    // Time required to reach max velocity
    float timeToReachMaxV = _profileMaxVelocity / _profileAcceleration;
    // Distance covered during acceleration to max velocity
    float distanceToAccel = 0.5f * _profileAcceleration * timeToReachMaxV * timeToReachMaxV;

    if (totalDistance < 2.0f * distanceToAccel) {
        // --- Triangular Profile ---
        _profileTimeAccel = sqrt(totalDistance / _profileAcceleration);
        _profileVelPeak = _profileTimeAccel * _profileAcceleration;
        _profileDistAccel = totalDistance / 2.0f;
        _profileTimeCruise = 0.0f;
        _profileDistCruise = 0.0f;
    } else {
        // --- Trapezoidal Profile ---
        _profileTimeAccel = timeToReachMaxV;
        _profileVelPeak = _profileMaxVelocity;
        _profileDistAccel = distanceToAccel;
        _profileDistCruise = totalDistance - 2.0f * _profileDistAccel;
        _profileTimeCruise = _profileDistCruise / _profileVelPeak;
    }

    _angleIntegralError = 0.0f;
    _profileStartTimeUs = micros();
    _profilePhase = ProfilePhase::ACCELERATING;
    _currentControlMode = ControlMode::RAMP_PROFILE;
    
    // The timeout calculation remains a useful safety feature
    float timeTotalS = 2.0f * _profileTimeAccel + _profileTimeCruise;
    float timeoutS = timeTotalS * 1.5f + 1.0f; // 50% buffer + 1 second fixed
    _profileMoveTimeoutCounter = static_cast<uint32_t>(timeoutS * 1000000.0f / _moveStepperInterruptInterval);
}

/**
 * @brief Calculates the exact target position and velocity
 * based on the elapsed time since the start of the move.
 */
void BoardControler::executeProfileStep() {
    // Elapsed time since the start of the move, in seconds
    float t = (micros() - _profileStartTimeUs) / 1000000.0f; 

    // Check for phase transitions
    if (_profilePhase == ProfilePhase::ACCELERATING && t >= _profileTimeAccel) {
        _profilePhase = (_profileTimeCruise > 0) ? ProfilePhase::CRUISING : ProfilePhase::DECELERATING;
    }
    if (_profilePhase == ProfilePhase::CRUISING && t >= (_profileTimeAccel + _profileTimeCruise)) {
        _profilePhase = ProfilePhase::DECELERATING;
    }

    // Calculate setpoints analytically based on the current phase
    switch (_profilePhase) {
        case ProfilePhase::ACCELERATING: {
            // v(t) = a * t
            _targetVelocity = _profileAcceleration * t * _profileDirection;
            // p(t) = p0 + 0.5 * a * t^2
            _targetPosition = _profileInitialPosition + 0.5f * _profileAcceleration * t * t * _profileDirection;
            break;
        }
        case ProfilePhase::CRUISING: {
            float tCruisePhase = t - _profileTimeAccel;
            // v(t) = v_peak
            _targetVelocity = _profileVelPeak * _profileDirection;
            // p(t) = p_accel + v_peak * t_cruise
            _targetPosition = _profileInitialPosition + (_profileDistAccel + _profileVelPeak * tCruisePhase) * _profileDirection;
            break;
        }
        case ProfilePhase::DECELERATING: {
            float tDecelPhase = t - _profileTimeAccel - _profileTimeCruise;
            // v(t) = v_peak - a * t
            _targetVelocity = (_profileVelPeak - _profileAcceleration * tDecelPhase) * _profileDirection;
            // p(t) = p_cruise + v_peak*t - 0.5*a*t^2
            _targetPosition = _profileInitialPosition + (_profileDistAccel + _profileDistCruise + _profileVelPeak * tDecelPhase - 0.5f * _profileAcceleration * tDecelPhase * tDecelPhase) * _profileDirection;
            break;
        }
        case ProfilePhase::IDLE: {
            _targetPosition = _profileTargetPosition;
            _targetVelocity = 0.0f;
            break;
        }
    }
    
    // --- Final Completion Check ---
    // This checks if the PHYSICAL motor has arrived at the destination.
    bool isFinished = false;
    float positionError = fabs(_profileTargetPosition - _stepper->getMotor()->shaftAngle());
    if (positionError < _profilePositionTolerance && fabs(_stepper->getMotor()->shaftVelocity()) < _profileVelocityTolerance) {
        isFinished = true;
    }

    // If the planned time has elapsed OR the motor has physically arrived, end the move.
    if (t >= (_profileTimeAccel * 2.0f + _profileTimeCruise) || isFinished) {
        _targetPosition = _profileTargetPosition; // Clamp to the exact final value
        _targetVelocity = 0.0f;
        _profilePhase = ProfilePhase::IDLE; // Prevent further calculations

        // The success check remains
        if (isFinished) {
            if (_debug) Serial.println("Ramp move finished successfully.");
            sendProcessComplete(CommandType::SET_POSITION_RAMP, ProcessStatus::SUCCESS);
            _currentControlMode = ControlMode::POSITION;
        }
    }
}

/**
 * @brief Resets the last received process completion state.
 * @details This is useful to prevent processing the same feedback message multiple times.
 */
void BoardControler::clearLastProcessFeedback() {
    // Set to a neutral value that won't be triggered again accidentally
    _remoteState.last_completed_process = CommandType::ENABLE_MOTOR; 
}

/**
 * @brief Sets the target torque manually.
 * @param targetTorque Desired torque in corresponding units (voltage/current).
 */
void BoardControler::setTargetTorque(float targetTorque) {
    _targetTorque = targetTorque;
}

/**
 * @brief Sets the target velocity manually.
 * @param targetVelocity Desired velocity in rad/s.
 */
void BoardControler::setTargetVelocity(float targetVelocity) {
    _targetVelocity = targetVelocity;
}

/**
 * @brief Sets the target position manually.
 * @param targetPosition Desired position in radians.
 */
void BoardControler::setTargetPosition(float targetPosition) {
    _targetPosition = targetPosition;
}

/**
 * @brief Configures the motor's underlying control and modulation types.
 * @param motionType    The desired MotionControlType (e.g., angle, velocity, torque).
 * @param focModulation The desired FOCModulationType (defaults to SinePWM).
 */
void BoardControler::setMotorControlType(MotionControlType motionType, FOCModulationType focModulation) {
    _stepper->setControlType(motionType, focModulation);
}

/**
 * @brief Configures the motor's underlying control, torque, and modulation types.
 * @param motionType    The desired MotionControlType.
 * @param torqueType    The desired TorqueControlType (e.g., foc_current, voltage).
 * @param focModulation The desired FOCModulationType (defaults to SinePWM).
 */
void BoardControler::setMotorControlType(MotionControlType motionType, TorqueControlType torqueType, FOCModulationType focModulation) {
    _stepper->setControlType(motionType, torqueType, focModulation);
}

/**
 * @brief Enables open-loop mode.
 * @details This is useful if open-loop is enough so controller gains do not have to be set.
 */
void BoardControler::enableOpenLoop() {
    if (!_stepper || !_stepper->getMotor()) return;

    StepperMotor* motor = _stepper->getMotor();

    // Set lower voltage limit in open-loop mode
    motor->voltage_limit = _openLoopMotorVoltageLimit;

    if (_debug) {
        Serial.print("Voltage limit reduced to: ");
        Serial.println(motor->voltage_limit);
    }

    // Bumpless transfer between foc and open-loop
    float currentSensorPosition = motor->shaftAngle();

    // set target position 
    _targetPosition = currentSensorPosition;
    _stepper->setTarget(currentSensorPosition);
    
    // enable open-loop
    _openLoopEnabled = true;
    if(_debug) Serial.println("Open Loop Mode ENABLED");

    // switch to position control mode
    _currentControlMode == ControlMode::POSITION;
    _stepper->setControlType(MotionControlType::angle_openloop);

}

/**
 * @brief Disables open-loop mode.
 * @details Useful if switching between open and closed loop mode during operation is necessary.
 */
void BoardControler::disableOpenLoop() {
    _openLoopEnabled = false;
    if (_debug) Serial.println("Open Loop Mode DISABLED");

    if (!_stepper || !_stepper->getMotor()) return;

    float originalDriverLimit = _stepper->getDriverConfig().voltageLimit;
    float originalMotorLimit = originalDriverLimit / 2.0f;

    _stepper->getMotor()->voltage_limit = originalMotorLimit;

    if (_debug) {
        Serial.print("Voltage Limit resotred to: ");
        Serial.println(originalMotorLimit);
    }
}

/**
 * @brief Returns the status of the open-loop mode flag.
 * @return true  If open-loop mode is currently enabled.
 * @return false If open-loop mode is disabled (closed-loop active).
 */
bool BoardControler::isOpenLoopEnabled() const {
    return _openLoopEnabled;
}