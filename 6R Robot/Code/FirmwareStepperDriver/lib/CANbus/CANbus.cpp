/*
================================================================================
File: CANbus.cpp
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Implementation file for the CANbus class. Manages the initialization of the 
STM32 HAL FDCAN peripheral, configuration of filters, and the sending/receiving 
of CAN messages (both polling and interrupt-based).

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#include "CANbus.h"

// A static pointer to the CANbus instance, used by the C-style interrupt handlers.
static CANbus* CANbusInstance = nullptr;

/**
 * @brief Constructor implementation.
 * @param hfdcan A reference to an existing FDCAN_HandleTypeDef struct.
 */
CANbus::CANbus(FDCAN_HandleTypeDef& hfdcan) {
    _hfdcan = hfdcan;
}

/**
 * @brief Activates debug prints.
 */
void CANbus::activateDebug() {
    _debug = true;
}

/**
 * @brief Enables interrupt mode. Must be called before initCANbus().
 */
void CANbus::enableInterrupt() {
    _enableInterrupt = true;
}

/**
 * @brief Sets the receive callback function for interrupt-driven reception.
 * @param callback The std::function to be executed when a message arrives.
 */
void CANbus::setReceiveCallback(const std::function<void(FDCAN_RxHeaderTypeDef, uint8_t*)>& callback) {
    _receiveCallback = callback;
}

/**
 * @brief Gets the receive callback function.
 * @return std::function<void(FDCAN_RxHeaderTypeDef, uint8_t*)> The registered callback function.
 */
std::function<void(FDCAN_RxHeaderTypeDef, uint8_t*)> CANbus::getReceiveCallback() {
    return _receiveCallback;
}

/**
 * @brief Initializes and starts the FDCAN peripheral.
 * @param baudrate The desired baud rate in bits/s. If 0, the default prescaler is used.
 * @param instance A pointer to the FDCAN peripheral instance (e.g., FDCAN1).
 */
void CANbus::initCANbus(uint32_t baudrate, FDCAN_GlobalTypeDef* instance) {
    // Set the static instance pointer so the C interrupt handlers can find this object.
    CANbusInstance = this;

    // Enable the clock for the FDCAN peripheral.
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1); // TODO: make this so that user can change the clock source

    // Copy settings from the configuration struct into the HAL handle.
    _hfdcan.Instance = instance;
    _hfdcan.Init.ClockDivider = _canConfig.ClockDivider;
    _hfdcan.Init.FrameFormat = _canConfig.FrameFormat;
    _hfdcan.Init.Mode = _canConfig.Mode;
    _hfdcan.Init.AutoRetransmission = _canConfig.AutoRetransmission;
    _hfdcan.Init.TransmitPause = _canConfig.TransmitPause;
    _hfdcan.Init.ProtocolException = _canConfig.ProtocolException;

    // Calculate the prescaler based on the desired baud rate if provided.
    if (baudrate != 0) {
        _hfdcan.Init.NominalPrescaler = calcPrescaler(baudrate);
    } else {
        _hfdcan.Init.NominalPrescaler = _canConfig.NominalPrescaler;
    }

    // Copy the rest of the timing and configuration settings.
    _hfdcan.Init.NominalSyncJumpWidth = _canConfig.NominalSyncJumpWidth;
    _hfdcan.Init.NominalTimeSeg1 = _canConfig.NominalTimeSeg1;
    _hfdcan.Init.NominalTimeSeg2 = _canConfig.NominalTimeSeg2;
    _hfdcan.Init.DataPrescaler = _canConfig.DataPrescaler;
    _hfdcan.Init.DataSyncJumpWidth = _canConfig.DataSyncJumpWidth;
    _hfdcan.Init.DataTimeSeg1 = _canConfig.DataTimeSeg1;
    _hfdcan.Init.DataTimeSeg2 = _canConfig.DataTimeSeg2;
    _hfdcan.Init.TxFifoQueueMode = _canConfig.TxFifoQueueMode;
    _hfdcan.Init.StdFiltersNbr = _canConfig.StdFiltersNbr;
    _hfdcan.Init.ExtFiltersNbr = _canConfig.ExtFiltersNbr;

    // Initialize the FDCAN peripheral with the configured settings.
    if (HAL_FDCAN_Init(&_hfdcan) != HAL_OK) {
        if (_debug) Serial.println("FDCAN Init failed!");
        while(true); // Halt on failure
    }

    // If interrupts are enabled, configure and enable the RX FIFO interrupt.
    if (_enableInterrupt) {
        if (HAL_FDCAN_ActivateNotification(&_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            if (_debug) Serial.println("Error: Failed to activate CAN RX interrupt.");
            while(true); // Halt on failure
        }
        // Enable the corresponding interrupt in the NVIC.
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }

    // Configure the default transmit header.
    _txHeader.Identifier = _txHeaderConfig.Identifier;
    _txHeader.IdType = _txHeaderConfig.IdType;
    _txHeader.TxFrameType = _txHeaderConfig.TxFrameType;
    _txHeader.BitRateSwitch = _txHeaderConfig.BitRateSwitch;
    _txHeader.ErrorStateIndicator = _txHeaderConfig.ErrorStateIndicator;
    _txHeader.MessageMarker = _txHeaderConfig.MessageMarker;

    // Configure all filters that were added via the addFilter() method.
    for (const auto& filter : _filters) {
        _sFilterConfig.IdType = filter.IdType;
        _sFilterConfig.FilterIndex = filter.FilterIndex;
        _sFilterConfig.FilterType = filter.FilterType;
        _sFilterConfig.FilterConfig = filter.FilterConfig;
        _sFilterConfig.FilterID1 = filter.FilterID1;
        _sFilterConfig.FilterID2 = filter.FilterID2;
        if (HAL_FDCAN_ConfigFilter(&_hfdcan, &_sFilterConfig) != HAL_OK) {
            if (_debug) Serial.println("Filter Config failed.");
            while(true); // Halt on failure
        }
    }

    // Configure global filter, to refuse every message with an other ID
    if (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        if (_debug) Serial.println("Global Filter Config failed.");
        while(true);
    }

    // Start the FDCAN peripheral.
    if (HAL_FDCAN_Start(&_hfdcan) != HAL_OK) {
        if (_debug) Serial.println("FDCAN Start failed.");
        while(true); // Halt on failure
    } else if (_debug) Serial.println("FDCAN Started successfully.");
}

/**
 * @brief Adds a filter configuration to a vector to be processed during initialization.
 * @param id1 The primary ID to filter for (or match ID).
 * @param id2 The secondary ID (or mask, depending on filter type).
 * @param extendedId Set to true if the filter targets an extended 29-bit CAN ID.
 */
void CANbus::addFilter(uint32_t id1, uint32_t id2, bool extendedId) {
    FilterConfig filter;
    
    // Increment the appropriate filter counter based on ID type.
    if (!extendedId) {
        _canConfig.StdFiltersNbr += 1;
        filter.IdType = FDCAN_STANDARD_ID;
    } else {
        _canConfig.ExtFiltersNbr += 1;
        filter.IdType = FDCAN_EXTENDED_ID;
    }

    // Set filter parameters from arguments and default config.
    filter.FilterIndex = _canConfig.StdFiltersNbr + _canConfig.ExtFiltersNbr - 1;
    filter.FilterType = _filterConfig.FilterType;
    filter.FilterConfig = _filterConfig.FilterConfig;
    filter.FilterID1 = id1;
    filter.FilterID2 = id2;

    // Add the configured filter to the list.
    _filters.push_back(filter);
}

/**
 * @brief Adds a message to the transmit FIFO queue.
 * @param id The CAN ID of the message.
 * @param data A pointer to the data payload array.
 * @param length The length of the data payload in bytes.
 * @return true If the message was successfully added to the TX FIFO.
 * @return false If the TX FIFO was full or an error occurred.
 */
bool CANbus::sendMessage(uint32_t id, const uint8_t* data, uint8_t length) {
    // Set the ID and data length for this specific message.
    _txHeader.Identifier = id;
    _txHeader.DataLength = getDLCFromLength(length);

    // Check if there is space in the transmit FIFO.
    if (HAL_FDCAN_GetTxFifoFreeLevel(&_hfdcan) > 0) {
        // Add the message to the queue.
        if (HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan, &_txHeader, (uint8_t*)data) != HAL_OK) {
            if (_debug) Serial.println("Error: Failed to send CAN message.");
            return false;
        }
    } else {
        if (_debug) Serial.println("Message not sent: TX FIFO Full.");
        return false;
    }

    return true;
}

/**
 * @brief Polls the RX FIFO for a new message.
 * @return true If a message was successfully received and stored.
 * @return false If no new message was available in the RX FIFO.
 */
bool CANbus::receiveMessage() {
    uint8_t rxData[64]; // Buffer for the largest possible CAN-FD message.

    // Try to get a message from RX FIFO 0.
    if (HAL_FDCAN_GetRxMessage(&_hfdcan, FDCAN_RX_FIFO0, &_rxHeader, rxData) == HAL_OK) {
        uint8_t dataLength = getLengthFromDLC(_rxHeader.DataLength);
        if (dataLength == 0) return true; // Message with no data is still a valid reception.

        // Store the received message in the map.
        std::vector<uint8_t> data(rxData, rxData + dataLength);
        _messages[_rxHeader.Identifier] = {_rxHeader, data};
    } else {
        return false; // No message was received.
    }
    return true; // A message was successfully received.
}

// --- Implementation of Getters ---

/**
 * @brief Retrieves the internal filter configuration struct.
 * @return FilterConfig& Reference to the FilterConfig struct.
 */
FilterConfig& CANbus::getFilterConfig() { return _filterConfig; }

/**
 * @brief Retrieves the general CAN configuration struct.
 * @return CANConfig& Reference to the CANConfig struct.
 */
CANConfig& CANbus::getCANConfig() { return _canConfig; }

/**
 * @brief Retrieves the transmit header configuration struct.
 * @return TXHeaderConfig& Reference to the TXHeaderConfig struct.
 */
TXHeaderConfig& CANbus::getTXHeaderConfig() { return _txHeaderConfig; }

/**
 * @brief Determines the FDCAN peripheral's clock frequency based on system configuration.
 * @return uint32_t The clock frequency of the CAN peripheral in Hz.
 */
uint32_t CANbus::getCANClockFrequency() {
    RCC_ClkInitTypeDef clkInit;
    uint32_t flashLatency;

    HAL_RCC_GetClockConfig(&clkInit, &flashLatency);
    uint32_t canClockSource = __HAL_RCC_GET_FDCAN_SOURCE();

    // Return the frequency based on the configured clock source.
    if (canClockSource == RCC_FDCANCLKSOURCE_HSE) {
        return HSE_VALUE;
    } else if (canClockSource == RCC_FDCANCLKSOURCE_PLL) {
        return HAL_RCC_GetPCLK1Freq(); // Assuming PLL is derived from PCLK1
    } else if (canClockSource == RCC_FDCANCLKSOURCE_PCLK1) {
        return HAL_RCC_GetPCLK1Freq();
    } else {
        if (_debug) Serial.println("Error: Unable to determine CAN clock source. Halting program.");
        while (true);
    }
    return 0;
}

/**
 * @brief Calculates the NominalPrescaler value for a desired baud rate.
 * @param baudrate The targeted baud rate in bits/s.
 * @return uint32_t The calculated prescaler value needed to achieve the baud rate.
 */
uint32_t CANbus::calcPrescaler(uint32_t baudrate) {
    // Check if the requested baud rate is in the list of commonly supported rates.
    const uint32_t supportedBaudRates[] = {125000, 250000, 500000, 1000000};
    bool isSupported = false;
    for (uint32_t baud : supportedBaudRates) {
        if (baudrate == baud) {
            isSupported = true;
            break;
        }
    }
    if (!isSupported) {
        if (_debug) {
            Serial.print("Error: Baud rate ");
            Serial.print(baudrate);
            Serial.println(" bit/s is not supported. Halting program.");
        }
        while (true);
    }

    // Formula: Prescaler = F_can / (Baudrate * TotalTimeQuanta)
    uint32_t NominalTimeSeg1 = _canConfig.NominalTimeSeg1;
    uint32_t NominalTimeSeg2 = _canConfig.NominalTimeSeg2;
    uint32_t CAN_CLK = getCANClockFrequency();
    uint32_t totalTimeQuanta = NominalTimeSeg1 + NominalTimeSeg2 + 1;
    uint32_t prescaler = CAN_CLK / (baudrate * totalTimeQuanta);

    // Validate the calculated prescaler value.
    if (prescaler < 1 || prescaler > 512) {
        if (_debug) {
            Serial.println("Error: Calculated prescaler is invalid. Halting program.");
            Serial.print("Calculated prescaler: ");
            Serial.println(prescaler);
        }
        while (true);
    }

    if (_debug) {
        Serial.print("Prescaler calculated successfully: ");
        Serial.println(prescaler);
    }
    return prescaler;
}

/**
 * @brief Converts a data length in bytes to the HAL FDCAN DLC enum value.
 * @param length The data payload length in bytes.
 * @return uint32_t The corresponding FDCAN_DLC_BYTES_x constant.
 */
uint32_t CANbus::getDLCFromLength(uint8_t length) {
    switch (length) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default:
            if (_debug) {
                Serial.print("Error: Invalid Data Length provided: ");
                Serial.println(length);
            }
            while(true); // Halt on invalid length
    }
}

/**
 * @brief Converts a HAL FDCAN DLC enum value back to a length in bytes.
 * @param dlc The FDCAN_DLC_BYTES_x constant.
 * @return uint8_t The data length in bytes.
 */
uint8_t CANbus::getLengthFromDLC(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 0;
    }
}

/**
 * @brief Retrieves the last message stored for a specific ID.
 * @param id The CAN identifier of the requested message.
 * @return std::pair<FDCAN_RxHeaderTypeDef, std::vector<uint8_t>> A pair mapping the RX header to its payload.
 */
std::pair<FDCAN_RxHeaderTypeDef, std::vector<uint8_t>> CANbus::getLastMessage(uint32_t id) {
    if (_messages.find(id) != _messages.end()) {
        return _messages[id];
    } else {
        // Return an empty object if no message with the given ID was found.
        return {FDCAN_RxHeaderTypeDef(), std::vector<uint8_t>()};
    }
}

/**
 * @brief Returns a pointer to the internal FDCAN handle.
 * @return FDCAN_HandleTypeDef* Pointer to the HAL handle.
 */
FDCAN_HandleTypeDef* CANbus::getHandle() {
    return &_hfdcan;
}

// --- C-style Interrupt Handlers ---

/**
 * @brief C-style callback function executed by the HAL when a new message arrives in RX FIFO 0.
 * @details This function acts as a bridge from the hardware interrupt to the C++ class method.
 * @param hfdcan Pointer to the FDCAN handle.
 * @param RxFifo0ITs The RX FIFO 0 interrupt flags.
 */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        uint8_t rxData[64];
        FDCAN_RxHeaderTypeDef rxHeader;

        // Retrieve the message from the hardware FIFO.
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
            // If a C++ class instance exists, call its callback function.
            if (CANbusInstance) {
                auto callback = CANbusInstance->getReceiveCallback();
                if (callback) {
                    callback(rxHeader, rxData);
                }
            }
        }
    }
}

/**
 * @brief C-style interrupt handler for FDCAN1 IT0 line.
 * @details This function passes the interrupt up to the main HAL IRQ handler.
 */
extern "C" void FDCAN1_IT0_IRQHandler(void) {
    if (CANbusInstance) {
        HAL_FDCAN_IRQHandler((CANbusInstance->getHandle()));
    }
}