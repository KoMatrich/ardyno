#pragma once

#include <Dynamixel.h>
#include <DynamixelMotor.h>

/*
///until c++20 is thing in embedded world
class AX_12A_Address
{
    using enum DynCommonAddress;
    using enum DynMotorAdditionalAddress;
    using enum DynMotorAddress;
};
*/

enum class AX_12A_Address
{
    MODEL = DynCommonAddress::MODEL,
    FIRMWARE = DynCommonAddress::FIRMWARE,
    ID = DynCommonAddress::ID,
    BAUDRATE = DynCommonAddress::BAUDRATE,
    RDT = DynCommonAddress::RDT,
    SRL = DynCommonAddress::SRL,

    TEMP_LIMIT = DynMotorAdditionalAddress::TEMP_LIMIT,
    MIN_VOLTAGE_LIMIT = DynMotorAdditionalAddress::MIN_VOLTAGE_LIMIT,
    MAX_VOLTAGE_LIMIT = DynMotorAdditionalAddress::MAX_VOLTAGE_LIMIT,
    ALARM_LED = DynMotorAdditionalAddress::ALARM_LED,
    SHUTDOWN = DynMotorAdditionalAddress::SHUTDOWN,
    CURRENT_SPEED = DynMotorAdditionalAddress::CURRENT_SPEED,
    CURRENT_TORQUE = DynMotorAdditionalAddress::CURRENT_TORQUE,
    CURRENT_VOLTAGE = DynMotorAdditionalAddress::CURRENT_VOLTAGE,
    CURRENT_TEMP = DynMotorAdditionalAddress::CURRENT_TEMP,
    EEPROM_LOCK = DynMotorAdditionalAddress::EEPROM_LOCK,
    MIN_CURRENT = DynMotorAdditionalAddress::MIN_CURRENT,

    CW_LIMIT = DynMotorAddress::CW_LIMIT,
    CCW_LIMIT = DynMotorAddress::CCW_LIMIT,
    MAX_TORQUE = DynMotorAddress::MAX_TORQUE,
    ENABLE_TORQUE = DynMotorAddress::ENABLE_TORQUE,
    LED = DynMotorAddress::LED,
    CW_COMP_MARGIN = DynMotorAddress::CW_COMP_MARGIN,
    CCW_COMP_MARGIN = DynMotorAddress::CCW_COMP_MARGIN,
    CW_COMP_SLOPE = DynMotorAddress::CW_COMP_SLOPE,
    CCW_COMP_SLOPE = DynMotorAddress::CCW_COMP_SLOPE,
    GOAL_POSITION = DynMotorAddress::GOAL_POSITION,
    GOAL_SPEED = DynMotorAddress::GOAL_SPEED,
    TORQUE_LIMIT = DynMotorAddress::TORQUE_LIMIT,
    CURRENT_POSITION = DynMotorAddress::CURRENT_POSITION,
    MOVING = DynMotorAddress::MOVING
};
