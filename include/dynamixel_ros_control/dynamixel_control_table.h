#ifndef DYNAMIXEL_CONTROL_TABLE_H_
#define DYNAMIXEL_CONTROL_TABLE_H_

#include <map>
#include <dynamixel_ros_control/dynamixel_info.h>

enum class DynamixelSeries
{
    SERIES_DYNAMIXEL_PRO_PLUS,
    SERIES_DYNAMIXEL_PRO,
    SERIES_DYNAMIXEL_X,
    SERIES_ROBOTIS_HAND,
};

enum class DynamixelControlTableItem
{
    OPERATING_MODE,
    TORQUE_ENABLE,
    GOAL_VELOCITY,
    PROFILE_ACCELERATION,
    PROFILE_VELOCITY,
    GOAL_POSITION,
    MOVING,
    PRESENT_CURRENT,
    PRESENT_VELOCITY,
    PRESENT_POSITION
};

static std::map<int, DynamixelSeries> DynamixelModel =
{
    {XL430_W250, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XM430_W210, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XM430_W350, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XM540_W150, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XM540_W270, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XH430_V210, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XH430_V350, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XH430_W210, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {XH430_W350, DynamixelSeries::SERIES_DYNAMIXEL_X},
    {PRO_H42_20_S300_R, DynamixelSeries::SERIES_DYNAMIXEL_PRO},
    {PRO_H54_100_S500_R, DynamixelSeries::SERIES_DYNAMIXEL_PRO},
    {PRO_H54_200_S500_R, DynamixelSeries::SERIES_DYNAMIXEL_PRO},
    {PRO_PLUS_H42P_020_S300_R, DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS},
    {PRO_PLUS_H54P_100_S500_R, DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS},
    {PRO_PLUS_H54P_200_S500_R, DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS},
    {ROBOTIS_HAND_RH_P12_RN, DynamixelSeries::SERIES_ROBOTIS_HAND},
};

static std::map<DynamixelSeries, int> DynamixelReadStartAddress =
{
    {DynamixelSeries::SERIES_ROBOTIS_HAND, 122},
    {DynamixelSeries::SERIES_DYNAMIXEL_X, 122},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO, 610},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS, 570},
};

static std::map<DynamixelSeries, int> DynamixelReadLength =
{
    {DynamixelSeries::SERIES_ROBOTIS_HAND, 14},
    {DynamixelSeries::SERIES_DYNAMIXEL_X, 14},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO, 11},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS, 14},
};

static std::map<DynamixelSeries, std::map<DynamixelControlTableItem, int>> DynamixelControlTable =
{
    {DynamixelSeries::SERIES_ROBOTIS_HAND, {
        {DynamixelControlTableItem::OPERATING_MODE,        11},
        {DynamixelControlTableItem::TORQUE_ENABLE,         64},
        {DynamixelControlTableItem::GOAL_VELOCITY,         104},
        {DynamixelControlTableItem::PROFILE_ACCELERATION,  108},
        {DynamixelControlTableItem::PROFILE_VELOCITY,      112},
        {DynamixelControlTableItem::GOAL_POSITION,         116},
        {DynamixelControlTableItem::MOVING,                122},
        {DynamixelControlTableItem::PRESENT_CURRENT,       126},
        {DynamixelControlTableItem::PRESENT_VELOCITY,      128},
        {DynamixelControlTableItem::PRESENT_POSITION,      132}
    }},

    {DynamixelSeries::SERIES_DYNAMIXEL_X, {
        {DynamixelControlTableItem::OPERATING_MODE,        11},
        {DynamixelControlTableItem::TORQUE_ENABLE,         64},
        {DynamixelControlTableItem::GOAL_VELOCITY,         104},
        {DynamixelControlTableItem::PROFILE_ACCELERATION,  108},
        {DynamixelControlTableItem::PROFILE_VELOCITY,      112},
        {DynamixelControlTableItem::GOAL_POSITION,         116},
        {DynamixelControlTableItem::MOVING,                122},
        {DynamixelControlTableItem::PRESENT_CURRENT,       126},
        {DynamixelControlTableItem::PRESENT_VELOCITY,      128},
        {DynamixelControlTableItem::PRESENT_POSITION,      132}
    }},

    {DynamixelSeries::SERIES_DYNAMIXEL_PRO, {
        {DynamixelControlTableItem::OPERATING_MODE,        11},
        {DynamixelControlTableItem::TORQUE_ENABLE,         562},
        {DynamixelControlTableItem::GOAL_VELOCITY,         600},
        {DynamixelControlTableItem::PROFILE_ACCELERATION,  606},
        {DynamixelControlTableItem::PROFILE_VELOCITY,      600},
        {DynamixelControlTableItem::GOAL_POSITION,         596},
        {DynamixelControlTableItem::MOVING,                610},
        {DynamixelControlTableItem::PRESENT_CURRENT,       621},
        {DynamixelControlTableItem::PRESENT_VELOCITY,      615},
        {DynamixelControlTableItem::PRESENT_POSITION,      611}
    }},

    {DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS, {
        {DynamixelControlTableItem::OPERATING_MODE,        11},
        {DynamixelControlTableItem::TORQUE_ENABLE,         512},
        {DynamixelControlTableItem::GOAL_VELOCITY,         552},
        {DynamixelControlTableItem::PROFILE_ACCELERATION,  556},
        {DynamixelControlTableItem::PROFILE_VELOCITY,      560},
        {DynamixelControlTableItem::GOAL_POSITION,         564},
        {DynamixelControlTableItem::MOVING,                570},
        {DynamixelControlTableItem::PRESENT_CURRENT,       574},
        {DynamixelControlTableItem::PRESENT_VELOCITY,      576},
        {DynamixelControlTableItem::PRESENT_POSITION,      580}
    }},
};

// different lenth?
// different member?


#endif //DYNAMIXEL_CONTROL_TABLE_H_