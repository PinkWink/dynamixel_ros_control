#ifndef DYNAMIXEL_CONTROL_TABLE_H_
#define DYNAMIXEL_CONTROL_TABLE_H_

#include <map>
#include <math.h>
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
    VELOCITY_LIMIT,
    TORQUE_ENABLE,
    GOAL_CURRENT,
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
    {DynamixelSeries::SERIES_ROBOTIS_HAND, 610},
    {DynamixelSeries::SERIES_DYNAMIXEL_X, 122},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO, 610},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS, 570},
};

static std::map<DynamixelSeries, int> DynamixelReadLength =
{
    {DynamixelSeries::SERIES_ROBOTIS_HAND, 11},
    {DynamixelSeries::SERIES_DYNAMIXEL_X, 14},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO, 11},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS, 14},
};

static std::map<DynamixelSeries, std::map<DynamixelControlTableItem, int>> DynamixelControlTable =
{
    {DynamixelSeries::SERIES_ROBOTIS_HAND, {
        {DynamixelControlTableItem::OPERATING_MODE,        11},
        {DynamixelControlTableItem::VELOCITY_LIMIT,        32},
        {DynamixelControlTableItem::TORQUE_ENABLE,         562},
        {DynamixelControlTableItem::GOAL_CURRENT,          604},
        {DynamixelControlTableItem::GOAL_VELOCITY,         600},
        {DynamixelControlTableItem::PROFILE_ACCELERATION,  606},
        {DynamixelControlTableItem::PROFILE_VELOCITY,      600},
        {DynamixelControlTableItem::GOAL_POSITION,         596},
        {DynamixelControlTableItem::MOVING,                610},
        {DynamixelControlTableItem::PRESENT_CURRENT,       621},
        {DynamixelControlTableItem::PRESENT_VELOCITY,      615},
        {DynamixelControlTableItem::PRESENT_POSITION,      611}
    }},

    {DynamixelSeries::SERIES_DYNAMIXEL_X, {
        {DynamixelControlTableItem::OPERATING_MODE,        11},
        {DynamixelControlTableItem::VELOCITY_LIMIT,        44},
        {DynamixelControlTableItem::TORQUE_ENABLE,         64},
        {DynamixelControlTableItem::GOAL_CURRENT,          102},
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
        {DynamixelControlTableItem::VELOCITY_LIMIT,        32},
        {DynamixelControlTableItem::TORQUE_ENABLE,         562},
        {DynamixelControlTableItem::GOAL_CURRENT,          550},
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
        {DynamixelControlTableItem::VELOCITY_LIMIT,        44},
        {DynamixelControlTableItem::TORQUE_ENABLE,         512},
        {DynamixelControlTableItem::GOAL_CURRENT,          550},
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

static std::map<DynamixelSeries, double> DynamixelCurrentConvert =
{
    {DynamixelSeries::SERIES_ROBOTIS_HAND, (4.024 / 1000.0)},
    {DynamixelSeries::SERIES_DYNAMIXEL_X, (2.69 / 1000.0)},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO, (16.11328 / 1000.0)},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS, (1.0 / 1000.0)},
};

static std::map<DynamixelSeries, double> DynamixelVeolcityConvert =
{
    {DynamixelSeries::SERIES_ROBOTIS_HAND, (1.0 * M_PI / 60.0)},
    {DynamixelSeries::SERIES_DYNAMIXEL_X, (0.229 * M_PI / 60.0)},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO, (0.00199234 * M_PI / 60.0)},
    {DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS, (0.01 * M_PI / 60.0)},
};

static std::map<int, double> DynamixelPositionConvert =
{
    {XL430_W250, (M_PI / 2048.0)},
    {XM430_W210, (M_PI / 2048.0)},
    {XM430_W350, (M_PI / 2048.0)},
    {XM540_W150, (M_PI / 2048.0)},
    {XM540_W270, (M_PI / 2048.0)},
    {XH430_V210, (M_PI / 2048.0)},
    {XH430_V350, (M_PI / 2048.0)},
    {XH430_W210, (M_PI / 2048.0)},
    {XH430_W350, (M_PI / 2048.0)},
    {PRO_H42_20_S300_R, (M_PI / 151875.0)},
    {PRO_H54_100_S500_R, (M_PI / 250961.5)},
    {PRO_H54_200_S500_R, (M_PI / 250961.5)},
    {PRO_PLUS_H42P_020_S300_R, (M_PI / 303750.0)},
    {PRO_PLUS_H54P_100_S500_R, (M_PI / 501923.0)},
    {PRO_PLUS_H54P_200_S500_R, (M_PI / 501923.0)},
    {ROBOTIS_HAND_RH_P12_RN, 1.0},
};

#endif //DYNAMIXEL_CONTROL_TABLE_H_