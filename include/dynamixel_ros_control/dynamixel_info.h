#ifndef DYNAMIXEL_INFO_H_
#define DYNAMIXEL_INFO_H_

#include <map>

#define AX_12A     12
#define AX_12W     300
#define AX_18A     18

#define RX_10      10
#define RX_24F     24
#define RX_28      28
#define RX_64      64

#define EX_106     107

#define MX_12W     360
#define MX_28      29
#define MX_28_2    30
#define MX_64      310
#define MX_64_2    311
#define MX_106     320
#define MX_106_2   321

#define XL_320     350
#define XL430_W250 1060

#define XM430_W210 1030
#define XM430_W350 1020
#define XM540_W150 1130
#define XM540_W270 1120

#define XH430_V210 1050
#define XH430_V350 1040
#define XH430_W210 1010
#define XH430_W350 1000

#define PRO_L42_10_S300_R  35072
#define PRO_L54_30_S400_R  37928
#define PRO_L54_30_S500_R  37896
#define PRO_L54_50_S290_R  38176
#define PRO_L54_50_S500_R  38152

#define PRO_M42_10_S260_R  43288
#define PRO_M54_40_S250_R  46096
#define PRO_M54_60_S250_R  46352

#define PRO_H42_20_S300_R  51200
#define PRO_H54_100_S500_R 53768
#define PRO_H54_200_S500_R 54024

#define PRO_PLUS_H42P_020_S300_R  2000
#define PRO_PLUS_H54P_100_S500_R  2010
#define PRO_PLUS_H54P_200_S500_R  2020

#define ROBOTIS_HAND_RH_P12_RN 35073

static std::map<unsigned int, const std::string> dynamixel_model_name = {
    {AX_12A, "AX-12A"},
    {AX_12W, "AX-12W"},
    {AX_18A, "AX-18A"},
    {RX_10, "RX-10"},
    {RX_24F, "RX-24F"},
    {RX_28, "RX-28"},
    {RX_64, "RX-64"},
    {EX_106, "EX-106"},
    {MX_12W, "MX-12W"},
    {MX_28, "MX-28"},
    {MX_28_2, "MX-28-2"},
    {MX_64, "MX-64"},
    {MX_64_2, "MX-64-2"},
    {MX_106, "MX-106"},
    {MX_106_2, "MX-106-2"},
    {XL_320, "XL-320"},
    {XL430_W250, "XL430-W250"},
    {XM430_W210, "XM430-W210"},
    {XM430_W350, "XM430-W350"},
    {XM540_W150, "XM540-W150"},
    {XM540_W270, "XM540-W270"},
    {XH430_V210, "XH430-V210"},
    {XH430_V350, "XH430-V350"},
    {XH430_W210, "XH430-W210"},
    {XH430_W350, "XH430-W350"},
    {PRO_L42_10_S300_R, "PRO-L42-10-S300-R"},
    {PRO_L54_30_S400_R, "PRO-L54-30-S400-R"},
    {PRO_L54_30_S500_R, "PRO-L54-30-S500-R"},
    {PRO_L54_50_S290_R, "PRO-L54-50-S290-R"},
    {PRO_L54_50_S500_R, "PRO-L54-50-S500-R"},
    {PRO_M42_10_S260_R, "PRO-M42-10-S260-R"},
    {PRO_M54_40_S250_R, "PRO-M54-40-S250-R"},
    {PRO_M54_60_S250_R, "PRO-M54-60-S250-R"},
    {PRO_H42_20_S300_R, "PRO-H42-20-S300-R"},
    {PRO_H54_100_S500_R, "PRO-H54-100-S500-R"},
    {PRO_H54_200_S500_R, "PRO-H54-200-S500-R"},
    {PRO_PLUS_H42P_020_S300_R, "PRO-PLUS-H42P-020-S300-R"},
    {PRO_PLUS_H54P_100_S500_R, "PRO-PLUS-H54P-100-S500-R"},
    {PRO_PLUS_H54P_200_S500_R, "PRO-PLUS-H54P-200-S500-R"},
    {ROBOTIS_HAND_RH_P12_RN, "ROBOTIS-HAND-RH-P12-RN"},
};

#endif //DYNAMIXEL_INFO_H_