#define QMA6100P_I2C_ADDRESS 0x12

#define QMA6100P_DEVICE_ID 0x90
#define QMA6100P_REG_XOUTL 0x01
#define QMA6100P_REG_XOUTH 0x02
#define QMA6100P_REG_YOUTL 0x03
#define QMA6100P_REG_YOUTH 0x04
#define QMA6100P_REG_ZOUTL 0x05
#define QMA6100P_REG_ZOUTH 0x06
#define QMA6100P_REG_RESET 0x36
#define QMA6100P_REG_POWER_MANAGE 0x11
#define QMA6100P_REG_RANGE 0x0f
#define QMA6100P_REG_BW_ODR 0x10
#define QMA6100P_REG_CHIP_ID 0x00
#define QMA6100P_REG_NO_MOTION_TH 0x2d
#define QMA6100P_REG_ANY_MOTION_TH 0x2e
#define QMA6100P_REG_NO_ANY_MOTION_DUR 0x2c
#define QMA6100P_REG_INT_EN_0 0x16
#define QMA6100P_REG_INT_EN_2 0x18
#define QMA6100P_REG_INT1_MAP_0 0x19
#define QMA6100P_REG_INT1_MAP_1 0x1a
#define QMA6100P_REG_TAP_QUIET_TH 0x1e
#define QMA6100P_REG_TAP_1 0x2a
#define QMA6100P_REG_TAP_2 0x2b
#define QMA6100P_REG_INT_STATUS_0 0x09
#define QMA6100P_REG_INT_STATUS_1 0x0a

typedef enum
{
    QMA6100P_RANGE_2G = 0x01,
    QMA6100P_RANGE_4G = 0x02,
    QMA6100P_RANGE_8G = 0x04,
    QMA6100P_RANGE_16G = 0x08,
    QMA6100P_RANGE_32G = 0x0f
} qma6100p_range;

typedef enum
{
    QMA6100P_BW_100 = 0,
    QMA6100P_BW_200 = 1,
    QMA6100P_BW_400 = 2,
    QMA6100P_BW_800 = 3,
    QMA6100P_BW_1600 = 4,
    QMA6100P_BW_50 = 5,
    QMA6100P_BW_25 = 6,
    QMA6100P_BW_12_5 = 7,
    QMA6100P_BW_OTHER = 8
} qma6100p_bw;

typedef enum
{
    QMA6100P_SENSITITY_2G = 244,
    QMA6100P_SENSITITY_4G = 488,
    QMA6100P_SENSITITY_8G = 977,
    QMA6100P_SENSITITY_16G = 1950,
    QMA6100P_SENSITITY_32G = 3910
} qma6100p_sensitity;
