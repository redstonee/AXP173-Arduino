/**
 * @file AXP173.h
 * @author By mondraker (691806052@qq.com) (qq:735791683)
 * @brief The base library comes from m5stack,They open-sourced
 * the AXP192 library,Thanks in advance!
 *
 * https://docs.m5stack.com/en/products
 *
 * @version 0.2
 * @date 2022-12-05
 * @copyright Copyright (c) 2022
 */

#ifndef _AXP173_H_
#define _AXP173_H_

#include <Wire.h>
#include <functional>
#include <bitset>

enum OutputChannel
{                 // 可调电压输出通道（ldo1为RTC电源，电压不可调）
    OP_DCDC1 = 0, // 0
    OP_LDO4,      // 1
    OP_LDO2,      // 2
    OP_LDO3,      // 3
    OP_DCDC2,     // 4
    OP_EXTEN = 6,
};

enum AdcChannel
{               // ADC采集电压与电流参数 （ADC使能地址：0x82;默认值：0x83）
    ADC_TS = 0, // 温敏电阻管脚ADC
    ADC_APS_V,  // APS电压
    ADC_VBUS_C, // 直流输入电流
    ADC_VBUS_V, // 直流输入电压
    ADC_ACIN_C, // 交流输入电流
    ADC_ACIN_V, // 交流输入电压
    ADC_BAT_C,  // 电池输入电流
    ADC_BAT_V,  // 电池输入电压
};

enum ChargeCurrent
{                  // 电池充电电流设置 （地址：0x33;初值：0xC8）
    CHG_100mA = 0, // 0000
    CHG_190mA,     // 0001
    CHG_280mA,     // 0010
    CHG_360mA,     // 0011
    CHG_450mA,     // 0100
    CHG_550mA,     // 0101
    CHG_630mA,     // 0110
    CHG_700mA,     // 0111
    CHG_780mA,     // 1000
    CHG_880mA,     // 1001
    CHG_960mA,     // 1010
    CHG_1000mA,    // 1011
    CHG_1080mA,    // 1100
    CHG_1160mA,    // 1101
    CHG_1240mA,    // 1110
    CHG_1320mA,    // 1111
};

enum CoulometerStatus
{ // 库仑计控制 （地址：0xB8;默认值：0x00）
    COULOMETER_RESET = 5,
    COULOMETER_PAUSE,
    COULOMETER_ENABLE,
};

enum PowerOffPressTime
{                    // 关机时长设置 （地址：0x36;【只操作01两位】）
    POWEROFF_4S = 0, // 00
    POWEROFF_6S,     // 01
    POWEROFF_8S,     // 10
    POWEROFF_10S,    // 11
};

enum PowerOnPressTime
{                      // 开机时长设置（地址：0x36;【只操作67两位】）
    POWERON_128mS = 0, // 0000 0000
    POWERON_512mS,     // 0100 0000
    POWERON_1S,        // 1000 0000
    POWERON_2S,        // 1100 0000
};

enum LongPressTime
{                  // 长按键PEK触发时间（地址：0x36）
    LPRESS_1S = 0, // 00
    LPRESS_1_5S,   // 01
    LPRESS_2S,     // 10
    LPRESS_2_5S,   // 11
};

class AXP173
{
public:
    enum AXP_IRQn : uint8_t
    {
        /* IRQ register 1*/
        RESERVED1_IRQn,      // Reserved
        VBUS_UNDERVOLT_IRQn, // VBUS voltage under VHOLD
        VBUS_REMOVE_IRQn,    // VBUS remove
        VBUS_INSERT_IRQn,    // VBUS insert
        VBUS_OVERVOLT_IRQn,  // VBUS over voltage
        ACIN_REMOVE_IRQn,    // ACIN remove
        ACIN_INSERT_IRQn,    // ACIN insert
        ACIN_OVERVOLT_IRQn,  // ACIN over voltage
        /* IRQ register 2*/
        BAT_UNDER_TEMP_IRQn, // Battery temperature under limit
        BAT_OVER_TEMP_IRQn,  // Battery temperature over limit
        BAT_CHG_FIN_IRQn,    // Battery charging finish
        BAT_CHG_START_IRQn,  // Battery charging start
        BAT_ACT_STOP_IRQn,   // Battery activation stop
        BAT_ACT_START_IRQn,  // Battery activation start
        BAT_REMOVE_IRQn,     // Battery remove
        BAT_INSERT_IRQn,     // Battery insert
        /* IRQ register 3*/
        PEK_LONG_PRESS_IRQn,    // PEK long press
        PEK_SHORT_PRESS_IRQn,   // PEK short press
        RESERVED2_IRQn,         // Reserved
        LDO4_UNDERVOLT_IRQn,    // LDO4 output voltage too low
        DCDC2_UNDERVOLT_IRQn,   // DCDC2 output voltage too low
        DCDC1_UNDERVOLT_IRQn,   // DCDC1 output voltage too low
        BAT_CHG_UNDERCURR_IRQn, // Battery charging current too low
        AXP_OVER_TEMP_IRQn,     // Chip internal temperature over limit
        /* IRQ register 4*/
        APS_UNDERVOLT_IRQn,   // APS voltage under limit
        RESERVED3_IRQn,       // Reserved
        VBUS_SESS_END_IRQn,   // VBUS session end
        VBUS_SESS_START_IRQn, // VBUS session start
        VBUS_INVALID_IRQn,    // VBUS invalid
        VBUS_VALID_IRQn,      // VBUS valid
        RESERVED4_IRQn,       // Reserved
        RESERVED5_IRQn,       // Reserved

        NUM_IRQn, // All interrupts
    };

private:
    TwoWire &_wire;
    uint8_t _dev_addr;

    bool writeRegs(uint8_t addr, const uint8_t *buf, uint8_t len);
    uint8_t readRegs(uint8_t addr, uint8_t *buf, uint8_t len);

    inline bool writeReg(uint8_t addr, uint8_t data)
    {
        return writeRegs(addr, &data, 1);
    }

    inline uint8_t readReg(uint8_t addr)
    {
        uint8_t data;
        if (!readRegs(addr, &data, 1))
        {
            return 0; // Read failed
        }
        return data;
    }

public:
    AXP173(TwoWire &wire = Wire, uint8_t dev_addr = 0x34);
    bool begin();

    void setPmuPower();
    void setDefaultConfig();

    /* Power input state（输入电源状态检测） */
    // 地址：0x00
    bool isACINExist();      // ACIN存在指示
    bool isACINAvl();        // ACIN是否可用
    bool isVBUSExist();      // VBUS存在指示
    bool isVBUSAvl();        // VBUS是否可用
    bool getBatCurrentDir(); // 获取电池电流方向（0：在放电；1：在充电）
    // 地址：0x01
    bool isAXP173OverTemp(); // 指示AXP173是否过温
    bool isCharging();       // 充电指示（0：未充电或已充电完成；1：正在充电）
    bool isBatExist();       // 电池存在状态指示
    bool isChargeCsmaller(); // 指示充电电流是否小于期望电流（0：实际充电电流等于期望电流；1：实际充电电流小于期望电流）

    /* Power output control （电源输出控制）*/
    void setOutputEnable(OutputChannel channel, bool state);        // channel：设置电源输出通道（OUTPUT_CHANNEL）；state：设置是否输出
    void setOutputVoltage(OutputChannel channel, uint16_t voltage); // channel：设置电源输出通道（OUTPUT_CHANNEL）；voltage：设置输出电压
                                                                    // DCDC1 & LDO4: 700~3500(mV), DCDC2: 700~2275(mV), LDO2 & LDO3: 1800~3300(mV)
    /* Basic control (开关芯片控制) */
    void powerOFF(void);                                  // 调用直接关机
    bool powerState(void);                                // 若关机则返回false
    void setPowerKeyShutdownEnable(bool enabled);         // 按键时长大于关机时长自动关机使能（默认使能）
    void setLongPressTime(LongPressTime pressTime);       // 设置长按键触发时间
    void setPowerOffPressTime(PowerOffPressTime offTime); // 设置关机时间（输入参数见POWEROFF_TIME枚举体）
    void setPowerOnPressTime(PowerOnPressTime onTime);    // 设置开机时间（输入参数见POWERON_TIME枚举体）

    /* Charge control (电池充电设置) */
    void setChargeEnable(bool state);             // 充电功能使能控制位7bit（上电默认开启）
    void setChargeCurrent(ChargeCurrent current); // 充电电流设置0-3bit，写入电流见CHARGE_CURRENT枚举体

    /* ADC control (ADC设置) */
    void setADCEnable(AdcChannel channel, bool state); // ADC使能1 channel：设置ADC使能通道 参数见ADC_CHANNEL枚举体 state:设置是否输出
    void setChipTempEnable(bool state);                // ADC使能2 设置芯片温度检测ADC使能 state:设置是否输出 默认输出

    /* Coulometer control (库仑计模式设置) */
    void setCoulometer(CoulometerStatus option, bool state); // 设置库仑计状态（开关ENABLE，暂停PAUSE，清零RESET）

    /* Coulometer data (库仑计数据) */
    uint32_t getCoulometerChargeDataRaw(void);    // 电池充电库仑计数据寄存器3
    uint32_t getCoulometerDischargeDataRaw(void); // 电池放电库仑计数据寄存器3
    float getCoulometerChargeData(void);          // 库仑计输入计数
    float getCoulometerDischargeData(void);       // 库仑计输出计数
    float getCoulometerData(void);                // 计算后返回的值 get coulomb val affter calculation

    /* BAT data (电池状态数据) */
    float getBatVoltage(); // 返回高八位 + 低四位电池电压   地址：高0x78 低0x79
    float getBatCurrent(); // 返回高八位 + 低五位电池电流   地址：充电电流（高0x7A 低0x7B） & 放电电流（高0x7C 低0x7D）
    float getBatLevel();   // 返回电池电量等级（%）
    float getBatPower();   // 返回高八位 + 中八位 + 低八位电池瞬时功率  地址：高0x70 中0x71 低0x72
    // uint32_t getChargeTimeMS();

    /* VBUS data (外部输入电压状态数据) */
    float getVBUSVoltage(); // 返回高八位 + 低四位USB输入电压   地址：高0x5A 低0x5B
    float getVBUSCurrent(); // 返回高八位 + 低四位USB输入电流   地址：高0x5C 低0x5D

    /* Temperature data (温度监控数据) */
    float getChipTemp(); // 返回高八位 + 低四位芯片内置温度传感器温度 地址：高0x5E 低0x5F
    float getTSTemp();   // 返回高八位 + 低四位芯片TS脚热敏电阻检测到的电池温度  地址：高0x62 低0x63

    void enableIRQs(std::bitset<NUM_IRQn> irqs);
    void disableIRQs(std::bitset<NUM_IRQn> irqs);
    void clearIRQFlags(std::bitset<NUM_IRQn> irqs);
    std::bitset<NUM_IRQn> getIRQFlags();
};

#endif