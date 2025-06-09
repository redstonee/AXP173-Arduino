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
#include <bitset>

class AXP173
{
public:
    enum OutputChannel : uint8_t
    {              // 可调电压输出通道
        DCDC1 = 0, // 0
        LDO4,      // 1
        LDO2,      // 2
        LDO3,      // 3
        DCDC2,     // 4
        EXTEN = 6,
    };

    enum AdcChannel : uint8_t
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

    enum ChargeCurrent : uint8_t
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

    enum PowerOffPressTime : uint8_t
    {                    // 关机时长设置 （地址：0x36;【只操作01两位】）
        POWEROFF_4S = 0, // 00
        POWEROFF_6S,     // 01
        POWEROFF_8S,     // 10
        POWEROFF_10S,    // 11
    };

    enum PowerOnPressTime : uint8_t
    {                      // 开机时长设置（地址：0x36;【只操作67两位】）
        POWERON_128mS = 0, // 0000 0000
        POWERON_512mS,     // 0100 0000
        POWERON_1S,        // 1000 0000
        POWERON_2S,        // 1100 0000
    };

    enum LongPressTime : uint8_t
    {                  // 长按键PEK触发时间（地址：0x36）
        LPRESS_1S = 0, // 00
        LPRESS_1_5S,   // 01
        LPRESS_2S,     // 10
        LPRESS_2_5S,   // 11
    };

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

    // Controlling and status registers
    static constexpr uint8_t REG_PWR_STAT = 0x00;         // Power status register
    static constexpr uint8_t REG_CHG_STAT = 0x01;         // Charge status register
    static constexpr uint8_t REG_DATA_BUFF = 0x06;        // 6 bytes data buffer register addressing from 0x06 to 0x0B
    static constexpr uint8_t REG_OUT_CTL = 0x12;          // Output control register
    static constexpr uint8_t REG_DCDC2_VOLT = 0x23;       // DCDC2 voltage setting register
    static constexpr uint8_t REG_DCDC1_VOLT = 0x26;       // DCDC1 voltage setting register
    static constexpr uint8_t REG_LDO4_VOLT = 0x27;        // LDO4 voltage setting register
    static constexpr uint8_t REG_LDO23_VOLT = 0x28;       // LDO2 and LDO3 voltage setting register
    static constexpr uint8_t REG_VBUS_CTL = 0x30;         // VBUS control register
    static constexpr uint8_t REG_VOFF_CTL = 0x31;         // Auto-power-off voltage setting register
    static constexpr uint8_t REG_SD_LED_CTL = 0x32;       // Shutdown and LED control register
    static constexpr uint8_t REG_CHG_CTL = 0x33;          // Charge voltage and current control register
    static constexpr uint8_t REG_CHG_TIMEOUT_CTL = 0x34;  // Charge timeout control register
    static constexpr uint8_t REG_PEK_CTL = 0x36;          // Power key control register
    static constexpr uint8_t REG_DCDC_FREQ = 0x37;        // DCDC frequency control register
    static constexpr uint8_t REG_TS_CHG_LOW_VOLT = 0x38;  // Thermal sensor low temperature voltage threshold register(charging)
    static constexpr uint8_t REG_TS_CHG_HIGH_VOLT = 0x39; // Thermal sensor high temperature voltage threshold register(charging)
    static constexpr uint8_t REG_APS_LOW_VOLT1 = 0x3A;    // APS low voltage threshold 1 register
    static constexpr uint8_t REG_APS_LOW_VOLT2 = 0x3B;    // APS low voltage threshold 2 register
    static constexpr uint8_t REG_TS_DIS_LOW_VOLT = 0x3C;  // Thermal sensor low temperature voltage threshold register(discharging)
    static constexpr uint8_t REG_TS_DIS_HIGH_VOLT = 0x3D; // Thermal sensor high temperature voltage threshold register(discharging)
    static constexpr uint8_t REG_DCDC_MODE = 0x80;        // DCDC mode setting register
    static constexpr uint8_t REG_ADC_EN = 0x82;           // ADC enable register
    static constexpr uint8_t REG_INT_TS_EN = 0x83;        // Internal temperature sensor enable register
    static constexpr uint8_t REG_ADC_TS_CTL = 0x84;       // ADC sample rate and temperature sensor control register
    static constexpr uint8_t REG_TIMER_CTL = 0x85;        // Timer control register
    static constexpr uint8_t REG_OVER_TEMP_SD = 0x8F;     // Over temperature shutdown control register

    // IRQ registers
    static constexpr uint8_t REG_IRQ_CTL1 = 0x40;
    static constexpr uint8_t REG_IRQ_CTL2 = 0x41;
    static constexpr uint8_t REG_IRQ_CTL3 = 0x42;
    static constexpr uint8_t REG_IRQ_CTL4 = 0x43;
    static constexpr uint8_t REG_IRQ_STAT1 = 0x44;
    static constexpr uint8_t REG_IRQ_STAT2 = 0x45;
    static constexpr uint8_t REG_IRQ_STAT3 = 0x46;
    static constexpr uint8_t REG_IRQ_STAT4 = 0x47;
    static constexpr uint8_t REG_IRQ_CTL5 = 0x4A;  // For timer interrupt only
    static constexpr uint8_t REG_IRQ_STAT5 = 0x4D; // For timer interrupt only

    // ADC data registers
    static constexpr uint8_t REG_ADC_ACIN_VH = 0x56;    // ACIN voltage high byte
    static constexpr uint8_t REG_ADC_ACIN_VL = 0x57;    // ACIN voltage low (half) byte
    static constexpr uint8_t REG_ADC_ACIN_CH = 0x58;    // ACIN current high byte
    static constexpr uint8_t REG_ADC_ACIN_CL = 0x59;    // ACIN current low (half) byte
    static constexpr uint8_t REG_ADC_VBUS_VH = 0x5A;    // VBUS voltage high byte
    static constexpr uint8_t REG_ADC_VBUS_VL = 0x5B;    // VBUS voltage low (half) byte
    static constexpr uint8_t REG_ADC_VBUS_CH = 0x5C;    // VBUS current high byte
    static constexpr uint8_t REG_ADC_VBUS_CL = 0x5D;    // VBUS current low (half) byte
    static constexpr uint8_t REG_ADC_INT_TS_H = 0x5E;   // Internal temperature sensor high byte
    static constexpr uint8_t REG_ADC_INT_TS_L = 0x5F;   // Internal temperature sensor low (half) byte
    static constexpr uint8_t REG_ADC_BAT_TS_H = 0x62;   // Battery temmperature sensor high byte
    static constexpr uint8_t REG_ADC_BAT_TS_L = 0x63;   // Battery temperature sensor low (half) byte
    static constexpr uint8_t REG_BAT_PWR_H = 0x70;      // Battery power high byte
    static constexpr uint8_t REG_BAT_PWR_M = 0x71;      // Battery power middle byte
    static constexpr uint8_t REG_BAT_PWR_L = 0x72;      // Battery power low byte
    static constexpr uint8_t REG_ADC_BAT_VH = 0x78;     // Battery voltage high byte
    static constexpr uint8_t REG_ADC_BAT_VL = 0x79;     // Battery voltage low (half) byte
    static constexpr uint8_t REG_ADC_BAT_CHG_CH = 0x7A; // Battery charge current high byte
    static constexpr uint8_t REG_ADC_BAT_CHG_CL = 0x7B; // Battery charge current low (half) byte
    static constexpr uint8_t REG_ADC_BAT_DIS_CH = 0x7C; // Battery discharge current high byte
    static constexpr uint8_t REG_ADC_BAT_DIS_CL = 0x7D; // Battery discharge current low (half) byte
    static constexpr uint8_t REG_ADC_APS_VH = 0x7E;     // APS voltage high byte
    static constexpr uint8_t REG_ADC_APS_VL = 0x7F;     // APS voltage low (half) byte

    // Coulometer registers
    static constexpr uint8_t REG_COL_CHG_DATA3 = 0xB0; // Coulometer charging data register 3
    static constexpr uint8_t REG_COL_CHG_DATA2 = 0xB1; // Coulometer charging data register 2
    static constexpr uint8_t REG_COL_CHG_DATA1 = 0xB2; // Coulometer charging data register 1
    static constexpr uint8_t REG_COL_CHG_DATA0 = 0xB3; // Coulometer charging data register 0
    static constexpr uint8_t REG_COL_DIS_DATA3 = 0xB4; // Coulometer discharging data register 3
    static constexpr uint8_t REG_COL_DIS_DATA2 = 0xB5; // Coulometer discharging data register 2
    static constexpr uint8_t REG_COL_DIS_DATA1 = 0xB6; // Coulometer discharging data register 1
    static constexpr uint8_t REG_COL_DIS_DATA0 = 0xB7; // Coulometer discharging data register 0
    static constexpr uint8_t REG_COL_CTL = 0xB8;       // Coulometer control register

public:
    AXP173(TwoWire &wire = Wire, uint8_t dev_addr = 0x34);
    bool begin();

    inline bool isACINExist()
    {
        return (readReg(REG_PWR_STAT) & 0B10000000) ? true : false;
    }

    inline bool isACINAvl()
    {
        return (readReg(REG_PWR_STAT) & 0B01000000) ? true : false;
    }

    inline bool isVBUSExist()
    {
        return (readReg(REG_PWR_STAT) & 0B00100000) ? true : false;
    }

    inline bool isVBUSAvl()
    {
        return (readReg(REG_PWR_STAT) & 0B00010000) ? true : false;
    }

    inline bool getBatCurrentDir()
    {
        return (readReg(REG_PWR_STAT) & 0B00000100) ? true : false;
    }

    inline bool isChipOverTemp()
    {
        return (readReg(REG_CHG_STAT) & 0B10000000) ? true : false;
    }

    inline bool isCharging()
    {
        return (readReg(REG_CHG_STAT) & 0B01000000) ? true : false;
    }

    inline bool isBatExist()
    {
        return (readReg(REG_CHG_STAT) & 0B00100000) ? true : false;
    }

    inline bool isChargeUnderCurrent()
    {
        return (readReg(REG_CHG_STAT) & 0B00000100) ? true : false;
    }

    /* Power output control （电源输出控制）*/
    void setOutputEnable(OutputChannel channel, bool state);        // channel：设置电源输出通道（OUTPUT_CHANNEL）；state：设置是否输出
    void setOutputVoltage(OutputChannel channel, uint16_t voltage); // channel：设置电源输出通道（OUTPUT_CHANNEL）；voltage：设置输出电压
                                                                    // DCDC1 & LDO4: 700~3500(mV), DCDC2: 700~2275(mV), LDO2 & LDO3: 1800~3300(mV)

    /* Basic control (开关芯片控制) */
    inline void powerOff(void)
    {
        writeReg(REG_SD_LED_CTL, (readReg(REG_SD_LED_CTL) | 0B10000000));
    }

    inline bool isPowerOff(void)
    {
        return (readReg(REG_SD_LED_CTL) & 0B10000000) == 0;
    }

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
    void setCoulometerEnable(bool enabled); // 设置库仑计使能
    void pauseCoulometer();                 // 暂停库仑计
    void resetCoulometer();                 // 清零库仑计

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

    /* IRQ support */
    void enableIRQs(const std::bitset<NUM_IRQn> irqs);
    void disableIRQs(const std::bitset<NUM_IRQn> irqs);
    void clearIRQFlags(const std::bitset<NUM_IRQn> irqs);
    std::bitset<NUM_IRQn> getIRQFlags();
};

#endif