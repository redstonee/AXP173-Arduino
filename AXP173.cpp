/**
 * @file AXP173.cpp
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
#include "AXP173.h"
#include <Arduino.h>

/**
 * @brief Clamp a value to a specified range.
 *
 * If input is less than min, it returns min; if input is greater than max, it returns max.
 *
 * @param input  The value to clamp.
 * @param min The minimum value of the range.
 * @param max The maximum value of the range.
 *
 * @return The clamped value, which will be within the range [min, max].
 *
 */
inline uint16_t clampToRange(uint16_t input, uint16_t min, uint16_t max)
{
    return std::max(std::min(input, max), min);
}

/**
 * @brief The AXP173 PMIC constructor.
 *
 * @param wire The TwoWire instance to use for I2C communication.
 * @param dev_addr The I2C device address of the AXP173 PMIC.
 */
AXP173::AXP173(TwoWire &wire, uint8_t dev_addr)
    : _wire(wire), _dev_addr(dev_addr)
{
}

/**
 * @brief Write a few registers in the AXP173 PMIC.
 *
 * This function writes a sequence of bytes to consecutive registers starting from the specified address.
 *
 * @param addr The starting address of the registers to write to.
 * @param buf The pointer to the buffer containing the data to write.
 * @param len The number of bytes to write from the buffer.
 *
 * @return true if the write operation was successful, false otherwise.
 */
bool AXP173::writeRegs(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(_dev_addr);
    for (uint8_t i = 0; i < len; ++i)
    {
        Wire.write(addr + i);
        Wire.write(buf[i]);
    }
    return Wire.endTransmission() == 0;
}

/**
 * @brief Read a few registers from the AXP173 PMIC.
 *
 * @param addr The starting address of the registers to read from.
 * @param buf The pointer to the buffer where the read data will be stored.
 * @param len The number of bytes to read into the buffer.
 *
 * @return The number of bytes successfully read from the registers.
 */
uint8_t AXP173::readRegs(uint8_t addr, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(_dev_addr);
    Wire.write(addr);
    if (Wire.endTransmission(false) != 0)
    {
        return 0; // Transmission failed
    }

    Wire.requestFrom(_dev_addr, len);
    uint8_t bytesRead = 0;
    while (Wire.available() && bytesRead < len)
    {
        buf[bytesRead++] = Wire.read();
    }
    return bytesRead;
}

/**
 * @brief Initialize the AXP173 PMIC.
 *
 * This function attempts to write a magic data sequence to the PMIC's data buffer
 * and reads it back to verify that the PMIC is responding correctly.
 * If the read data matches the magic data, it proceeds to set the PMIC configuration.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool AXP173::begin()
{
    // Try to write something into the PMIC's data buffer
    constexpr uint8_t magicData[]{0x11, 0x45, 0x14};
    writeRegs(REG_DATA_BUFF, magicData, sizeof(magicData));
    // Read back the data to check if the PMIC is responding
    uint8_t readBuf[sizeof(magicData)];
    if (readRegs(REG_DATA_BUFF, readBuf, sizeof(readBuf)) != sizeof(readBuf))
    {
        return false; // Initialization failed
    }
    // Check if the read data matches the magic data
    if (memcmp(readBuf, magicData, sizeof(magicData)) != 0)
    {
        return false; // Magic data mismatch
    }

    disableIRQs(NUM_IRQn);     // Disable all interrupts
    clearIRQFlags(NUM_IRQn);   // Clear all interrupts
    setCoulometerEnable(true); // 库仑计使能

    return true;
}

/**
 * @brief Set the output enable state for a specific output channel.
 *
 * @param channel The output channel to control (DCDC1, DCDC2, LDO2, LDO3, LDO4, EXTEN).
 * @param state The desired state for the output channel (true for enable, false for disable).
 */
void AXP173::setOutputEnable(OutputChannel channel, bool state)
{
    uint8_t buff = readReg(REG_OUT_CTL); // 读取寄存器0x12的值到buff中
    buff = state ? (buff | (1U << channel)) : (buff & ~(1U << channel));
    writeRegs(REG_OUT_CTL, &buff, 1); // 将修改后的值写回寄存器0x12
}

/* 电源输出电压配置寄存器
 * DC-DC2     0x23  25mV/step   7-6bit(stable)   5-0bit(usage)
 * DC-DC1     0x26  25mV/step   7bit(stable)     6-0bit(usage)
 * LDO4       0x27  25mV/step   7bit(stable)     6-0bit(usage)
 * LDO2&LDO3  0x28  100mV/step  None(stable)     7-4bit&3-0bit(usage)
 * 函数返回值：None
 * 函数作用：控制某一通道电源电压输出大小（具体见，h文件）
 *
 * 函数写法功能解析：_getMid()可以避免用户输入过大或者过小的值导致程序意外错误，输入过小值直接
 * 输出最小电压，过大值直接输出最大电压。然后将用户输入值转化为对应的电压乘数（step）
 * (buff & 0B10000000)：将buff转为八位二进制,重置电压设置位0-6bit,"维持保留位7bit" !!!
 * (voltage & 0B01111111)：将voltage转为八位二进制,维持待写入电压位0-6bit，"重置保留位7bit" !!!
 * (buff & 0B10000000) | (voltage & 0B01111111)：最后的或运算结束后，只有0-6bit改变了，7bit还仍
 * 然维持读取时的状态，起到了不更改保留位但却更改了电压位的作用！！！
 */
void AXP173::setOutputVoltage(OutputChannel channel, uint16_t voltage)
{
    uint8_t buff = 0;
    switch (channel)
    {
    case DCDC1:
        voltage = (clampToRange(voltage, 700, 3500) - 700) / 25; // 0 - 112(step)
        buff = readReg(REG_DCDC1_VOLT);
        buff = (buff & 0B10000000) | (voltage & 0B01111111);
        writeReg(REG_DCDC1_VOLT, buff);
        break;
    case DCDC2:
        voltage = (clampToRange(voltage, 700, 2275) - 700) / 25;
        buff = readReg(REG_DCDC2_VOLT);
        buff = (buff & 0B11000000) | (voltage & 0B00111111);
        writeReg(0x23, buff);
        break;
    case LDO2:
        voltage = (clampToRange(voltage, 1800, 3300) - 1800) / 100;
        buff = readReg(REG_LDO23_VOLT);
        buff = (buff & 0B00001111) | (voltage << 4);
        writeReg(REG_LDO23_VOLT, buff);
        break;
    case LDO3:
        voltage = (clampToRange(voltage, 1800, 3300) - 1800) / 100;
        buff = readReg(REG_LDO23_VOLT);
        buff = (buff & 0B11110000) | (voltage);
        writeReg(REG_LDO23_VOLT, buff);
        break;
    case LDO4:
        voltage = (clampToRange(voltage, 700, 3500) - 700) / 25;
        buff = readReg(REG_LDO4_VOLT);
        buff = (buff & 0B10000000) | (voltage & 0B01111111);
        writeReg(REG_LDO4_VOLT, buff);
        break;
    default:
        break;
    }
}

/* 长按按键芯片开关机时间设置寄存器（地址：0x36）
 * 函数返回值：None
 * 函数作用：设置长按按键芯片开关机时间（具体见，h文件）
 *
 * 函数写法功能解析："&" 运算先将 "4 and 5 bit" 置位，然后保留其它位已经写 "1" 的配置，
 * 最后 "|" 运算对 "4 and 5 bit" 写"入 " 0 or 1 "
 */
void AXP173::setPowerOnPressTime(PowerOnPressTime onTime)
{ // 7 and 6 bit   开机时间
    uint8_t buff = readReg(REG_PEK_CTL);
    buff &= 0B00111111;    // 保留前6位，重置7 and 6 bit
    buff |= (onTime << 6); // 将onTime左移6位后与buff做或运算
    writeReg(REG_PEK_CTL, buff);
}

void AXP173::setPowerOffPressTime(PowerOffPressTime offTime)
{ // 0 and 1 bit   关机时间
    uint8_t buff = readReg(REG_PEK_CTL);
    buff &= 0B11111100; // 保留前6位，重置0 and 1 bit
    buff |= (offTime);  // 将offTime直接与buff做或运算
    writeReg(REG_PEK_CTL, buff);
}

void AXP173::setLongPressTime(LongPressTime pressTime)
{
    uint8_t buff = readReg(REG_PEK_CTL);
    buff &= 0B11001111;       // 重置4 and 5 bit
    buff |= (pressTime << 4); // 将pressTime左移4位后与buff做或运算
    writeReg(REG_PEK_CTL, buff);
}

/* 充电控制寄存器1（地址：0x33）
 * 函数返回值：None
 * 函数作用：设置充电电流以及是否使能充电功能，充电目标电压(5 and 6 bit)默认为4.2V(1 and 0)
 *
 * 函数写法功能解析：同上 "&" 与位运算置位，"|" 或位运算写入
 */
void AXP173::setChargeEnable(bool state)
{ // 充电功能使能控制位7bit
    uint8_t buff = readReg(REG_CHG_CTL);
    if (state)
        buff |= 0B10000000; // 使能充电功能
    else
        buff &= 0B01111111; // 禁止充电功能
    writeReg(REG_CHG_CTL, buff);
}

void AXP173::setChargeCurrent(ChargeCurrent current)
{ // 写入电流见CHARGE_CURRENT枚举体
    uint8_t buff = readReg(REG_CHG_CTL);
    buff &= 0B11110000; // 保留前4位，重置3-0bit
    buff |= current;    // 将current直接与buff做或运算
    writeReg(REG_CHG_CTL, buff);
}

/* ADC使能寄存器1（地址：0x82）
 * 函数返回值：None
 * 函数作用：设置ADC使能
 *
 * 函数写法功能解析：同 "电源输出控制寄存器" 参数见ADC_CHANNEL枚举体
 */
void AXP173::setADCEnable(AdcChannel channel, bool state)
{
    uint8_t buff = readReg(REG_ADC_EN);
    if (state)
        buff |= (1U << channel); // 使能ADC通道
    else
        buff &= ~(1U << channel); // 禁止ADC通道
    writeReg(REG_ADC_EN, buff);
}

/* ADC使能寄存器2（地址：0x83）
 * 函数返回值：None
 * 函数作用：设置芯片温度检测ADC使能（默认开启）
 *
 * 函数写法功能解析：同上
 */
void AXP173::setChipTempEnable(bool state)
{
    uint8_t buff = readReg(REG_INT_TS_EN);
    if (state)
        buff |= 0B10000000; // 使能芯片温度检测
    else
        buff &= 0B01111111; // 禁止芯片温度检测
    writeReg(REG_INT_TS_EN, buff);
}

/* 库仑计控制寄存器（地址：0xB8）
 * 函数返回值：None
 * 函数作用：设置库仑计（开关ENABLE 7bit，暂停PAUSE 6bit，清零RESET 5bit。5 and 6 bit 操作结束后会自动置零，0-4bit为保留位）
 *
 * 函数写法功能解析：同上
 */
void AXP173::setCoulometerEnable(bool enabled)
{
    uint8_t buff = readReg(REG_COL_CTL);
    if (enabled)
        buff |= (1U << 7);
    else
        buff &= ~(1U << 7);
    writeReg(REG_COL_CTL, buff);
}

void AXP173::pauseCoulometer()
{
    uint8_t buff = readReg(REG_COL_CTL);
    buff |= (1U << 6); // 设置PAUSE位
    writeReg(REG_COL_CTL, buff);
}

void AXP173::resetCoulometer()
{
    uint8_t buff = readReg(REG_COL_CTL);
    buff |= (1U << 5); // 设置RESET位
    writeReg(REG_COL_CTL, buff);
}

/* 以上为设置寄存器数据 */
/* 以下为读取寄存器数据 */

/* IRQ引脚中断寄存器（地址：0xB8）
 * 函数返回值：None
 * 函数作用：设置库仑计（开关ENABLE 7bit，暂停PAUSE 6bit，清零RESET 5bit。5 and 6 bit 操作结束后会自动置零，0-4bit为保留位）
 *
 * 函数写法功能解析：同上
 */

/* 库仑计数据读取寄存器（地址：0xB0、0xB4）(待理解)
 * 函数返回值：float
 * 函数作用：读取库仑计数值并且计算电池电量
 *
 * 函数写法功能解析：(2^16) * 0.5 * (int32_t)(CCR - DCR) / 3600.0 / 25.0;
 *
 * SOC = RM / FCC：荷电状态% = 剩余电容量 / 完全充电容量
 * RM = CCR - DCR：剩余电容量 = 充电计数器寄存器 - 放电计数器寄存器
 *
 * (2^16)：65536
 * 0.5：current_LSB（数据手册说库仑计精度为0.5%）
 * (int32_t)：将结果强制转换为正数
 * (CCR - DCR)：剩余电容量mAh
 * 3600.0：1mAh = 0.001A * 3600s = 3.6库仑，将剩余电容量转化为电流
 * 25.0：ADC rate (0x84寄存器 "6 and 7 bit" 默认为 "00",使得ADC速率为"25*(2^0)"Hz)
 */
inline uint32_t AXP173::getCoulometerChargeDataRaw(void)
{ // 电池充电库仑计数据寄存器3（积分后数据）
    uint8_t buff[4];
    if (readRegs(REG_COL_CHG_DATA3, buff, 4) != 4)
    {
        return 0; // 读取失败
    }
    // 将4个字节合并为一个32位整数
    uint32_t data = (static_cast<uint32_t>(buff[0]) << 24) |
                    (static_cast<uint32_t>(buff[1]) << 16) |
                    (static_cast<uint32_t>(buff[2]) << 8) |
                    static_cast<uint32_t>(buff[3]);
    return data;
}

inline uint32_t AXP173::getCoulometerDischargeDataRaw(void)
{ // 电池放电库仑计数据寄存器3（积分后数据）
    uint8_t buff[4];
    if (readRegs(REG_COL_DIS_DATA3, buff, 4) != 4)
    {
        return 0; // 读取失败
    }
    // 将4个字节合并为一个32位整数
    uint32_t data = (static_cast<uint32_t>(buff[0]) << 24) |
                    (static_cast<uint32_t>(buff[1]) << 16) |
                    (static_cast<uint32_t>(buff[2]) << 8) |
                    static_cast<uint32_t>(buff[3]);
    return data;
}

float AXP173::getCoulometerChargeData(void)
{
    uint32_t ReData = getCoulometerChargeDataRaw();
    return ReData * 65536 * 0.5f / 3600 / 25.0f;
}

float AXP173::getCoulometerDischargeData(void)
{
    uint32_t ReData = getCoulometerDischargeDataRaw();
    return ReData * 65536 * 0.5f / 3600 / 25.0f;
}

float AXP173::getCoulometerData(void)
{ // 返回库仑计计算数据
    uint32_t coin = getCoulometerChargeDataRaw();
    uint32_t coout = getCoulometerDischargeDataRaw();
    // data = 65536 * current_LSB（电池电流ADC精度为0.5mA） * (coin - coout) / 3600（单位换算） / ADC rate (0x84寄存器 "6 and 7 bit" 默认为 "00",使得ADC速率为"25*(2^0)"Hz)
    float CCC = 65536 * 0.5f * (int32_t)(coin - coout) / 3600.0f / 25.0f;
    return CCC;
}

/**
 * @brief Get the battery voltage.
 *
 * @return float The battery voltage in millivolts.
 */
float AXP173::getBatVoltage()
{
    constexpr float ADCLSB = 1.1;
    uint8_t buff[2];
    if (readRegs(REG_ADC_BAT_VH, buff, 2) != 2)
    {
        return 0; // 读取失败
    }
    // 将高八位和低四位合并为一个12位整数
    uint16_t adcValue = (static_cast<uint16_t>(buff[0]) << 4) | buff[1];
    return adcValue * ADCLSB;
}

/**
 * @brief Get the battery current.
 *
 * @return float The battery current in milliamperes.
 */
float AXP173::getBatCurrent()
{
    constexpr float ADCLSB = 0.5;
    uint8_t buff[4];
    if (readRegs(REG_ADC_BAT_CHG_CH, buff, 4) != 4)
    {
        return 0; // 读取失败
    }

    // 将高八位和低四位合并为一个12位整数
    uint16_t CurrentIn = (static_cast<uint16_t>(buff[0]) << 4) | buff[1];
    uint16_t CurrentOut = (static_cast<uint16_t>(buff[2]) << 4) | buff[3];
    return (CurrentIn - CurrentOut) * ADCLSB;
}

// 返回高八位 + 中八位 + 低八位电池瞬时功率  地址：高0x70 中0x71 低0x72 数据乘以精度减小误差
float AXP173::getBatPower()
{
    constexpr float VoltageLSB = 1.1;
    constexpr float CurrentLCS = 0.5;
    uint8_t buff[3];
    if (readRegs(REG_BAT_PWR_H, buff, 3) != 3)
    {
        return 0; // 读取失败
    }
    // 将高八位、中八位和低八位合并为一个24位整数
    uint32_t ReData = (static_cast<uint32_t>(buff[0]) << 16) |
                      (static_cast<uint32_t>(buff[1]) << 8) |
                      static_cast<uint32_t>(buff[2]);

    return VoltageLSB * CurrentLCS * ReData / 1000.0;
}

// 返回高八位 + 低四位USB输入电压   地址：高0x5A 低0x5B  精度：1.7mV
float AXP173::getVBUSVoltage()
{
    constexpr float ADCLSB = 1.7;
    uint8_t buff[2];
    if (readRegs(REG_ADC_VBUS_VH, buff, 2) != 2)
    {
        return 0; // 读取失败
    }
    // 将高八位和低四位合并为一个12位整数
    uint16_t adcValue = (static_cast<uint16_t>(buff[0]) << 4) | buff[1];
    // 返回电压值，单位为毫伏
    return adcValue * ADCLSB;
}

// 返回高八位 + 低四位USB输入电流   地址：高0x5C 低0x5D  精度：0.375mA
float AXP173::getVBUSCurrent()
{
    constexpr float ADCLSB = 0.375;
    uint8_t buff[2];
    if (readRegs(REG_ADC_VBUS_CH, buff, 2) != 2)
    {
        return 0; // 读取失败
    }
    // 将高八位和低四位合并为一个12位整数
    uint16_t adcValue = (static_cast<uint16_t>(buff[0]) << 4) | buff[1];
    // 返回电流值，单位为毫安
    return adcValue * ADCLSB;
}

// 返回高八位 + 低四位芯片内置温度传感器温度 地址：高0x5E 低0x5F 精度：0.1℃  最小值-144.7℃
float AXP173::getChipTemp()
{
    constexpr float ADCLSB = 0.1;
    constexpr float OFFSET_DEG_C = -144.7;

    uint8_t buff[2];
    if (readRegs(REG_ADC_INT_TS_H, buff, 2) != 2)
    {
        return 0; // 读取失败
    }
    // 将高八位和低四位合并为一个12位整数
    uint16_t adcValue = (static_cast<uint16_t>(buff[0]) << 4) | buff[1];

    return OFFSET_DEG_C + adcValue * ADCLSB;
}

// 返回高八位 + 低四位芯片TS脚热敏电阻检测到的电池温度  地址：高0x62 低0x63 精度：0.1℃  最小值-144.7℃
float AXP173::getTSTemp()
{
    constexpr float ADCLSB = 0.1;
    constexpr float OFFSET_DEG_C = -144.7;

    uint8_t buff[2];
    if (readRegs(REG_ADC_BAT_TS_H, buff, 2) != 2)
    {
        return 0; // 读取失败
    }
    // 将高八位和低四位合并为一个12位整数
    uint16_t adcValue = (static_cast<uint16_t>(buff[0]) << 4) | buff[1];

    return OFFSET_DEG_C + adcValue * ADCLSB;
}

/* 按键状态检测 */
void AXP173::setPowerKeyShutdownEnable(bool enabled)
{ // 按键时长大于关机时长自动关机
    uint8_t buff = readReg(REG_PEK_CTL);
    if (enabled)
        buff |= 0B00001000; // 设置第3位为1，表示按键时长大于关机时长关机
    else
        buff &= 0B11110111; // 设置第3位为0，表示按键时长大于关机时长不关机
    writeReg(REG_PEK_CTL, buff);
}

/**
 * @brief Enable or disable specific IRQs.
 *
 * @param irqs A bitset representing the IRQs to enable or disable.
 */
void AXP173::enableIRQs(std::bitset<NUM_IRQn> irqs)
{
    uint32_t irqControl;
    readRegs(REG_IRQ_CTL1, reinterpret_cast<uint8_t *>(&irqControl), 4);
    irqControl |= irqs.to_ulong(); // 将要使能的中断位设置为1
    writeRegs(REG_IRQ_CTL1, reinterpret_cast<const uint8_t *>(&irqControl), 4);
}

/**
 * @brief Disable specific IRQs.
 *
 * @param irqs A bitset representing the IRQs to disable.
 */
void AXP173::disableIRQs(std::bitset<NUM_IRQn> irqs)
{
    uint32_t irqControl;
    readRegs(REG_IRQ_CTL1, reinterpret_cast<uint8_t *>(&irqControl), 4);
    irqControl &= ~irqs.to_ulong(); // 将要禁用的中断位设置为0
    writeRegs(REG_IRQ_CTL1, reinterpret_cast<const uint8_t *>(&irqControl), 4);
}

/**
 * @brief Clear specific IRQ flags.
 *
 * @param irqs A bitset representing the IRQs to clear.
 */
void AXP173::clearIRQFlags(std::bitset<NUM_IRQn> irqs)
{
    uint32_t irqStatus;
    readRegs(REG_IRQ_STAT1, reinterpret_cast<uint8_t *>(&irqStatus), 4);
    irqStatus &= irqs.to_ulong(); // 清除指定的中断标志位
    writeRegs(REG_IRQ_STAT1, reinterpret_cast<const uint8_t *>(&irqStatus), 4);
}

/**
 * @brief Get the pending IRQ flags.
 *
 * @return std::bitset<NUM_IRQn> A bitset representing the pending IRQ flags.
 */
std::bitset<AXP173::NUM_IRQn> AXP173::getIRQFlags()
{
    uint32_t irqStatus;
    readRegs(REG_IRQ_STAT1, reinterpret_cast<uint8_t *>(&irqStatus), 4);
    std::bitset<NUM_IRQn> flags(irqStatus);
    return flags;
}
