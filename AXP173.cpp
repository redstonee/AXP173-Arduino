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

// Controlling and status registers
constexpr uint8_t REG_PWR_STAT = 0x00;         // Power status register
constexpr uint8_t REG_CHG_STAT = 0x01;         // Charge status register
constexpr uint8_t REG_DATA_BUFF = 0x06;        // 6 bytes data buffer register addressing from 0x06 to 0x0B
constexpr uint8_t REG_OUT_CTL = 0x12;          // Output control register
constexpr uint8_t REG_DCDC2_VOLT = 0x23;       // DCDC2 voltage setting register
constexpr uint8_t REG_DCDC1_VOLT = 0x26;       // DCDC1 voltage setting register
constexpr uint8_t REG_LDO4_VOLT = 0x27;        // LDO4 voltage setting register
constexpr uint8_t REG_LDO23_VOLT = 0x28;       // LDO2 and LDO3 voltage setting register
constexpr uint8_t REG_VBUS_CTL = 0x30;         // VBUS control register
constexpr uint8_t REG_VOFF_CTL = 0x31;         // Auto-power-off voltage setting register
constexpr uint8_t REG_SD_LED_CTL = 0x32;       // Shutdown and LED control register
constexpr uint8_t REG_CHG_CTL = 0x33;          // Charge voltage and current control register
constexpr uint8_t REG_CHG_TIMEOUT_CTL = 0x34;  // Charge timeout control register
constexpr uint8_t REG_PEK_CTL = 0x36;          // Power key control register
constexpr uint8_t REG_DCDC_FREQ = 0x37;        // DCDC frequency control register
constexpr uint8_t REG_TS_CHG_LOW_VOLT = 0x38;  // Thermal sensor low temperature voltage threshold register(charging)
constexpr uint8_t REG_TS_CHG_HIGH_VOLT = 0x39; // Thermal sensor high temperature voltage threshold register(charging)
constexpr uint8_t REG_APS_LOW_VOLT1 = 0x3A;    // APS low voltage threshold 1 register
constexpr uint8_t REG_APS_LOW_VOLT2 = 0x3B;    // APS low voltage threshold 2 register
constexpr uint8_t REG_TS_DIS_LOW_VOLT = 0x3C;  // Thermal sensor low temperature voltage threshold register(discharging)
constexpr uint8_t REG_TS_DIS_HIGH_VOLT = 0x3D; // Thermal sensor high temperature voltage threshold register(discharging)
constexpr uint8_t REG_DCDC_MODE = 0x80;        // DCDC mode setting register
constexpr uint8_t REG_ADC_EN = 0x82;           // ADC enable register
constexpr uint8_t REG_INT_TS_EN = 0x83;        // Internal temperature sensor enable register
constexpr uint8_t REG_ADC_TS_CTL = 0x84;       // ADC sample rate and temperature sensor control register
constexpr uint8_t REG_TIMER_CTL = 0x85;        // Timer control register
constexpr uint8_t REG_OVER_TEMP_SD = 0x8F;     // Over temperature shutdown control register

// IRQ registers
constexpr uint8_t REG_IRQ_CTL1 = 0x40;
constexpr uint8_t REG_IRQ_CTL2 = 0x41;
constexpr uint8_t REG_IRQ_CTL3 = 0x42;
constexpr uint8_t REG_IRQ_CTL4 = 0x43;
constexpr uint8_t REG_IRQ_STAT1 = 0x44;
constexpr uint8_t REG_IRQ_STAT2 = 0x45;
constexpr uint8_t REG_IRQ_STAT3 = 0x46;
constexpr uint8_t REG_IRQ_STAT4 = 0x47;
constexpr uint8_t REG_IRQ_CTL5 = 0x4A;  // For timer interrupt only
constexpr uint8_t REG_IRQ_STAT5 = 0x4D; // For timer interrupt only

// ADC data registers
constexpr uint8_t REG_ADC_ACIN_VH = 0x56;    // ACIN voltage high byte
constexpr uint8_t REG_ADC_ACIN_VL = 0x57;    // ACIN voltage low (half) byte
constexpr uint8_t REG_ADC_ACIN_CH = 0x58;    // ACIN current high byte
constexpr uint8_t REG_ADC_ACIN_CL = 0x59;    // ACIN current low (half) byte
constexpr uint8_t REG_ADC_VBUS_VH = 0x5A;    // VBUS voltage high byte
constexpr uint8_t REG_ADC_VBUS_VL = 0x5B;    // VBUS voltage low (half) byte
constexpr uint8_t REG_ADC_VBUS_CH = 0x5C;    // VBUS current high byte
constexpr uint8_t REG_ADC_VBUS_CL = 0x5D;    // VBUS current low (half) byte
constexpr uint8_t REG_ADC_INT_TS_H = 0x5E;   // Internal temperature sensor high byte
constexpr uint8_t REG_ADC_INT_TS_L = 0x5F;   // Internal temperature sensor low (half) byte
constexpr uint8_t REG_ADC_BAT_TS_H = 0x62;   // Battery temmperature sensor high byte
constexpr uint8_t REG_ADC_BAT_TS_L = 0x63;   // Battery temperature sensor low (half) byte
constexpr uint8_t REG_BAT_PWR_H = 0x70;      // Battery power high byte
constexpr uint8_t REG_BAT_PWR_M = 0x71;      // Battery power middle byte
constexpr uint8_t REG_BAT_PWR_L = 0x72;      // Battery power low byte
constexpr uint8_t REG_ADC_BAT_VH = 0x78;     // Battery voltage high byte
constexpr uint8_t REG_ADC_BAT_VL = 0x79;     // Battery voltage low (half) byte
constexpr uint8_t REG_ADC_BAT_CHG_CH = 0x7A; // Battery charge current high byte
constexpr uint8_t REG_ADC_BAT_CHG_CL = 0x7B; // Battery charge current low (half) byte
constexpr uint8_t REG_ADC_BAT_DIS_CH = 0x7C; // Battery discharge current high byte
constexpr uint8_t REG_ADC_BAT_DIS_CL = 0x7D; // Battery discharge current low (half) byte
constexpr uint8_t REG_ADC_APS_VH = 0x7E;     // APS voltage high byte
constexpr uint8_t REG_ADC_APS_VL = 0x7F;     // APS voltage low (half) byte

// Coulometer registers
constexpr uint8_t REG_COL_CHG_DATA3 = 0xB0; // Coulometer charging data register 3
constexpr uint8_t REG_COL_CHG_DATA2 = 0xB1; // Coulometer charging data register 2
constexpr uint8_t REG_COL_CHG_DATA1 = 0xB2; // Coulometer charging data register 1
constexpr uint8_t REG_COL_CHG_DATA0 = 0xB3; // Coulometer charging data register 0
constexpr uint8_t REG_COL_DIS_DATA3 = 0xB4; // Coulometer discharging data register 3
constexpr uint8_t REG_COL_DIS_DATA2 = 0xB5; // Coulometer discharging data register 2
constexpr uint8_t REG_COL_DIS_DATA1 = 0xB6; // Coulometer discharging data register 1
constexpr uint8_t REG_COL_DIS_DATA0 = 0xB7; // Coulometer discharging data register 0
constexpr uint8_t REG_COL_CTL = 0xB8;       // Coulometer control register

inline uint16_t clampToRange(uint16_t input, uint16_t min, uint16_t max)
{
    return std::max(std::min(input, max), min);
}

AXP173::AXP173(TwoWire &wire, uint8_t dev_addr) : _wire(wire), _dev_addr(dev_addr)
{
}

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

/* Public functions (包含IIC的初始化以及是否初始化的判断)*/
bool AXP173::begin(TwoWire *wire)
{
    // Try to write something into the PMIC's data buffer
    constexpr uint8_t magicData[]{0x11, 0x45, 0x14, 0x19, 0x19, 0x81};
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

    /* Set PMU Config */
    setPmuConfig();
    return true;
}

// 写在一切IIC设备初始化前面，电源芯片必须第一个初始化，并且在其他设备iic初始化之前设置好电压，否则其他设备程序初始化完结果没供电。
void AXP173::setPmuPower()
{ // 电源通道电压输出设置，交换位置可以设置上电时序，中间加delay可以延迟上电
    /* Enable and set LDO2 voltage */
    setOutputEnable(OP_LDO2, true);  // LDO2设置为输出
    setOutputVoltage(OP_LDO2, 3300); // LDO2电压设置为3.000V

    /* Enable and set LDO3 voltage */
    setOutputEnable(OP_LDO3, true);  // LDO3设置为输出
    setOutputVoltage(OP_LDO3, 3300); // LDO3电压设置为3.300V

    /* Enable and set LDO4 voltage */
    setOutputEnable(OP_LDO4, true);  // LDO4设置为输出
    setOutputVoltage(OP_LDO4, 3300); // LDO4电压设置为3.300V

    /* Enable and set DCDC1 voltage */
    setOutputEnable(OP_DCDC1, true);  // DCDC1设置为输出
    setOutputVoltage(OP_DCDC1, 3300); // DCDC1电压设置为3.300V

    /* Enable and set DCDC2 voltage */
    setOutputEnable(OP_DCDC2, true);  // DCDC2设置为输出
    setOutputVoltage(OP_DCDC2, 2275); // DCDC2电压设置为2.275V

    /* Enable Battery Charging */
    setChargeEnable(true);       // 充电功能使能
    setChargeCurrent(CHG_450mA); // 设置充电电流为450mA
}
void AXP173::setPmuConfig()
{ // 电源芯片ADC，库仑计等功能设置
    /* Clear IRQ */
    initIRQState();

    /* Set on time */
    setPowerOnTime(POWERON_1S); // 设置PEK开机时长为1S

    /* Set off time */
    setPowerOffTime(POWEROFF_4S); // 设置PEK关机时长为4S（我这个芯片因为定制好像只能设置6，8，10s）

    /* Set PEKLongPress time */
    setLongPressTime(LPRESS_1_5S); // 设置PEK长按键时长为1.5S

    /* Enable VBUS ADC */
    setADCEnable(ADC_VBUS_V, true); // VBUS ADC 电压使能
    setADCEnable(ADC_VBUS_C, true); // VBUS ADC 电流使能

    /* Enable Battery ADC */
    setADCEnable(ADC_BAT_V, true); // Battery ADC 电压使能
    setADCEnable(ADC_BAT_C, true); // Battery ADC 电流使能

    /* Enable Coulometer and set COULOMETER_ENABLE*/
    setCoulometer(COULOMETER_ENABLE, true); // 库仑计使能
}

/* 输入电源状态寄存器（地址：0x00）
 * 函数返回值：0 or 1
 * 函数作用：（具体见，h文件）
 * 函数写法功能解析：
 *  首先判断从输入电源状态寄存器读取的8bit值（由高到低读取7-->0）与0Bxxxxxxxx进行与运算，若两者相同则输出则为true反之为false
 * 再判断如下语句：  ("true" or" false") ? true ：false;   意为：若条件为true则返回1,反之返回0
 * 优点：直观简洁，避免使用if...else...语句拖慢处理效率
 */
bool AXP173::isACINExist()
{
    return (readReg(REG_PWR_STAT) & 0B10000000) ? true : false;
}

bool AXP173::isACINAvl()
{
    return (readReg(REG_PWR_STAT) & 0B01000000) ? true : false;
}

bool AXP173::isVBUSExist()
{
    return (readReg(REG_PWR_STAT) & 0B00100000) ? true : false;
}

bool AXP173::isVBUSAvl()
{
    return (readReg(REG_PWR_STAT) & 0B00010000) ? true : false;
}

bool AXP173::getBatCurrentDir()
{
    return (readReg(REG_PWR_STAT) & 0B00000100) ? true : false;
}

/* 电源工作模式以及充电状态指示寄存器（地址：0x01）
 * 函数返回值：0 or 1
 * 函数作用：（具体见，h文件）
 * 函数写法功能解析：（同上）
 */
bool AXP173::isAXP173OverTemp()
{
    return (readReg(REG_CHG_STAT) & 0B10000000) ? true : false;
}

bool AXP173::isCharging()
{
    return (readReg(REG_CHG_STAT) & 0B01000000) ? true : false;
}

bool AXP173::isBatExist()
{
    return (readReg(REG_CHG_STAT) & 0B00100000) ? true : false;
}

bool AXP173::isChargeCsmaller()
{
    return (readReg(REG_CHG_STAT) & 0B00000100) ? true : false;
}

/* 电源输出控制寄存器（地址：0x12）
 * 函数返回值：None
 * 函数作用：开关某一通道电源输出（具体见，h文件）
 * 函数写法功能解析：(见附件：pmu_outPutState_test.c)
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
    case OP_DCDC1:
        voltage = (clampToRange(voltage, 700, 3500) - 700) / 25; // 0 - 112(step)
        buff = readReg(REG_DCDC1_VOLT);
        buff = (buff & 0B10000000) | (voltage & 0B01111111);
        writeReg(REG_DCDC1_VOLT, buff);
        break;
    case OP_DCDC2:
        voltage = (clampToRange(voltage, 700, 2275) - 700) / 25;
        buff = readReg(REG_DCDC2_VOLT);
        buff = (buff & 0B11000000) | (voltage & 0B00111111);
        writeReg(0x23, buff);
        break;
    case OP_LDO2:
        voltage = (clampToRange(voltage, 1800, 3300) - 1800) / 100;
        buff = readReg(REG_LDO23_VOLT);
        buff = (buff & 0B00001111) | (voltage << 4);
        writeReg(REG_LDO23_VOLT, buff);
        break;
    case OP_LDO3:
        voltage = (clampToRange(voltage, 1800, 3300) - 1800) / 100;
        buff = readReg(REG_LDO23_VOLT);
        buff = (buff & 0B11110000) | (voltage);
        writeReg(REG_LDO23_VOLT, buff);
        break;
    case OP_LDO4:
        voltage = (clampToRange(voltage, 700, 3500) - 700) / 25;
        buff = readReg(REG_LDO4_VOLT);
        buff = (buff & 0B10000000) | (voltage & 0B01111111);
        writeReg(REG_LDO4_VOLT, buff);
        break;
    default:
        break;
    }
}

/* 开关芯片控制寄存器（地址：0x32）
 * 函数返回值：None
 * 函数作用：开关芯片输出（具体见，h文件）
 *
 * 函数写法功能解析：给该寄存器7bit位写 "1" 会关闭芯片所有输出，为了不改变该寄存器的其它
 * 位设置，让二进制形式仅对7bit写一"0B1000 0000"和读取到的原寄存器状态做 "|" 或位运算即
 * 可。由于关机后该位上电会自动置 "0",因此更改时可以不使用 "&" 与位运算重置该位.
 */
void AXP173::powerOFF(void)
{ // 关闭芯片所有输出
    writeReg(REG_SD_LED_CTL, (readReg(REG_SD_LED_CTL) | 0B10000000));
}

bool AXP173::powerState(void)
{ // 若关机则返回false
    return (readReg(REG_SD_LED_CTL) & 0B10000000) ? false : true;
}

/* 长按按键芯片开关机时间设置寄存器（地址：0x36）
 * 函数返回值：None
 * 函数作用：设置长按按键芯片开关机时间（具体见，h文件）
 *
 * 函数写法功能解析："&" 运算先将 "4 and 5 bit" 置位，然后保留其它位已经写 "1" 的配置，
 * 最后 "|" 运算对 "4 and 5 bit" 写"入 " 0 or 1 "
 */
void AXP173::setPowerOnTime(PowerOnPressTime onTime)
{ // 7 and 6 bit   开机时间
    uint8_t buff = readReg(REG_PEK_CTL);
    buff &= 0B00111111;    // 保留前6位，重置7 and 6 bit
    buff |= (onTime << 6); // 将onTime左移6位后与buff做或运算
    writeReg(REG_PEK_CTL, buff);
}

void AXP173::setPowerOffTime(PowerOffPressTime offTime)
{ // 0 and 1 bit   关机时间
    uint8_t buff = readReg(REG_PEK_CTL);
    buff &= 0B11111100; // 保留前6位，重置0 and 1 bit
    buff |= (offTime);  // 将offTime直接与buff做或运算
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
void AXP173::setCoulometer(CoulometerStatus option, bool state)
{
    uint8_t buff = readReg(REG_COL_CTL);
    if (state)
        buff |= (1U << option); // 使能库仑计状态
    else
        buff &= ~(1U << option); // 禁止库仑计状态
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
    float ADCLSB = 1.1;
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
    float ADCLSB = 0.5;
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
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData = _I2C_read24Bit(0x70);
    return VoltageLSB * CurrentLCS * ReData / 1000.0;
}

/* uint32_t AXP173::getChargeTimeMS() {

    static uint32_t chargeTime = 0;
    static uint32_t startTime = 0;
    uint32_t nowTime = millis();

    if (isCharging())
    {
        chargeTime = nowTime - startTime;
        return chargeTime;
    }
    else
    {
        startTime = nowTime;
        return chargeTime;
    }

} */

// 返回高八位 + 低四位USB输入电压   地址：高0x5A 低0x5B  精度：1.7mV
float AXP173::getVBUSVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = _I2C_read12Bit(0x5A);
    return ReData * ADCLSB;
}

// 返回高八位 + 低四位USB输入电流   地址：高0x5C 低0x5D  精度：0.375mA
float AXP173::getVBUSCurrent()
{
    float ADCLSB = 0.375;
    uint16_t ReData = _I2C_read12Bit(0x5C);
    return ReData * ADCLSB;
}

// 返回高八位 + 低四位芯片内置温度传感器温度 地址：高0x5E 低0x5F 精度：0.1℃  最小值-144.7℃
float AXP173::getAXP173Temp()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = _I2C_read12Bit(0x5E);
    return OFFSET_DEG_C + ReData * ADCLSB;
}

// 返回高八位 + 低四位芯片TS脚热敏电阻检测到的电池温度  地址：高0x62 低0x63 精度：0.1℃  最小值-144.7℃
float AXP173::getTSTemp()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = _I2C_read12Bit(0x62);
    return OFFSET_DEG_C + ReData * ADCLSB;
}

/* 按键状态检测 */
void AXP173::aoToPowerOFFEnabale(void)
{ // 按键时长大于关机时长自动关机
    _I2C_write1Byte(0x36, (_I2C_read8Bit(0x36) | 0B00001000));
}
void AXP173::initIRQState(void)
{ // 所有IRQ中断使能置零REG40H 41H 42H 43H 4AH
    _I2C_write1Byte(0x40, ((_I2C_read8Bit(0x40) & 0B00000001) | 0B00000000));
    _I2C_write1Byte(0x41, ((_I2C_read8Bit(0x41) & 0B00000000) | 0B00000000));
    _I2C_write1Byte(0x42, ((_I2C_read8Bit(0x42) & 0B00000100) | 0B00000000));
    _I2C_write1Byte(0x43, ((_I2C_read8Bit(0x43) & 0B11000010) | 0B00000000));
    _I2C_write1Byte(0x4A, ((_I2C_read8Bit(0x4A) & 0B01111111) | 0B00000000));
}

void AXP173::setShortPressEnabale(void)
{ // 短按键使能REG31H[3] 调用后立刻导致短按键中断发生
    _I2C_write1Byte(0x31, (_I2C_read8Bit(0x31) | 0B00001000));
}
bool AXP173::getShortPressIRQState(void)
{ // 读取短按键IRQ中断状态
    return (_I2C_read8Bit(0x46) & 0B00000010) ? true : false;
}
void AXP173::setShortPressIRQDisabale(void)
{ // 短按键对应位写1结束中断
    _I2C_write1Byte(0x46, (_I2C_read8Bit(0x46) | 0B00000010));
}

void AXP173::setLongPressTime(LongPressTime pressTime)
{ // 设置长按键触发时间 5 and 4 bit
    _I2C_write1Byte(0x36, ((_I2C_read8Bit(0x36) & 0B11001111) | pressTime));
}
bool AXP173::getLongPressIRQState(void)
{ // 读取长按键IRQ中断状态
    return (_I2C_read8Bit(0x46) & 0B00000001) ? true : false;
}
void AXP173::setLongPressIRQDisabale(void)
{ // 长按键对应位写1结束中断
    _I2C_write1Byte(0x46, (_I2C_read8Bit(0x46) | 0B00000001));
}

/* 按键与睡眠 */
void AXP173::prepareToSleep(void)
{ // ldo断电

    // setOutputEnable(AXP173::OP_LDO3, false);     //LDO3关闭输出
}

void AXP173::lightSleep(uint64_t time_in_us)
{ // 类似于锁屏，需要打开REG31[3]

    // prepareToSleep();
}

void AXP173::deepSleep(uint64_t time_in_us)
{ // ldo断电加MCU低功耗模式
  // prepareToSleep();

    // /* nnn */
    // setSleepMode (WiFiSleepType_t type, int listenInterval=0);  //WiFiSleepType_t type, int listenInterval=0

    // RestoreFromLightSleep();
}

void AXP173::RestoreFromLightSleep(void)
{ // ldo重启输出

    // setOutputEnable(AXP173::OP_LDO3, true);     //LDO3设置为输出
    // setOutputVoltage(AXP173::OP_LDO3, 3300);    //LDO3电压设置为3.300V
}
