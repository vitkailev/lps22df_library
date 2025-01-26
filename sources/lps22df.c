#include "lps22df.h"

static const uint8_t WHOAMI = 0xB4;

enum LPS22DF_RegAddresses {
    INTERRUPT_CFG_ADDR = 0x0B, // Interrupt register
    THS_P_L_ADDR, //Pressure threshold registers
    THS_P_H_ADDR,
    IF_CTRL_ADDR, // Interface control register
    WHO_AM_I_ADDR,
    CTRL_REG1_ADDR, // Control registers
    CTRL_REG2_ADDR,
    CTRL_REG3_ADDR,
    CTRL_REG4_ADDR,
    FIFO_CTRL_ADDR, // FIFO configuration registers
    FIFO_WTM_ADDR,
    REF_P_L_ADDR, // Reference pressure registers
    REF_P_H_ADDR,
    I3C_IF_CTRL_ADDR = 0x19, // Interface configuration register
    RPDS_L_ADDR, // Pressure offset registers
    RPDS_H_ADDR,
    INT_SOURCE_ADDR = 0x24, // Interrupt register
    FIFO_STATUS1_ADDR, // FIFO status registers
    FIFO_STATUS2_ADDR,
    STATUS_ADDR, // Status register
    PRESSURE_OUT_XL_ADDR, // Pressure output registers
    PRESSURE_OUT_L_ADDR,
    PRESSURE_OUT_H_ADDR,
    TEMP_OUT_L_ADDR, // Temperature output registers
    TEMP_OUT_H_ADDR,
    FIFO_DATA_OUT_PRESS_XL_ADDR = 0x78, // FIFO pressure output registers
    FIFO_DATA_OUT_PRESS_L_ADDR,
    FIFO_DATA_OUT_PRESS_H_ADDR
};

/**
 * @brief Check, that the pressure sensor is initialized
 * @param lps is the LPS22DF data structure
 * @return True - sensor has been initialized, otherwise - False
 */
static bool isInit(const LPS22DF_Def *lps) {
    return lps->isInit;
}

/**
 * @brief Check, that the pressure sensor is reading register values
 * @param lps is the LPS22DF data structure
 * @return True - is reading, otherwise - False
 */
static bool isReading(const LPS22DF_Def *lps) {
    return lps->isReading;
}

/**
 * @brief Write/update the register value
 * @param lps is the LPS22DF data structure
 * @param addr is the register address
 * @param value is the required register value
 * @param size is the value size (1 or 2 bytes)
 * @return LPS22DF_Errors values
 */
static int setting(LPS22DF_Def *lps, uint8_t addr, uint16_t value, uint8_t size) {
    if (!isInit(lps))
        return LPS22DF_NOT_INIT;
    if (isReading(lps))
        return LPS22DF_BUSY;

    uint8_t data[3] = {addr};
    data[1] = value & 0xFF;
    if (size == sizeof(uint16_t))
        data[2] = value >> 8;
    return I2C_writeData(lps->i2c, lps->devAddr, data, sizeof(uint8_t) + size, true);
}

/**
 * @brief Read the register value
 * @param lps is the LPS22DF data structure
 * @param addr is the register address
 * @param size is the data size (bytes)
 * @return LPS22DF_Errors values
 */
static int read(LPS22DF_Def *lps, uint8_t addr, uint8_t size) {
    if (!isInit(lps))
        return LPS22DF_NOT_INIT;
    if (isReading(lps))
        return LPS22DF_BUSY;

    lps->regAddr = addr;
    lps->dataSize = size;

    int result = I2C_writeData(lps->i2c, lps->devAddr, &lps->regAddr, sizeof(uint8_t), false);
    if (result == I2C_SUCCESS)
        lps->isReading = true;

    return result;
}

/**
 * @brief The pressure sensor initialization
 * @param lps is the LPS22DF data structure
 * @param i2c is the base I2C interface data structure
 * @param addr is the device address (on I2C bus)
 * @return LPS22DF_Errors values
 */
int LPS22DF_init(LPS22DF_Def *lps, I2CDef *i2c, uint8_t addr) {
    if (lps == NULL || i2c == NULL || addr == 0)
        return LPS22DF_WRONG_DATA;

    lps->i2c = i2c;
    lps->devAddr = addr;
    lps->pressure = 0;
    lps->refPressure = 0;
    lps->temp = -273.15f;
    lps->isInit = true;
    return LPS22DF_SUCCESS;
}

/**
 * @brief Read the value of the "WHOAMI" register to check the connection between MCU and the pressure sensor
 * @param lps is the LPS22DF data structure
 * @return LPS22DF_Errors values
 */
int LPS22DF_checkConnection(LPS22DF_Def *lps) {
    return read(lps, WHO_AM_I_ADDR, 1);
}

/**
 * @brief Check, that the pressure sensor is connected to MCU (I2C interface)
 * @param lps is the LPS22DF data structure
 * @return True - there is a connection between MCU and the sensor, otherwise - False
 */
bool LPS22DF_isConnected(const LPS22DF_Def *lps) {
    return lps->isConnected;
}

/**
 * @brief Setting the main control registers
 * @param lps is the LPS22DF data structure
 * @param num is the configuration register number (1 - 4)
 * @param value is the register value
 * @return LPS22DF_Errors values
 */
int LPS22DF_setting(LPS22DF_Def *lps, uint8_t num, uint8_t value) {
    if (num == 0 || num > 4)
        return LPS22DF_WRONG_DATA;

    int result = setting(lps, CTRL_REG1_ADDR + --num, value, sizeof(uint8_t));
    if (result == I2C_SUCCESS)
        lps->settings[num] = value;

    return result;
}

/**
 * @brief Setting the interface control register
 * @param lps is the LPS22DF data structure
 * @param value is the register value
 * @return LPS22DF_Errors values
 */
int LPS22DF_settingInterface(LPS22DF_Def *lps, uint8_t value) {
    return setting(lps, IF_CTRL_ADDR, value, sizeof(uint8_t));
}

/**
 * @brief Setting FIFO module (24-bit, 128 pressure values)
 * @param lps is the LPS22DF data structure
 * @param ctrl is the control register value
 * @param wtm is the watermark value (max 127)
 * @return LPS22DF_Errors values
 */
int LPS22DF_settingFIFO(LPS22DF_Def *lps, uint8_t ctrl, uint8_t wtm) {
    if (wtm > 127)
        return LPS22DF_WRONG_DATA;

    lps->useFIFO = true;
    return setting(lps, FIFO_CTRL_ADDR, ((uint16_t) wtm << 8) | ctrl, sizeof(uint16_t));
}

/**
 * @brief Interrupt source configuration (pressure value)
 * @param lps is the LPS22DF data structure
 * @param value is the interrupt configuration register value
 * @return LPS22DF_Errors values
 */
int LPS22DF_settingInterrupt(LPS22DF_Def *lps, uint8_t value) {
    return setting(lps, INTERRUPT_CFG_ADDR, value, sizeof(uint8_t));
}

/**
 * @brief Set an interrupt threshold level
 * @param lps is the LPS22DF data structure
 * @param level is the desired interrupt threshold (hPa, max value 1260)
 * @return LPS22DF_Errors values
 */
int LPS22DF_setPressureThreshold(LPS22DF_Def *lps, uint16_t level) {
    if (level > 1260)
        return LPS22DF_WRONG_DATA;

    // Datasheet, DS13316, Rev2, Jun 2023, page 31
    level <<= 4;

    return setting(lps, THS_P_L_ADDR, level, sizeof(uint16_t));
}

/**
 * @brief Set the pressure offset value to implement one-point calibration (OPC, to Non Volatile Memory)
 * @param lps is the LPS22DF data structure
 * @param offset is the required offset (hPa)
 * @return LPS22DF_Errors values
 */
int LPS22DF_setPressureOffset(LPS22DF_Def *lps, uint16_t offset) {
    if (offset > 1260)
        return LPS22DF_WRONG_DATA;

    return setting(lps, RPDS_L_ADDR, offset, sizeof(uint16_t));
}

/**
 * @brief Read the status, temperature and pressure registers values
 * @param lps is the LPS22DF data structure
 * @return LPS22DF_Errors values
 */
int LPS22DF_measure(LPS22DF_Def *lps) {
    uint8_t addr = STATUS_ADDR;
    uint8_t size = 6; // status + pressure + temperature

    if (lps->useFIFO) {
        addr = INT_SOURCE_ADDR;
        size = 4; // interrupt + FIFO status 1 and 2 + status
    }

    return read(lps, addr, size);
}

/**
 * @brief Read the reference pressure value
 * @param lps is the LPS22DF data structure
 * @return LPS22DF_Errors values
 */
int LPS22DF_readRefPressure(LPS22DF_Def *lps) {
    return read(lps, REF_P_L_ADDR, 2);
}

/**
 * @brief Get the last measured pressure value
 * @param lps is the LPS22DF data structure
 * @return pressure value (hPa)
 */
int LPS22DF_getPressure(const LPS22DF_Def *lps) {
    return lps->pressure;
}

/**
 * @brief Get a referense pressure value
 * @param lps is the LPS22DF data structure
 * @return pressure value (hPa)
 */
int LPS22DF_getRefPressure(const LPS22DF_Def *lps) {
    return lps->refPressure;
}

/**
 * @brief Get the last measured temperature value (C)
 * @param lps is the LPS22DF data structure
 * @return temperature value (degrees Celsius)
 */
float LPS22DF_getTemp_C(const LPS22DF_Def *lps) {
    return lps->temp;
}

/**
 * @brief Get the last measured temperature value (F)
 * @param lps is the LPS22DF data structure
 * @return temperature value (degrees Fahrenheit)
 */
float LPS22DF_getTemp_F(const LPS22DF_Def *lps) {
    float value = lps->temp;
    value = 32.0f + value * 9.0f / 5.0f;
    return value;
}

/**
 * @brief Check, that the pressure sensor is booting
 * @param lps is the LPS22DF data structure
 * @return True - boot phase is running, otherwise - False
 */
bool LPS22DF_isBooting(const LPS22DF_Def *lps) {
    return lps->interruptSource.fields.boot_on;
}

/**
 * @brief Check, that the pressure sensor has generated one or more interrupt events
 * @param lps is the LPS22DF data structure
 * @return True - is generated, otherwise - False
 */
bool LPS22DF_isInterrupt(const LPS22DF_Def *lps) {
    return lps->interruptSource.fields.ia;
}

/**
 * @brief Check, that the pressure sensor has detected a value that is lower than the low limit
 * @param lps is the LPS22DF data structure
 * @return True - is underpressure, otherwise - False
 */
bool LPS22DF_isLowPressure(const LPS22DF_Def *lps) {
    return lps->interruptSource.fields.pl;
}

/**
 * @brief Check, that the pressure sensor has detected a value that is higher than the high limit
 * @param lps is the LPS22DF data structure
 * @return True - is overpressure, otherwise - False
 */
bool LPS22DF_isHighPressure(const LPS22DF_Def *lps) {
    return lps->interruptSource.fields.ph;
}

/**
 * @brief Convert sensor register values to pressure
 * @param xlOut is the "...OUT_XL" and "...OUT_PRESS_XL" registers value
 * @param out is the "H" and "L" registers value
 * @return pressure value (hPa)
 */
static int16_t calculatePressure(uint8_t xlOut, int16_t out) {
    // Datasheet, DS13316, Rev2, Jun 2023, page 10
    int32_t pressure = ((int32_t) out << 8) | xlOut;
    return (int16_t)(pressure >> 12);
}

/**
 * @brief Convert sensor register values to degrees
 * @param value is the "TEMP_OUT_H" and "TEMP_OUT_L" registers value
 * @return temperature value (degrees Celsius)
 */
static float calculateTemp(int16_t value) {
    // Datasheet, DS13316, Rev2, Jun 2023, page 11
    return (float) value / 100.0f;
}

/**
 * @brief Update current state of the LPS22DF
 * @param lps is the LPS22DF data structure
 */
void LPS22DF_update(LPS22DF_Def *lps) {
    if (!isInit(lps))
        return;
    if (!isReading(lps))
        return;

    if (I2C_isReading(lps->i2c) || I2C_isWriting(lps->i2c))
        return;

    if (lps->addrSent) {
        lps->addrSent = false;
        lps->isReading = false;

        if (!I2C_isFailed(lps->i2c)) {
            const uint8_t *data = (const uint8_t *) I2C_getReceivedData(lps->i2c);
            switch (lps->regAddr) {
                case WHO_AM_I_ADDR:
                    lps->isConnected = (WHOAMI == *data);
                    break;
                case INT_SOURCE_ADDR:
                    lps->interruptSource.full = data[0];
                    lps->fifoStatus.dataLevel = data[1];
                    lps->fifoStatus.flags.full = data[2];
                    lps->status.full = data[3];

                    read(lps, TEMP_OUT_L_ADDR, sizeof(uint16_t));
                    break;
                case FIFO_STATUS1_ADDR:
                    break;
                case STATUS_ADDR:
                    lps->status.full = data[0];
                    if (lps->status.fields.p_da)
                        lps->pressure = calculatePressure(data[1], *(int16_t * )(data + 2));
                    if (lps->status.fields.t_da)
                        lps->temp = calculateTemp(*(int16_t * )(data + 4));
                    break;
                case TEMP_OUT_L_ADDR:
                    lps->temp = calculateTemp(*(int16_t *) data);

                    read(lps, FIFO_DATA_OUT_PRESS_XL_ADDR, 3);
                    break;
                case FIFO_DATA_OUT_PRESS_XL_ADDR:
                    lps->pressure = calculatePressure(*data, *(int16_t * )(data + 1));
                    break;
                case REF_P_L_ADDR:
                    lps->refPressure = *(int16_t *) data;
                    break;
            }
        }
    } else {
        int result = I2C_readData(lps->i2c, lps->devAddr, lps->dataSize);
        if (result == I2C_SUCCESS)
            lps->addrSent = true;
        else
            lps->isReading = false;
    }
}
