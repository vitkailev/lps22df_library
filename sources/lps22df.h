#ifndef LPS22DF_H
#define LPS22DF_H

#ifdef __cplusplus
extern "C" {
#endif

enum LPS22DF_Erros {
    LPS22DF_SUCCESS = 0,

    LPS22DF_NOT_INIT = -I2C_NUMBER_ERRORS - 1,
    LPS22DF_WRONG_DATA = -I2C_NUMBER_ERRORS - 2,
    LPS22DF_BUSY = -I2C_NUMBER_ERRORS - 3,
};

enum LPS22DF_Constants {
    LPS22DF_ODR_ONE_SHOT = 0,
    LPS22DF_ODR_1Hz,
    LPS22DF_ODR_4Hz,
    LPS22DF_ODR_10Hz,
    LPS22DF_ODR_25Hz,
    LPS22DF_ODR_50Hz,
    LPS22DF_ODR_75Hz,
    LPS22DF_ODR_100Hz,
    LPS22DF_ODR_200Hz,

    LPS22DF_AVG_4 = 0,
    LPS22DF_AVG_8,
    LPS22DF_AVG_16,
    LPS22DF_AVG_32,
    LPS22DF_AVG_64,
    LPS22DF_AVG_128,
    LPS22DF_AVG_512 = 7,

    LPS22DF_FIFOMODE_BYPASS = 0,
    LPS22DF_FIFOMODE_FIFO_OR_BYPASS_FIFO,
    LPS22DF_FIFOMODE_CONT_OR_BYPASS_CONT,
    LPS22DF_FIFOMODE_CONT_FIFO,
};

typedef union {
    struct LPS22DF_InterruptConfigurationRegister {
        unsigned phe: 1; // Enable interrupt generation on pressure high event. 1 - enable
        unsigned ple: 1; // Enable interrupt generation on pressure low event. 1 - enable
        unsigned lir: 1; // Latch interrupt request to the INT_SOURCE register. 1 - interrupt request latched
        unsigned: 1;
        unsigned reset_az: 1; // Reset AUTOZERO function. 1 - reset
        unsigned autozero: 1; // Enable AUTOZERO function. 1 - enable
        unsigned reset_arp: 1; // Reset AUTOREFP function. 1 - reset
        unsigned autorefp: 1; // Enable AUTOREFP function. 1 - enable
    } fields;

    uint8_t full;
} LPS22DF_InterruptConfiguration_Def;

typedef union {
    struct LPS22DF_InterfaceControlRegister {
        unsigned : 1;
        unsigned cs_pu_dis: 1; // Disable pull-up on the CS pin. 1 - disconnected
        unsigned int_pd_dis: 1; // Disable pull-down on the INT pin. 1 - disconnected
        unsigned sdo_pu_en: 1; // Enable pull-up on the SDO pin. 1 - with pull-up
        unsigned sda_pu_en: 1; // Enable pull-up on the SDA pin. 1 - with pull-up
        unsigned sim: 1; // SPI serial interface mode selection. 1 - 3-wire interface
        unsigned i2c_i3c_dis: 1; // Disable I2C and I3C digital interfaces. 1 - disable
        unsigned int_en_i3c: 1; // Enable INT pin with MIPI I3C. 1 - enable
    } fields;

    uint8_t full;
} LPS22DF_InterfaceControl_Def;

typedef union {
    struct LPS22DF_Control1Register {
        unsigned avg: 3; // Average selection
        unsigned odr: 4; // Output data rate selection
        unsigned : 1;
    } fields;

    uint8_t full;
} LPS22DF_Control1_Def;

typedef union {
    struct LPS22DF_Control2Register {
        unsigned oneshot: 1; // Enable one-shot mode. 1 - a new dataset is acquired
        unsigned : 1;
        unsigned swreset: 1; // Software reset. 1 - software reset
        unsigned bdu: 1; // Block data update. 1 - output registers not updated until MSB and LSB have been read
        unsigned en_lpfp: 1; // Enable low-pass filter on pressure data. 1 - enable
        unsigned lfpf_cfg: 1; // Low-pass filter configuration. 1 - ODR/9; 0 - ODR/4
        unsigned : 1;
        unsigned boot: 1; // Reboot memory content. 1 - reboot memory content
    } fields;

    uint8_t full;
} LPS22DF_Control2_Def;

typedef union {
    struct LPS22DF_Control3Register {
        unsigned if_add_inc: 1; // Register address automatically incremented during a multiple byte access with a serial interface. 1 - enable
        unsigned pp_od: 1; // Push-pull/open-drain selection on interrupt pin. 1 - open-drain
        unsigned : 1;
        unsigned int_h_l: 1; // Select interrupt active-high, active-low. 1 - active-low
        unsigned : 4;
    } fields;

    uint8_t full;
} LPS22DF_Control3_Def;

typedef union {
    struct LPS22DF_Control4Register {
        unsigned int_f_ovr: 1; // FIFO overrun status on INT pin. 1 - at least one sample in the FIFO has been overwritten
        unsigned int_f_wtm: 1; // FIFO threshold (watermark) status on INT pin. 1 - FIFO is equal to or higher than WTM level
        unsigned int_f_full: 1; // FIFO full flag on INT pin. 1 - FIFO is full of 128 unread samples
        unsigned : 1;
        unsigned int_en: 1; // Interrupt signal on INT pin. 1 - enable
        unsigned drdy: 1; // Date-ready signal on INT pin. 1 - enable
        unsigned drdy_pls: 1; // Data-ready pulsed on INT pin. 1 - enable data-ready pulsed on INT pin, pulse width around 5 us
        unsigned : 1;
    } fields;

    uint8_t full;
} LPS22DF_Control4_Def;

typedef union {
    struct LPS22DF_FIFOControlRegister {
        unsigned f_mode: 2; // Selects triggered FIFO modes
        unsigned trig_modes: 1; // Enables triggered FIFO modes
        unsigned stop_on_wtm: 1; // Stop-on-FIFO watermark.Enables FIFO watermark level use. 1 - enable
        unsigned : 4;
    } fields;

    uint8_t full;
} LPS22DF_FIFOControl_Def;

typedef union {
    struct LPS22DF_I3CInterfaceControlRegister {
        unsigned i3c_bus_avb_sel: 2; // These bits are used to select the bus available time when I3C IBI is used
        unsigned : 3;
        unsigned asf_on: 1; // Enables antispike filters. 1 - antispike filters on SCL and SDA lines are always enabled
        unsigned reserved: 2; // !!!VALUE == 2, datasheet!!!
    } fields;

    uint8_t full;
} LPS22DF_I3CInterfaceControl_Def;

typedef union {
    struct LPS22DF_InterruptSourceRegister {
        unsigned ph: 1; // Differential pressure high. 1 - high differential pressure event has occurred
        unsigned pl: 1; // Differential pressure low. 1 - low differential pressure event has occurred
        unsigned ia: 1; // Interrupt active. 1 - one or more interrupt events have been generated
        unsigned : 4;
        unsigned boot_on: 1; // Indication that the boot (reboot) phase is running
    } fields;

    uint8_t full;
} LPS22DF_InterruptSource_Def;

typedef union {
    struct LPS22DF_FIFOStatus2Register {
        unsigned: 5;
        unsigned fifo_full_ia: 1; // FIFO full status. 1 - FIFO is completely filled, no samples overwritten
        unsigned fifo_ovr_ia: 1; // FIFO overrun status. 1 - FIFO is full and at least one sample in the FIFO has been overwritten
        unsigned fifo_wtm_ia: 1; // FIFO threshold (watermark) status. 1 - FIFO filling is equal or higher than threshold level
    } fields;

    uint8_t full;
} LPS22DF_FIFOStatus2_Def;

typedef struct {
    uint8_t dataLevel; // FIFO Status 1, max 128
    LPS22DF_FIFOStatus2_Def flags;
} LPS22DF_FIFOStatus_Def;

typedef union {
    struct LPS22DF_StatusRegister {
        unsigned p_da: 1; // Pressure data available. 1 - new pressure data is generated
        unsigned t_da: 1; // Temperature data available. 1 - new temperature data is generated
        unsigned : 2;
        unsigned p_or: 1; // Pressure data overrun. 1 - new data for pressure has overwritten the previous data
        unsigned t_or: 1; // Temperature data overrun. 1 - new data for temperature has overwritten the previous data
        unsigned : 2;
    } fields;

    uint8_t full;
} LPS22DF_Status_Def;

typedef struct {
    bool isInit;
    bool isConnected;

    bool isReading;
    bool addrSent;
    uint8_t regAddr;

    bool useFIFO;
    uint8_t dataSize; // bytes
    uint8_t settings[4]; // control registers (1 - 4)
    LPS22DF_InterruptSource_Def interruptSource;
    LPS22DF_FIFOStatus_Def fifoStatus;
    LPS22DF_Status_Def status;
    int16_t pressure; // hPa
    int16_t refPressure;
    float temp; // C

    uint8_t devAddr;
    void *i2c;
} LPS22DF_Def;

int LPS22DF_init(LPS22DF_Def *lps, void *i2c, uint8_t addr);

int LPS22DF_checkConnection(LPS22DF_Def *lps);

bool LPS22DF_isConnected(const LPS22DF_Def *lps);

int LPS22DF_setting(LPS22DF_Def *lps, uint8_t num, uint8_t value);

int LPS22DF_settingInterface(LPS22DF_Def *lps, uint8_t value);

int LPS22DF_settingFIFO(LPS22DF_Def *lps, uint8_t ctrl, uint8_t wtm);

int LPS22DF_settingInterrupt(LPS22DF_Def *lps, uint8_t value);

int LPS22DF_setPressureThreshold(LPS22DF_Def *lps, uint16_t level);

int LPS22DF_setPressureOffset(LPS22DF_Def *lps, uint16_t offset);

int LPS22DF_measure(LPS22DF_Def *lps);

int LPS22DF_readRefPressure(LPS22DF_Def *lps);

int LPS22DF_getPressure(const LPS22DF_Def *lps);

int LPS22DF_getRefPressure(const LPS22DF_Def *lps);

float LPS22DF_getTemp_C(const LPS22DF_Def *lps);

float LPS22DF_getTemp_F(const LPS22DF_Def *lps);

bool LPS22DF_isBooting(const LPS22DF_Def *lps);

bool LPS22DF_isInterrupt(const LPS22DF_Def *lps);

bool LPS22DF_isLowPressure(const LPS22DF_Def *lps);

bool LPS22DF_isHighPressure(const LPS22DF_Def *lps);

void LPS22DF_update(LPS22DF_Def *lps);

#ifdef __cplusplus
}
#endif

#endif // LPS22DF_H
