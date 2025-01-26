# LPS22DF I2C library

This is a library that covers the main capabilities of the LPS22DF pressure sensor via the I2C interface  
[Official STMicroelectronics data source](https://www.st.com/en/mems-and-sensors/lps22df.html)

## Sensor description

- Absolute pressure range: 260 to 1260 hPa;
- Pressure accuracy: 0.2 hPa
- Low pressure sensor noise: 0.34 hPa
- Embedded temperature compensation
- Embedded FIFO
- 24-bit pressure data output
- 16-bit temperature data output;
- ODR from 1 Hz to 200 Hz
- SPI, I2C and MIPI I3C interface;
- Supply voltage: 1.7 to 3.6 V;
- Current consumption: 1.7 uA (one-shot mode);
- Factory calibrated;

## Library features

- Check the connection between MCU and the sensor (read "WHOAMI" register value);
- Setting sensor (except I3C interface);
- Setting the pressure threshold level (for INT pin);
- Reading the values of status, temperature and pressure registers;

## Algorithm

1. Initialization: LPS22DF_init;
2. Check connection (optional): LPS22DF_checkConnection;
3. Setting: LPS22DF_setting (and LPS22DF_settingFIFO, if you plan to use FIFO);
4. Read values: LPS22DF_measure

- if you configured the control register 1 (AVR and ODR) only, you will read:
-
    - Status register
-
    - Pressure XL, L and H registers
-
    - Temperature L and H registers
- if you also configured FIFO (control register and watermark), you will read:
-
    - Interrupt register
-
    - FIFO status 1 and 2 registers
-
    - Status register
-
    - Temperature L and H registers
-
    - FIFO data output XL, L and H registers
