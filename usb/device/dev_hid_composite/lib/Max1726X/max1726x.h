#ifndef _MAX1726X_H_
#define _MAX1726X_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "usb_descriptors.h"

#include "hardware/i2c.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "lib/Settings/Board.h"

#define V_RANGE MILIVOLT
#define A_RANGE MILIAMPERE

#define MAX1726X_TABLE_SIZE 32
#define MAX1726X_TABLE_SIZE_IN_BYTES (2 * MAX1726X_TABLE_SIZE)

/* Model loading options */
#define MODEL_LOADING_OPTION1 1
#define MODEL_LOADING_OPTION2 2
#define MODEL_LOADING_OPTION3 3

/* Model lock/unlock */
#define MODEL_UNLOCK1 0X0059
#define MODEL_UNLOCK2 0X00C4
#define MODEL_LOCK1 0X0000
#define MODEL_LOCK2 0X0000

#define MODEL_SCALING 1

// Devuelve 1 si obtenemos un 1 en la posición pos del byte y.
#define CHECK_BIT(y, pos) ((0u == ((y) & (BIT(pos)))) ? 0u : 1u)
#define BIT(pos) (1 << (pos))

#define SET_BIT(x, pos) ((x) |= (BIT(pos)))
#define UNSET_BIT(x, pos) ((x) &= ~(BIT(pos)))

struct max1726x_platform_data
{
    uint16_t designcap;
    uint16_t ichgterm;
    uint16_t vempty;
    int vcharge;

    uint16_t learncfg;
    uint16_t relaxcfg;
    uint16_t config;
    uint16_t config2;
    uint16_t fullsocthr;
    uint16_t tgain;
    uint16_t toff;
    uint16_t curve;
    uint16_t rcomp0;
    uint16_t tempco;
    uint16_t qrtable00;
    uint16_t qrtable10;
    uint16_t qrtable20;
    uint16_t qrtable30;
    uint16_t cvhalftime;
    uint16_t cvmixcap;

    uint16_t dpacc;
    uint16_t modelcfg;

    int model_option;

    /*
     * rsense in miliOhms.
     * default 10 (if rsense = 0) as it is the recommended value by
     * the datasheet although it can be changed by board designers.
     */
    unsigned int rsense;
    int volt_min; /* in mV */
    int volt_max; /* in mV */
    int temp_min; /* in DegreC */
    int temp_max; /* in DegreeC */
    int soc_max;  /* in percent */
    int soc_min;  /* in percent */
    int curr_max; /* in mA */
    int curr_min; /* in mA */
};

enum max1726x_register
{
    MAX1726X_STATUS_REG = 0x00,
    MAX1726X_VALRTTH_REG = 0x01,
    MAX1726X_TALRTTH_REG = 0x02,
    MAX1726X_SALRTTH_REG = 0x03,
    MAX1726X_REPCAP_REG = 0x05,
    MAX1726X_REPSOC_REG = 0x06,
    MAX1726X_TEMP_REG = 0x08,
    MAX1726X_VCELL_REG = 0x09,
    MAX1726X_CURRENT_REG = 0x0A,
    MAX1726X_AVGCURRENT_REG = 0x0B,
    MAX1726X_REMCAP_REG = 0x0F,

    MAX1726X_FULLCAPREP_REG = 0x10,
    MAX1726X_TTE_REG = 0X11,
    MAX1726X_QRTABLE00_REG = 0x12,
    MAX1726X_FULLSOCTHR_REG = 0x13,
    MAX1726X_CYCLES_REG = 0x17,
    MAX1726X_DESIGNCAP_REG = 0x18,
    MAX1726X_AVGVCELL_REG = 0x19,
    MAX1726X_MAXMINVOLT_REG = 0x1B,
    MAX1726X_CONFIG_REG = 0x1D,
    MAX1726X_ICHGTERM_REG = 0x1E,

    MAX1726X_TTF_REG = 0X20,
    MAX1726X_VERSION_REG = 0x21,
    MAX1726X_QRTABLE10_REG = 0x22,
    MAX1726X_FULLCAPNOM_REG = 0x23,
    MAX1726X_LEARNCFG_REG = 0x28,
    MAX1726X_RELAXCFG_REG = 0x2A,
    MAX1726X_TGAIN_REG = 0x2C,
    MAX1726X_TOFF_REG = 0x2D,

    MAX1726X_QRTABLE20_REG = 0x32,
    MAX1726X_RCOMP0_REG = 0x38,
    MAX1726X_TEMPCO_REG = 0x39,
    MAX1726X_VEMPTY_REG = 0x3A,
    MAX1726X_FSTAT_REG = 0x3D,

    MAX1726X_QRTABLE30_REG = 0x42,
    MAX1726X_DQACC_REG = 0x45,
    MAX1726X_DPACC_REG = 0x46,
    MAX1726X_VFSOC0_REG = 0x48,
    MAX1726X_QH0_REG = 0x4C,
    MAX1726X_QH_REG = 0x4D,

    MAX1726X_VFSOC0_QH0_LOCK_REG = 0x60,
    MAX1726X_LOCK1_REG = 0x62,
    MAX1726X_LOCK2_REG = 0x63,

    MAX1726X_MODELDATA_START_REG = 0x80,

    MAX1726X_POWER_REG = 0xB1,
    MAX1726X_POWER_AVR_REG = 0xB3,
    MAX1726X_IALRTTH_REG = 0xB4,
    MAX1726X_CVMIXCAP_REG = 0xB6,
    MAX1726X_CVHALFIME_REG = 0xB7,
    MAX1726X_CURVE_REG = 0xB9,
    MAX1726X_HIBCFG_REG = 0xBA,
    MAX1726X_CONFIG2_REG = 0xBB,

    MAX1726X_MODELCFG_REG = 0xDB,

    MAX1726X_OCV_REG = 0xFB,
    MAX1726X_VFSOC_REG = 0xFF,
};

enum Max_Status_register
{

    // En las variables de tres letras (VMX,VMN.... etc) significa threshold cuando es el mínimo, cuando es acabado en X, significa threshold cuando superamos el valor máximo.

    POWER_ON_RESET = 1,
    IMN_THRESHOLD = 2,
    BATTERY_STATUS = 3,
    IMX_THRESHOLD = 6,
    STATE_OF_CHARGE = 7,
    VMN_THRESHOLD = 8,
    TMN_THRESHOLD = 9,
    SMN_THRESHOLD = 10,
    BATTERY_INSERTION = 11,
    VMX_THRESHOLD = 12,
    TMX_THRESHOLD = 13,
    SMX_THRESHOLD = 14,
    BATTERY_REMOVAL = 15,

};

typedef struct
{

    // Batería, lo que queda por gastar, el SOC y lo que estima el sistema que tenemos.
    float RemainingCapacity;
    float SoC;
    uint16_t FullRepCap;
    uint16_t Cycles;

    // Tiempos de utilización.
    float EstimateTimeToEmpty;
    float EstimateTimeToFull;

    // Medidas analógicas
    float Voltage;
    float VoltageAvr;

    float Current;
    float CurrentAvr;

    float Temp;

    // Medidas sin uso de momento
#if 0
    uint16_t MaxMinVol;
    uint16_t MaxMinCurr;

    uint16_t MaxMinTemp;

    float PowerPeak;
    float Power;
    float PowerAvr;

#endif

} FuelGaugeReads_t;

enum Volt_Range
{
    VOLT = 0,
    MILIVOLT = 1,
    MICROVOLT = 2,
};

enum Ampere_Range
{
    AMPERES = 0,
    MILIAMPERE = 1,
    MICROAMPERE = 2,
};

class FuelGauge
{
public:
    FuelGauge();

    void FuelGaugeTask();

    void PrintRegister(uint8_t reg);
    int SetBatteryProperties(uint16_t DesignCap, uint16_t IchgTerm, uint16_t FullSOCThr, uint16_t VEmpty, float ChargeVoltage);
    int InitializeFuelGauge();
    void PrintValues();
    float get_max1726x_SOC();
    uint16_t get_max1726x_RemainCapacity();

    float get_max1726x_BattVoltage(Volt_Range Range);
    float get_max1726x_AvrgVoltage(Volt_Range Range);

    float get_max1726x_BattCurrent(Ampere_Range Range);
    float get_max1726x_AvgBattCurrent(Ampere_Range Range);

    float get_max1726x_Power();
    float get_max1726x_PowerAvr();
    float get_max1726x_PowerPeak();

    float get_max1726x_BattTemp();

    uint16_t get_max1726x_BattCycles();
    uint16_t get_max1726x_DesignCap();
    uint16_t get_max1726x_FullCapRep();

    float get_max1726x_TTE();
    float get_max1726x_TTF();

    FuelGaugeReads_t *PointerToFuelGaugeValues()
    {
        return &FuelValues;
    }
    FuelGaugeReads_t FuelValues;

private:
    void ReadWord(uint8_t addr, uint8_t reg, uint16_t *final_value);
    void WriteWord(uint8_t addr, uint8_t reg, uint16_t data);
    void VerifyAndWriteWord(uint8_t addr, uint8_t reg, uint16_t data);
    void writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);

    void max1726x_set_alert_thresholds();
    // Declaracion variables

    float max1726x_raw_current_to_uamps(uint16_t Value);
    float max1726x_raw_current_to_mamps(uint16_t Value);
    float max1726x_raw_voltage_to_milivolts(uint16_t Value);
    float max1726x_raw_voltage_to_uvolts(uint16_t Value);
    float max1726x_raw_voltage_to_volts(uint16_t Value);
    float max1726x_raw_time_to_sec(uint16_t Value);
    float max1726x_raw_time_to_min(uint16_t Value);
    float max1726x_raw_time_to_hour(uint16_t Value);
    int max1726x_raw_to_percent(uint16_t Value);
    float max1726x_raw_to_Celsius(uint16_t Value);
    float max1726x_raw_power_to_mwatts(uint16_t Value);

    max1726x_platform_data MaxDataStruct;
};

#endif /* MAX1726X */
