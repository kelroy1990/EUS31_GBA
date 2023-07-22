/*
Este dispositivo tiene una máquina de estados como sigue.

El dispositivo sufre un reset cada vez que se queda sin batería.

Hay que comprobar si se ha quedado sin batería, lo que viene siendo un reset.
Si esto ocurre se debe cargar la configuración de la batería, sino.... se continua con sus lecturas.

Check fuel gauge reset  ->No Continue
                        ->Yes, load ini cnofiguration.






REMEMBER en el max el tamaño es siempre de 2 bytes, y el data[0] es el menos significativo
siendo el data[1] el más significativo.




*/

#include "max1726x.h"

// Parte Defines

#define FuelGaugeAdress 0x36
#define RSENSE 0.010 // 10mΩ

#define STATUS_REG 0x00
#define FSTAT_REG 0x3D
#define HibCFG_REG 0xBA
#define XTABLE0_REG 0x90
#define SOFT_WAKE_UP_REG 0x60
#define DESIGN_CAP_REG 0x18
#define Ichg_TERM_REG 0x1E
#define VEmpty_REG 0x3A
#define MODEL_CFG_REG 0xDB
#define REP_CAP_REG 0x05 // RepCap or reported remaining capacity in mAh. This register is protected from making sudden jumps during load changes
#define REP_SOC_REG 0x06 // RepSOC is the reported state-of-charge percentage output for use by the application GUI
#define TTE_REG 0x11
#define FULL_CAP_REG 0x10
#define VCELL_REG 0x09 // registro que tiene la tension que se mide en la bateria
#define AVGVCELL_REG 0x19
#define CURRENT_REG 0x0A
#define AVGCURRENT_REG 0x0B

/* CONFIG register bits */
#define MAX1726X_CONFIG_ALRT_EN (1 << 2)
#define MAX1726X_CONFIG2_LDMDL (1 << 5)

/* STATUS register bits */
#define MAX1726X_STATUS_BST (1 << 3)
#define MAX1726X_STATUS_POR (1 << 1)

/* MODELCFG register bits */
#define MAX1726X_MODELCFG_REFRESH (1 << 15)

/* TALRTTH register bits */
#define MIN_TEMP_ALERT 0
#define MAX_TEMP_ALERT 8

/* FSTAT register bits */
#define MAX1726X_FSTAT_DNR 0x0001

/* STATUS interrupt status bits */
#define MAX1726X_STATUS_ALRT_CLR_MASK (0x88BB)
#define MAX1726X_STATUS_SOC_MAX_ALRT (1 << 14)
#define MAX1726X_STATUS_TEMP_MAX_ALRT (1 << 13)
#define MAX1726X_STATUS_VOLT_MAX_ALRT (1 << 12)
#define MAX1726X_STATUS_SOC_MIN_ALRT (1 << 10)
#define MAX1726X_STATUS_TEMP_MIN_ALRT (1 << 9)
#define MAX1726X_STATUS_VOLT_MIN_ALRT (1 << 8)
#define MAX1726X_STATUS_CURR_MAX_ALRT (1 << 6)
#define MAX1726X_STATUS_CURR_MIN_ALRT (1 << 2)

#define MAX1726X_VMAX_TOLERANCE 50 /* 50 mV */

// Inicio conversion a C++

FuelGauge::FuelGauge()
{
    // Constructor vacio
    // Default battery settings
}

int FuelGauge::SetBatteryProperties(uint16_t DesignCap, uint16_t IchgTerm, uint16_t FullSOCThr, uint16_t VEmpty, float ChargeVoltage)
{
    MaxDataStruct.rsense = RSENSE;
    MaxDataStruct.designcap = DesignCap * 2;

    // IchTerm tiene el valor de corriente deseado, pero el campo se expresa en forma de voltaje, por lo que tenemos que convertirlo

    MaxDataStruct.ichgterm = IchgTerm;

    MaxDataStruct.fullsocthr = FullSOCThr;

    MaxDataStruct.vempty = VEmpty;

    MaxDataStruct.vcharge = ChargeVoltage;

    MaxDataStruct.model_option = 1;

    return 0;
}

float FuelGauge::get_max1726x_SOC()
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, MAX1726X_REPSOC_REG, &AuxCMD_);
    return ((float)AuxCMD_) / 256;
}

uint16_t FuelGauge::get_max1726x_RemainCapacity()
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, REP_CAP_REG, &AuxCMD_);
    return AuxCMD_ * 0.5;
}

float FuelGauge::get_max1726x_BattVoltage(Volt_Range Range)
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, VCELL_REG, &AuxCMD_);
    switch (Range)
    {
    case VOLT:
        return max1726x_raw_voltage_to_volts(AuxCMD_);
        break;

    case MILIVOLT:
        return max1726x_raw_voltage_to_milivolts(AuxCMD_);
        break;

    case MICROVOLT:
        return max1726x_raw_voltage_to_uvolts(AuxCMD_);

    default:
        return 0;
        break;
    }
}

float FuelGauge::get_max1726x_AvrgVoltage(Volt_Range Range)
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, AVGVCELL_REG, &AuxCMD_);
    switch (Range)
    {
    case VOLT:
        return max1726x_raw_voltage_to_volts(AuxCMD_);
        break;

    case MILIVOLT:
        return max1726x_raw_voltage_to_milivolts(AuxCMD_);
        break;

    case MICROVOLT:
        return max1726x_raw_voltage_to_uvolts(AuxCMD_);

    default:
        return 0;
        break;
    }
}
float FuelGauge::get_max1726x_BattCurrent(Ampere_Range Range)
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, CURRENT_REG, &AuxCMD_);
    switch (Range)
    {
    case AMPERES:
        return max1726x_raw_current_to_mamps(AuxCMD_) / 1000;
        break;

    case MILIAMPERE:
        return max1726x_raw_current_to_mamps(AuxCMD_);
        break;

    case MICROAMPERE:
        return max1726x_raw_current_to_uamps(AuxCMD_);

    default:
        return 0;
        break;
    }
}

float FuelGauge::get_max1726x_AvgBattCurrent(Ampere_Range Range)
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, AVGCURRENT_REG, &AuxCMD_);
    switch (Range)
    {
    case AMPERES:
        return max1726x_raw_current_to_mamps(AuxCMD_) / 1000;
        break;

    case MILIAMPERE:
        return max1726x_raw_current_to_mamps(AuxCMD_);
        break;

    case MICROAMPERE:
        return max1726x_raw_current_to_uamps(AuxCMD_);

    default:
        return 0;
        break;
    }
}

float FuelGauge::get_max1726x_BattTemp()
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, MAX1726X_TEMP_REG, &AuxCMD_);

    return max1726x_raw_to_Celsius(AuxCMD_);
}

uint16_t FuelGauge::get_max1726x_BattCycles()
{
    uint16_t AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, MAX1726X_CYCLES_REG, &AuxCMD_);
    return AuxCMD_;
}

uint16_t FuelGauge::get_max1726x_DesignCap()
{
    uint16_t AuxCMD_ = 0x00;
    ReadWord(FuelGaugeAdress, DESIGN_CAP_REG, &AuxCMD_);
    return AuxCMD_ * 0.5;
}

uint16_t FuelGauge::get_max1726x_FullCapRep()
{
    uint16_t AuxCMD_ = 0x00;
    ReadWord(FuelGaugeAdress, FULL_CAP_REG, &AuxCMD_);
    return AuxCMD_ * 0.5;
}

float FuelGauge::get_max1726x_TTE()
{
    uint16_t AuxCMD_ = 0x00;
    ReadWord(FuelGaugeAdress, MAX1726X_TTE_REG, &AuxCMD_);
    return max1726x_raw_time_to_min(AuxCMD_);
}

float FuelGauge::get_max1726x_TTF()
{
    uint16_t AuxCMD_ = 0x00;
    ReadWord(FuelGaugeAdress, MAX1726X_TTF_REG, &AuxCMD_);
    return max1726x_raw_time_to_min(AuxCMD_);
}

float FuelGauge::get_max1726x_Power()
{

    uint16_t AuxCMD_ = 0x00;
    ReadWord(FuelGaugeAdress, MAX1726X_POWER_REG, &AuxCMD_);
    return max1726x_raw_power_to_mwatts(AuxCMD_);
}

float FuelGauge::get_max1726x_PowerAvr()
{

    uint16_t AuxCMD_ = 0x00;
    ReadWord(FuelGaugeAdress, MAX1726X_POWER_AVR_REG, &AuxCMD_);
    return max1726x_raw_power_to_mwatts(AuxCMD_);
}

void FuelGauge::PrintValues()
{

    uint16_t AuxCMD_ = 0x000;
    printf("############################### \r\n");

    ReadWord(FuelGaugeAdress, STATUS_REG, &AuxCMD_);
    printf("Status Register 0x%04x mAh\r\n", AuxCMD_);
    AuxCMD_ = 0x000;

    ReadWord(FuelGaugeAdress, MAX1726X_CONFIG_REG, &AuxCMD_);
    printf("Config1 Register 0x%04x mAh\r\n", AuxCMD_);
    AuxCMD_ = 0x000;

    ReadWord(FuelGaugeAdress, MAX1726X_CONFIG2_REG, &AuxCMD_);
    printf("Config2 Register 0x%04x mAh\r\n", AuxCMD_);
    AuxCMD_ = 0x000;

    ReadWord(FuelGaugeAdress, DESIGN_CAP_REG, &AuxCMD_);

    printf("Battery capacity design %f mAh\r\n", AuxCMD_ * 0.5);
    AuxCMD_ = 0x000;

    ReadWord(FuelGaugeAdress, FULL_CAP_REG, &AuxCMD_);

    printf("Battery capacity on full %f mAh\r\n", AuxCMD_ * 0.5);
    AuxCMD_ = 0x000;

    ReadWord(FuelGaugeAdress, REP_CAP_REG, &AuxCMD_);
    printf("Battery remaining capacity %f mAh\r\n", AuxCMD_ * 0.5);

    AuxCMD_ = 0x000;

    ReadWord(FuelGaugeAdress, MAX1726X_REPSOC_REG, &AuxCMD_);
    printf("Battery SoC %d% y en hex entero 0x%04x\r\n", AuxCMD_ >> 8, AuxCMD_);

    AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, VCELL_REG, &AuxCMD_);
    printf("Battery Voltage %f mv\r\n", max1726x_raw_voltage_to_milivolts(AuxCMD_));

    AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, CURRENT_REG, &AuxCMD_);
    printf("Battery current %f mA\r\n", max1726x_raw_current_to_mamps(AuxCMD_));

    AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, MAX1726X_TEMP_REG, &AuxCMD_);
    printf("Onboard Battery temperature %f ºC\r\n", max1726x_raw_to_Celsius(AuxCMD_));

    AuxCMD_ = 0x000;
    ReadWord(FuelGaugeAdress, MAX1726X_CYCLES_REG, &AuxCMD_);
    printf("Battery have %d cycles\r\n", AuxCMD_);
}

void FuelGauge::PrintRegister(uint8_t reg)
{

    uint16_t Data = 0x00;

    ReadWord(FuelGaugeAdress, reg, &Data);

    printf("Leemos lo siguiente : %#x \r\n", Data);
}
int FuelGauge::InitializeFuelGauge()
{
    // Primero miramos si tenemos el POR a uno, este bit indica
    // un reset de la bateria, tanto software como hardware. Si se detecta debe ser bajado
    // por el software para poder revisar el siguiente reset.

    //uint8_t retry_times = 5;

    // Bucle de configuracion de bateria

    // Si tenemos un 1, en el bit D1 entonces significa que se ha reseteado y hay que configurarla
    // Para poder comprobar que tenemos el dato listo debemos ver el D0 de la ADDR 3D. Este bit
    // Lo coloca el Fuel cuando termina de actualizar los registros todos, tarda como 710ms si se saca la bateria.

    // ##Revisar con bit si llegamos a este estado con la placa jugando.... que lagearia.

    uint16_t FSTAT_DNR = 0x0000;

    do
    {
        sleep_ms(10);
        ReadWord(FuelGaugeAdress, FSTAT_REG, &FSTAT_DNR);
        //retry_times--;
        //printf("Estado actual de FSTAT_DNR 0b'" BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN "\r\n",BYTE_TO_BINARY(FSTAT_DNR>>8),BYTE_TO_BINARY(FSTAT_DNR));

    } while ((FSTAT_DNR & MAX1726X_FSTAT_DNR) == 1);// && (retry_times > 0));

    // 

    /* Una vez en este punto debemos hacer la configuracion del dispositivo

    */

    uint16_t HibCFG = 0x0000;
    ReadWord(FuelGaugeAdress, HibCFG_REG, &HibCFG);
    // Para salir del modo "hibernate" se deben mandar 3 comandos.

    uint16_t AuxCMD = 0x0090;

    WriteWord(FuelGaugeAdress, SOFT_WAKE_UP_REG, AuxCMD);
    AuxCMD = 0x0000;
    WriteWord(FuelGaugeAdress, HibCFG_REG, AuxCMD);
    WriteWord(FuelGaugeAdress, SOFT_WAKE_UP_REG, AuxCMD);

    uint16_t ModelCFG_Refresh = 0x000;

    // Ahora estamos fuera del modo sleep e iniciamos configuracion.
    // 2.1 OPTION 1 EZ Config(No INI file is needed) :

    WriteWord(FuelGaugeAdress, DESIGN_CAP_REG, MaxDataStruct.designcap);
    WriteWord(FuelGaugeAdress, Ichg_TERM_REG, MaxDataStruct.ichgterm);
    WriteWord(FuelGaugeAdress, VEmpty_REG, MaxDataStruct.vempty);

    if (MaxDataStruct.vcharge > 4275)
    {
        AuxCMD = 0x8400;
        WriteWord(FuelGaugeAdress, MODEL_CFG_REG, AuxCMD);
    }
    else
    {
        AuxCMD = 0x8000;
        WriteWord(FuelGaugeAdress, MODEL_CFG_REG, AuxCMD);
    }

    // Poll ModelCFG.Refresh(highest bit),
    // proceed to Step 3 when ModelCFG.Refresh=0.
    ModelCFG_Refresh = 0x000;
 
    do
    {
        sleep_ms(10);
        ReadWord(FuelGaugeAdress, MODEL_CFG_REG, &ModelCFG_Refresh);

    } while ((ModelCFG_Refresh & 0x8000) == 0 );// && (retry_times > 0));
    // printf("Estado actual de ModelCFG_Refresh 0b'" BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN "\r\n",BYTE_TO_BINARY(ModelCFG_Refresh>>8),BYTE_TO_BINARY(ModelCFG_Refresh));

    WriteWord(FuelGaugeAdress, HibCFG_REG, HibCFG);

    /* Optional step - alert threshold initialization */
    // max1726x_set_alert_thresholds();

    // Finalizamos configuracion
    uint16_t Status = 0x0000;
    ReadWord(FuelGaugeAdress, MAX1726X_STATUS_REG, &Status);

    VerifyAndWriteWord(FuelGaugeAdress, MAX1726X_STATUS_REG, (Status & ~MAX1726X_STATUS_POR));

    return 0;
}

void FuelGauge::max1726x_set_alert_thresholds()
{
    uint16_t value;
    /* Set VAlrtTh */
    value = (MaxDataStruct.volt_min / 20) & 0xff;
    value |= ((MaxDataStruct.volt_max / 20) << 8);
    WriteWord(FuelGaugeAdress, MAX1726X_VALRTTH_REG, value);

    /* Set TAlrtTh */
    value = (MaxDataStruct.temp_min) & 0xff;
    value |= ((MaxDataStruct.temp_max & 0xff) << 8);
    WriteWord(FuelGaugeAdress, MAX1726X_TALRTTH_REG, value);

    /* Set SAlrtTh */
    value = (MaxDataStruct.soc_min) & 0xff;
    value |= ((MaxDataStruct.soc_max) << 8);
    WriteWord(FuelGaugeAdress, MAX1726X_SALRTTH_REG, value);

    /* Set IAlrtTh */
    value = (MaxDataStruct.curr_min * MaxDataStruct.rsense / 400) & 0xff;
    value |= (((MaxDataStruct.curr_max * MaxDataStruct.rsense / 400) & 0xff) << 8);
    WriteWord(FuelGaugeAdress, MAX1726X_IALRTTH_REG, value);
}
float FuelGauge::max1726x_raw_current_to_mamps(uint16_t Value)
{

    if (Value & 0x8000)
    {
        Value = Value ^ 0xFFFF;
        Value = Value + 1;
        return (((float)Value * 156.25) / 1000) * -1;
    }
    return ((float)Value * 156.25) / 1000;
}

float FuelGauge::max1726x_raw_power_to_mwatts(uint16_t Value)
{

    return ((float)Value * 8) / 1000;
}

float FuelGauge::max1726x_raw_current_to_uamps(uint16_t Value)
{

    if (Value & 0x8000)
    {
        Value = Value ^ 0xFFFF;
        Value = Value + 1;
        return (((float)Value * 156.25) / 1000000) * -1;
    }
    return ((float)Value * 156.25) / 1000000;
}

float FuelGauge::max1726x_raw_voltage_to_volts(uint16_t Value)
{

    return (((float)Value * 78.125) / 1000000);
}

float FuelGauge::max1726x_raw_voltage_to_milivolts(uint16_t Value)
{

    return (((float)Value * 78.125) / 1000);
}

float FuelGauge::max1726x_raw_voltage_to_uvolts(uint16_t Value)
{

    return (float)Value * 78.125;
}

int FuelGauge::max1726x_raw_to_percent(uint16_t Value)
{

    return Value >> 8;
}

float FuelGauge::max1726x_raw_time_to_sec(uint16_t Value)
{

    return (float)Value * 5.625;
}

float FuelGauge::max1726x_raw_time_to_min(uint16_t Value)
{

    return ((float)Value * 5.625) / 60;
}

float FuelGauge::max1726x_raw_time_to_hour(uint16_t Value)
{

    return ((float)Value * 5.625) / 3600;
}

float FuelGauge::max1726x_raw_to_Celsius(uint16_t Value)
{

    /* The value is signed. */
    if (Value & 0x8000)
        Value |= 0xFFFF0000;

    /* The value is converted into centigrade scale */
    /* Units of LSB = 1 / 256 degree Celsius */
    Value >>= 8;

    return (float)Value;
}

void FuelGauge::ReadWord(uint8_t addr, uint8_t reg, uint16_t *final_value)
{
    // Read two bytes of data and store in a 16 bit data structure
    uint8_t buf[2];

    // Leemos LSB y luego MSB

    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, addr, buf, 2, false);

    // buf 0 is LSB buf 1 is msb.
    *final_value = (uint16_t)buf[0] | (uint16_t)(buf[1] << 8);
    sleep_ms(3);
}

void FuelGauge::WriteWord(uint8_t addr, uint8_t reg, uint16_t data)
{
    /*
    buf[0] = REG_TEMP_CRIT;
    buf[1] = crit_temp_lsb;
    buf[2] = crit_temp_msb;;

    Escribimos siempre lsb y luego msb.
    */
    uint8_t data_buff[3];

    data_buff[0] = reg;
    data_buff[1] = (uint8_t)data;
    data_buff[2] = data >> 8;

    i2c_write_blocking(i2c_default, addr, data_buff, 3, false);

    // uint16_t Aux =0x00;
    // ReadWord(FuelGaugeAdress,reg,&Aux);
    // printf("Escrito 0x%04x y leido 0x%04x\r\n",data,Aux);

#if 0


    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_write_blocking(i2c_default, addr, &lsb, 1, true);
    i2c_write_blocking(i2c_default, addr, &msb, 1, false);

#endif
}

void FuelGauge::VerifyAndWriteWord(uint8_t addr, uint8_t reg, uint16_t data)
{
    uint16_t Aux = 0x00;
    int retries = 10;

    // No se por que aquí se queda el micro clavado.
    do
    {
        WriteWord(addr, reg, data);
        sleep_ms(5);
        ReadWord(addr, reg, &Aux);
        if (Aux != data)
        {

            retries--;
        }
    } while ((retries > 0) && Aux != data);
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 */
void FuelGauge::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    ReadWord(devAddr, regAddr, &w);
    uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask;                     // zero all non-important bits in data
    w &= ~(mask);                     // zero all important bits in existing word
    w |= data;                        // combine data with existing word
    WriteWord(devAddr, regAddr, w);
}

void FuelGauge::FuelGaugeTask()
{

    uint16_t Status = 0x00;

    ReadWord(FuelGaugeAdress, STATUS_REG, &Status);
    //printf("Estado actual de Status 0b'" BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN "\r\n", BYTE_TO_BINARY(Status >> 8), BYTE_TO_BINARY(Status));

    // Aplicamos máscaras ahora para trabajar con un switch case.

    if (CHECK_BIT(Status, POWER_ON_RESET))
    {

        printf("## -- Start fuel gauge parameteres \r\n");
        InitializeFuelGauge();
    }
    else if (CHECK_BIT(Status, IMN_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, BATTERY_STATUS))
    {
    }
    else if (CHECK_BIT(Status, IMX_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, STATE_OF_CHARGE))
    {
        // Ha cambiado el porcentaje de batería por lo que actualizamos.
        // FuelValues.SoC = get_max1726x_SOC();
        printf("### ------ Change SOC %f\r\n", FuelValues.SoC);
        // Limpiamos el bit del registro STATE_OF_CHARGE
        writeBitsW(FuelGaugeAdress, STATUS_REG, STATE_OF_CHARGE, 1, 0x00);
    }
    else if (CHECK_BIT(Status, VMN_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, TMN_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, SMN_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, BATTERY_INSERTION))
    {
        printf("--------------------Battery Insertion caution \r\n");
        writeBitsW(FuelGaugeAdress, STATUS_REG, BATTERY_INSERTION, 1, 0x00);
    }
    else if (CHECK_BIT(Status, VMX_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, TMX_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, SMX_THRESHOLD))
    {
    }
    else if (CHECK_BIT(Status, BATTERY_REMOVAL))
    {

        printf("--------------------Battery Removal caution \r\n");
        writeBitsW(FuelGaugeAdress, STATUS_REG, BATTERY_REMOVAL, 1, 0x00);
    }

    // Una vez pasado todo esto, leemos los valores analógicos.
    FuelValues.SoC = get_max1726x_SOC(); // Debemos leerlo por lo menos la primera vez, puesto que luego el resto del programa depende de esto.

    FuelValues.RemainingCapacity = get_max1726x_RemainCapacity();
    FuelValues.FullRepCap = get_max1726x_FullCapRep();
    FuelValues.Cycles = get_max1726x_BattCycles();
    FuelValues.EstimateTimeToEmpty = get_max1726x_TTE();
    FuelValues.EstimateTimeToEmpty = get_max1726x_TTF();
    FuelValues.Voltage = get_max1726x_BattVoltage(V_RANGE);
    FuelValues.Voltage = get_max1726x_AvrgVoltage(V_RANGE);

    FuelValues.Current = get_max1726x_BattCurrent(A_RANGE);
    FuelValues.CurrentAvr = get_max1726x_AvgBattCurrent(A_RANGE);

    FuelValues.Temp = get_max1726x_BattTemp();
}