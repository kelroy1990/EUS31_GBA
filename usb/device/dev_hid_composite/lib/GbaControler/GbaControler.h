#ifndef GBACONTROLER_h
#define GBACONTROLER_h

#include <stdio.h>
#include "tusb.h"
#include "usb_descriptors.h"

#include "bsp/board.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "hardware/spi.h"

#include "pico/bootrom.h"

#include "lib/Settings/Board.h"

#include "lib/PWM/RP2040_PWM.h"

#include "lib/Max1726X/max1726x.h"

extern "C"
{
#include <hardware/sync.h>
#include <hardware/flash.h>
};

#define VALUE_TO_BIT(n, v) (v << (n))

/* From https://www.kernel.org/doc/html/latest/input/gamepad.html
          ____________________________              __
         / [__ZL__]          [__ZR__] \               |
        / [__ TL __]        [__ TR __] \              | Front Triggers
     __/________________________________\__         __|
    /                                  _   \          |
   /      /\           __             (N)   \         |
  /       ||      __  |MO|  __     _       _ \        | Main Pad
 |    <===DP===> |SE|      |ST|   (W) -|- (E) |       |
  \       ||    ___          ___       _     /        |
  /\      \/   /   \        /   \     (S)   /\      __|
 /  \________ | LS  | ____ |  RS | ________/  \       |
|         /  \ \___/ /    \ \___/ /  \         |      | Control Sticks
|        /    \_____/      \_____/    \        |    __|
|       /                              \       |
 \_____/                                \_____/

     |________|______|    |______|___________|
       D-Pad    Left       Right   Action Pad
               Stick       Stick

                 |_____________|
                    Menu Pad

  Most gamepads have the following features:
  - Action-Pad 4 buttons in diamonds-shape (on the right side) NORTH, SOUTH, WEST and EAST.
  - D-Pad (Direction-pad) 4 buttons (on the left side) that point up, down, left and right.
  - Menu-Pad Different constellations, but most-times 2 buttons: SELECT - START.
  - Analog-Sticks provide freely moveable sticks to control directions, Analog-sticks may also
  provide a digital button if you press them.
  - Triggers are located on the upper-side of the pad in vertical direction. The upper buttons
  are normally named Left- and Right-Triggers, the lower buttons Z-Left and Z-Right.
  - Rumble Many devices provide force-feedback features. But are mostly just simple rumble motors.
 */

/*
Estructura tendrá como dato:
-Batería
-Botones
--> Array

- USB C Charger
--> USB_Switch (Selector para el USBC, de manera que sea host o device)
--> ENBST


*/

//--------------------------------------------------------------------+
// Commands
//--------------------------------------------------------------------+

constexpr static const char *const REBOOT_USB = "[PI]Bootloader";
constexpr static const char *const HALT_NOW = "[PI]Halt_Hardware";

//--------------------------------------------------------------------+
// Enum
//--------------------------------------------------------------------+
enum RaspberryRun_State
{
  PI_STOPED = 0,
  PI_RUNINIG = 1,
  PI_SHUTDOWN = 3,
  PI_NO_SDCARD = 4,

};

enum Raspberrypi_Status
{
  STATUS_OFF = 0,
  STATUS_RUNNING = 1,
  STATUS_FAIL = 2,

};

enum Button_State
{
  RELEASED = 0,
  PRESSED = 1,
};

enum Led_State
{
  LD_OFF = 0,
  LD_ON = 1,
};
enum Charger_Status
{
  CHARGING = 0,
  CHARGED = 1,
  DISCHARGING = 2,
  REVERSE_BOOST = 3,
  UNKNOW = 4,
};

enum Vbost_Charger_Status
{
  MAX_NORMAL = 0,
  MAX_BOOST = 1,
};

enum Hardware_Status
{
  DEINIT = 0,
  RUNING = 1,
  BAT_LOW = 2,
  SHUTTING_DOWN_PI = 3,
  PI_IN_FAIL_STATE = 4,
  PI_WITHOUT_SDCARD_STATE = 5,

};

enum PWM_State
{
  WLED_STOPED = 0,
  WLED_RUNING = 1,
  WLED_CHANGE_DC_UP = 3,
  WLED_CHANGE_DC_DOWN = 4,
  WLED_STARTING = 5,
};

enum SYSTEM_Control_CMD
{
  DO_NOTHING = 0,
  POWER_OFF = 1,
  STANDBY = 2,
  WAKE_UP_HOST = 3,
};

enum MEDIA_Control
{
  MD_DO_NOTHING = 0,
  MD_VOLUME_UP = 1,
  MD_VOLUME_DOWN = 2,
  MD_MUTE = 3,
};

enum Event_type{
  EV_SYN = 0,
  EV_KEY = 1,
  EV_REL = 2,
  EV_ABS = 3,
  EV_MSC = 4,
};

//--------------------------------------------------------------------+
// Structs of each section
//--------------------------------------------------------------------+

struct Battery_t
{
  uint16_t Capacity;
  uint16_t SOC;
  float RemainingCapacity;
  float Voltage;
  float Current;
  float BatTemp;
  uint16_t Cycles;
};

struct Button_t
{
  uint8_t GPIO_Number;
  uint16_t ReadState;
  Button_State StateNow;
  Button_State StateLast;
};

struct Max_Charger_t
{
  Charger_Status StatusMAX;
  Vbost_Charger_Status EnbstOutput;
};

struct RaspberryPi_t
{
  uint8_t GPIO_run;
  RaspberryRun_State RunState;
  uint32_t BlinkTime;
};

struct WLED_LCD_t
{
  RP2040_PWM *PWM_Instance;
  uint16_t Frequency;
  uint16_t DutyCycle;   // Precision de (0 a 100) * 1000 si usamos SetPWM_Int, 0 a 100 si usamos SetPWM
  uint32_t FlashOffset; //! Solo debemos guardar la última posición del brillo.
  PWM_State State;      // 0 OFF, 1 ON
  bool Saved;           // Cada vez que hay un cambio esto se vuelve false, y true cuando se guarda en memoria.
};

struct MediaControl_t{

  uint8_t EVENT_TYPE;
  uint16_t KEY_CODE;
  uint32_t EV_ABS;
};


struct AUDIO_t
{
  uint8_t ActualDBConfigured;
  uint16_t VolumeValueDesired;
  uint16_t LastVolumeValueDesired;
  float LastPotValuemV;
  float PotValuemV;
  MEDIA_Control MD_Status;
  MediaControl_t ReportMD;
};



//--------------------------------------------------------------------+
// Struct that have all the information
//--------------------------------------------------------------------+

struct GbaControler_t
{

  Button_t Keys[NUMBER_OF_KEYS];
  Max_Charger_t MaximCharger;
  FuelGaugeReads_t *FuelGauge;
  // Battery_t FuelGauge;
  RaspberryPi_t PiZero2W;
  WLED_LCD_t Backlight;
  AUDIO_t AudioControl;
  Hardware_Status StatusMachineHardware;
};

//--------------------------------------------------------------------+
// Funciones del CS del SPI
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Class Begin
//--------------------------------------------------------------------+

class GbaHardware
{

public:
  GbaHardware();
  void HardwareControlTask();
  void UpdateKeys();
  void SampleKeys();
  void ReadFuelGauge();
  void PrintStructValue();
  void CheckStatusLedBehaivour();
  void CommandExecution(uint8_t *buffer, size_t len);
  void InitalizeHw();
  void BlinkingTask();
  void MediaControl();
  void SystemControl();

  void PrintFuelGauge(uint8_t reg)
  {
    BatDriver.PrintRegister(reg);
  }

  void PrintFuelGauge()
  {
    BatDriver.PrintValues();
  }
  void StartHubUsb()
  {

    // Ciclo de reset para el HUB USB
    gpio_put(RST_HUB, true);
    sleep_ms(80);
    gpio_put(RST_HUB, false);
    sleep_ms(100);
    gpio_put(RST_HUB, true);
  }

  void InitFuelGauge() { BatDriver.InitializeFuelGauge(); }

private:
  void UpdateMax();
  bool CheckBattery();
  void UpdateBacklight();
  void VolumeTask();
  void UpdateRPIState();
  void InitLCD();


  FuelGauge BatDriver;
  uint32_t TimeToShutdown = 0;

  //--------------------------------------------------------------------+
  // Function tools
  //--------------------------------------------------------------------+

  float adc_to_voltage_mv(uint16_t RawADCValue)
  {
    const float conversion_factor = 3.3f / (1 << 12);
    return conversion_factor * RawADCValue * 1000;
  }
  long map(long x, long in_min, long in_max, long out_min, long out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  void cs_select()
  {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_LCD, 0); // Active low
    asm volatile("nop \n nop \n nop");
  }

  void cs_deselect()
  {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_LCD, 1);
    asm volatile("nop \n nop \n nop");
  }

  void write_register(uint8_t reg, uint8_t data)
  {
    uint8_t buf[2];
    buf[0] = reg & 0x7f; // remove read bit as this is a write
    buf[1] = data;
    cs_select();
    spi_write_blocking(SPI_GBA, buf, 2);
    cs_deselect();
    sleep_ms(2);
  }

  void read_registers(uint8_t reg, uint8_t *buf, uint16_t len)
  {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_GBA, &reg, 1);
    sleep_ms(2);
    spi_read_blocking(SPI_GBA, 0, buf, len);
    cs_deselect();
    sleep_ms(2);
  }

  uint32_t timer_wled_update = 0;
  uint32_t timer_to_shutdown = 0;
  uint32_t time_interval = 0;
  uint32_t start_ms = 0;
  bool led_state = false;
  bool _shutdown_halt = false;
  SYSTEM_Control_CMD status = DO_NOTHING;
  bool UpdateMedia = false;

  void SaveNVMFlash()
  {

    if (GbaControler.Backlight.Saved)
      return;
    //! Si ya está guardado sobra ponerlo aquí.

    uint8_t buff[FLASH_PAGE_SIZE];
    memset(buff, 0xFF, FLASH_PAGE_SIZE); // No le cambiamos el BIT, de manera que así evitamos quemar zonas que no tocamos.
    buff[0] = BACKLIGHT_FLASH_CMD;
    buff[1] = (uint8_t)GbaControler.Backlight.DutyCycle >> 8;
    buff[2] = (uint8_t)GbaControler.Backlight.DutyCycle;

    uint32_t ints = save_and_disable_interrupts();
    // Erase the last sector of the flash
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    // Program buf[] into the first page of this sector
    flash_range_program(FLASH_TARGET_OFFSET, buff, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    GbaControler.Backlight.Saved = true;
  }

  void ReadNVMFlash()
  {

    const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

    if (flash_target_contents[0] == BACKLIGHT_FLASH_CMD)
    {
      //! Estamos leyendo backlight.
      GbaControler.Backlight.DutyCycle = (uint16_t)flash_target_contents[1] << 8 | (uint16_t)flash_target_contents[2];
      GbaControler.Backlight.Saved = false;
    }
    else
    {
      debug("No se ha guardado el dutycycle \r\n");
      return;
    }

    debug("Hemos recuperado el backlight con valor %d \r\n", GbaControler.Backlight.DutyCycle);
  }

  //! Initialize the struct data
  GbaControler_t GbaControler = {
      .Keys = {
          {.GPIO_Number = RIGHT, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = LEFT, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = DOWN, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = UP, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = A, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = B, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = L, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = R, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = START, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED},
          {.GPIO_Number = SELECT, .ReadState = 0, .StateNow = RELEASED, .StateLast = RELEASED}

      },
      .MaximCharger{.StatusMAX = UNKNOW, .EnbstOutput = MAX_NORMAL},
      //.FuelGauge{.Capacity = 0, .SOC = 10, .RemainingCapacity = 0, .Voltage = 0.00, .Current = 0.00, .BatTemp = 0.00, .Cycles = 0},
      .FuelGauge = nullptr,
      .PiZero2W{.GPIO_run = RUN_PI, .RunState = PI_STOPED, .BlinkTime = 0},
      .Backlight{.PWM_Instance = nullptr, .Frequency = FREQ_WLED_LCD, .DutyCycle = DEFAULT_DC_WLED, .FlashOffset = FLASH_TARGET_OFFSET, .State = WLED_STOPED, .Saved = false},
      .AudioControl{.ActualDBConfigured = 0, .VolumeValueDesired = 0, .LastVolumeValueDesired = 0, .LastPotValuemV = 0.0, .PotValuemV = 0.0, .MD_Status = MD_DO_NOTHING, .ReportMD = {.EVENT_TYPE = 0, .KEY_CODE = 0, .EV_ABS = 0}},
      .StatusMachineHardware = DEINIT

  };

  hid_gamepad_report_t GbaGamepadReport =
      {
          .x = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0, .hat = 0, .buttons = 0};
};

#endif