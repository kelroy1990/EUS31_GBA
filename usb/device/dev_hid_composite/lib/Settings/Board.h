////////////////////////////////*
////* Paterns
////////////////////////////////*

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)     \
  (byte & 0x80 ? '1' : '0'),     \
      (byte & 0x40 ? '1' : '0'), \
      (byte & 0x20 ? '1' : '0'), \
      (byte & 0x10 ? '1' : '0'), \
      (byte & 0x08 ? '1' : '0'), \
      (byte & 0x04 ? '1' : '0'), \
      (byte & 0x02 ? '1' : '0'), \
      (byte & 0x01 ? '1' : '0')

////////////////////////////////*
////* General defines
////////////////////////////////*
#define STRUCT_SAVED 0xA5
#define BACKLIGHT_FLASH_CMD 0xA0
#define RETURN_ON_OFF_BUTTON false
#define DEBUG_KEYS 0
#define I2C_BUS_SPEED 100000 // 100khz
#define DebugUSBLed 0


////////////////////////////////*
////* GPIOS Gamepad
////////////////////////////////*

#define NUMBER_OF_KEYS 10

#define UP 2
#define DOWN 4
#define LEFT 0
#define RIGHT 3

#define A 27
#define B 28

#define L 1
#define R 26

#define START 5
#define SELECT 6

////////////////////////////////*
////* TFuel Gauge
////////////////////////////////*

#define ENBST 22 // #define SEL_SWITCH 22 ahora son el mismo GPIO, ya que habilitan la misma función.
#define BTN_PRESS 10
#define GPIO_STH 21

////////////////////////////////*
////* Audio
////////////////////////////////*

#define ADC3_VOLUME_READ 29
#define MAX_VOLTAGE_RANGE 3300
#define MAX_POT_READ 3151
#define MIN_POT_READ 0
#define POT_DEAD_ZONE_MV 85 // Inicialmente eran 100
#define MAX_DB_VOLUME 6
#define DEFAULT_DB_VOLUME 0
#define MINIMUN_REGISTER_VALUE 0x00
#define MAX_REGISTER_VALUE 0x1F
#define MUTE 23
#define VOL_UP 24
#define VOL_DOWN 25
//! ADC RANGE 0 to 3,3V. Voltage max on the potentiometer 3,1518V

////////////////////////////////*
////* Display
////////////////////////////////*
#define RST_LCD 16
#define CS_LCD 20

#define MOSI_LCD 15
#define SCK_LCD 14
#define MISO_LCD 12

#define SPI_GBA spi1
#define READ_BIT 0x80

#define SPI_LCD_START_UP 0

////////////////////////////////*
////* Raspberry
////////////////////////////////*

#define RUN_PI 19
#define SD_DETECT 7
#ifdef LED_PIN
#undef LED_PIN
#define LED_PIN 11
#else
#define LED_PIN 11
#endif

#define LED_RP2040 11

////////////////////////////////*
////* USB HUB
////////////////////////////////*

#define RST_HUB 13

////////////////////////////////*
////* I2C
////////////////////////////////*
#define SDA_RP 8
#define SCL_RP 9
#define GBA_I2C_INSTANCE (&i2c0_inst) ///< Identifier for I2C HW Block 0

////////////////////////////////*
////* Backlight
////////////////////////////////*

#define WLED_LCD 18
#define FREQ_WLED_LCD 25 * 1000
#define DEFAULT_DC_WLED 80
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

////////////////////////////////*
////* Time constants
////////////////////////////////*

#define TIME_TO_FAIL_RPI 7000
#define TIME_TO_SHUTDOWN_PI 2000
#define HID_REPORT_TIME 100
#define SAMPLING_TIME (HID_REPORT_TIME / 20)
#define HARDWARE_TIME 250
#define BLINK_REVERSE_BOST_INTERVAL 2000
#define BLINK_LESS_30_PERCENT 1000
#define BLINK_LESS_15_PERCENT 500
#define BLINK_LESS_10_PERCENT 100
#define BLINK_REVERSE_BOOST 250
#define OFF_LED_RP2040 0
#define ON_PERMANENT_LED 1
#define PRINT_STRUCT_GBA_VALUES 3000
#define HUB_DELAY 500
#define START_DELAY 550
#define SHUTDOWN_WAIT_PI_TIME 10000
#define SHUTDOWN_BACKLIGHT 1500
#define NO_SDCARD_SHUTDOWN_TIME 5000
#define BLINK_NOSDCARD 250
#define TIME_TO_NEXT_UPDATE_BACKLIGTH 750
#define TIME_TO_DELAY_START_DISPLAY 3500


////////////////////////////////*
////* Mascaras de comandos
////////////////////////////////*

#define MASK_BACKLIGHT_UP (1 << POS_LINUX_BTN_START) | (1 << POS_LINUX_BTN_R)
#define MASK_BACKLIGHT_DOWN (1 << POS_LINUX_BTN_START) | (1 << POS_LINUX_BTN_L)

#define MASK_VOLUME_UP (1 << POS_LINUX_BTN_START) | (1 << POS_LINUX_BTN_A)
#define MASK_VOLUME_DOWN (1 << POS_LINUX_BTN_START) | (1 << POS_LINUX_BTN_B)
#define MASK_VOLUME_MUTE (1 << POS_LINUX_BTN_START) | (1 << POS_LINUX_BTN_B) | (1 << POS_LINUX_BTN_A)

#define MASK_BOOST_ENABLE (1 << POS_LINUX_BTN_SELECT) | (1 << POS_LINUX_BTN_L)
#define MASK_BOOST_DISABLE (1 << POS_LINUX_BTN_SELECT) | (1 << POS_LINUX_BTN_R)

//! This part its to adapt the HID report to Linux input expectations.
#define MASK_DPAD_CENTERED 0x00
#define MASK_DPAD_UP 0x01
#define MASK_DPAD_UP_RIGHT 0x09
#define MASK_DPAD_RIGHT 0x08
#define MASK_DPAD_DOWN_RIGHT 0x0A
#define MASK_DPAD_DOWN 0x02
#define MASK_DPAD_DOWN_LEFT 0x06
#define MASK_DPAD_LEFT 0x04
#define MASK_DPAD_LEFT_UP 0x05

#define POS_LINUX_BTN_A 0
#define POS_LINUX_BTN_B 1
#define POS_LINUX_BTN_L 6
#define POS_LINUX_BTN_R 7
#define POS_LINUX_BTN_START 11
#define POS_LINUX_BTN_SELECT 10

////////////////////////////////*
////* Bateria
////////////////////////////////*

#define BATT_CAPACITY 2000
#define SHUTDOWN_VOLTAGE_MV 3100

////////////////////////////////*
////* TAG MACRO
////////////////////////////////*

#define command(...) printf("[CMD]:" __VA_ARGS__)
#define debug(...) printf("[DBG]:" __VA_ARGS__)

////////////////////////////////*
////* Version
////////////////////////////////*

#define MAYOR_VERSION 1
#define MINOR_VERSION 0
#define PATCH_VERSION 0

#define RELEASE 0

