/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "usb_descriptors.h"

#include "hardware/i2c.h"
#include "pico/binary_info.h"

#include "hardware/spi.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include <time.h>

#include "lib/GbaControler/GbaControler.h"
#include "lib/Settings/Board.h"

#include "pico/multicore.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */

enum
{
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

GbaHardware _GbaHardware;

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
static uint32_t hid_time_task = 0;
static uint32_t keys_sampler_time = 0;
static uint32_t hardware_task_time = 0;
static uint32_t print_values_time = 0;
bool usb_start = false;

// Dirección del registro SIE_STATUS (Connected/NO connected posición 16 de 32) de la tabla USBCTRL_REGS_BASE, este indica si el USB está conectado o no. 1 == No conectado, 0 == Conectado.
uint32_t * USB_Connected_Address = (uint32_t *) (0x50110000 + 0x50);

/* Ejemplo de su utilización

      if( !bool(*USB_Connected_Address & (1<<16)))  gpio_put(LED_RP2040,true);
      else gpio_put(LED_RP2040,false);


*/

void led_blinking_task(void);
void hid_task(void);
void cdc_task(void);
void core1_entry();

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr)
{
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

/*------------- MAIN -------------*/
int main(void)
{

  // Incio board y GPIO
  board_init();
  tusb_init();
  stdio_usb_init();

  _GbaHardware.InitalizeHw();

  // Forzamos ON raspberry

  // gpio_put(RUN_PI, true);

  // multicore_launch_core1(core1_entry);

  // multicore_fifo_push_blocking((uintptr_t) &led_blinking_task);

  while (1)
  {

    //--------------------------------------------------------------------+
    // Main Loop Task without depedency
    //--------------------------------------------------------------------+

#if DebugUSBLed
    led_blinking_task(); // Tarea que hace blinkear el led de la placa.
#else
    _GbaHardware.BlinkingTask();
#endif
    tud_task(); // tinyusb device task
    cdc_task();


    //--------------------------------------------------------------------+
    // Initialize USB_HUB and USB task RP2040
    //--------------------------------------------------------------------+
    if ((to_ms_since_boot(get_absolute_time()) > HUB_DELAY) && (usb_start == false))
    {
      usb_start = true;
      _GbaHardware.StartHubUsb();
    }

    //--------------------------------------------------------------------+
    // Individually task after USB start
    //--------------------------------------------------------------------+
    
    if (to_ms_since_boot(get_absolute_time()) > print_values_time && (RELEASE == 0)) //! Solo pintamos en debug
    {
      // printf("HOla");
      _GbaHardware.PrintStructValue();
      //_GbaHardware.PrintFuelGauge();
      //_GbaHardware.PrintFuelGauge(0x3A);
      print_values_time = to_ms_since_boot(get_absolute_time()) + PRINT_STRUCT_GBA_VALUES;
    }

    if (to_ms_since_boot(get_absolute_time()) > hardware_task_time)
    {
      _GbaHardware.HardwareControlTask();
      hardware_task_time = to_ms_since_boot(get_absolute_time()) + HARDWARE_TIME;
    }

    if (to_ms_since_boot(get_absolute_time()) > print_values_time && (RELEASE == 0)) //! Solo pintamos en debug
    {
      _GbaHardware.PrintStructValue();
      //_GbaHardware.PrintFuelGauge();
      //_GbaHardware.PrintFuelGauge(0x3A);
      print_values_time = to_ms_since_boot(get_absolute_time()) + PRINT_STRUCT_GBA_VALUES;
    }

    if (to_ms_since_boot(get_absolute_time()) > hid_time_task)
    {
      hid_task();
      hid_time_task = to_ms_since_boot(get_absolute_time()) + HID_REPORT_TIME;
    }

    if (to_ms_since_boot(get_absolute_time()) > keys_sampler_time)
    {
      _GbaHardware.SampleKeys();
      keys_sampler_time = to_ms_since_boot(get_absolute_time()) + SAMPLING_TIME;
    }
  }

  return 0;
}

void core1_entry()
{

  //  int32_t (*debug)() = (int32_t(*)()) multicore_fifo_pop_blocking();
  int32_t (*func)() = (int32_t(*)())multicore_fifo_pop_blocking();
  while (1)
  {

    (*func)();
    // sleep_ms(1);

    // Function pointer is passed to us via the FIFO
    // We have one incoming int32_t as a parameter, and will provide an
    // int32_t return value by simply pushing it back on the FIFO
    // which also indicates the result is ready.

    // int32_t p = multicore_fifo_pop_blocking(); This is for execute a function with parameter, example (*func)(p);

    // Not working, only works if we used the pointer to the function max17048_read_data()
    //  Test maybe with a static void max17048_read_data()???
    // uint16_t data_=0x0000;
    // max17048_read_data(0x36,0x04,&data_);  //Seems that the Core1 dont know where its the object to execute.
    // printf("SOC %d\r\n",(data_>>8) );
    // data_=0x0000;
    // max17048_read_data(0x36,0x02,&data_);
    // printf("Con voltaje %f\r\n",(float)(data_*78.125/1000000)   );

    // Pointer to the function passed through the FIFO
    // Working the max17048_read_data() if passed with pointer
    // sleep_ms(1000);

    // Here a state machine should be initialized
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id)
{
  // skip if hid is not ready yet
  if (!tud_hid_ready())
  {
    debug("----- Tarea de tud hid no ready-----");
    // TODO Si esto pasa de manera continua, no podremos usar el control.... impliciaría reinicio de todo.
    return;
  }

  // debug("Reportin HID with ID %d\r\n",report_id);
  switch (report_id)
  {

  case REPORT_ID_GAMEPAD:
    _GbaHardware.UpdateKeys();

    break;

  case REPORT_ID_SYSTEM_CONTROL:
    _GbaHardware.SystemControl();

    break;
  case REPORT_ID_CONSUMER_CONTROL:
    _GbaHardware.MediaControl();

  default:

    break;
  }
}

/*

Este apartado se envía al segundo core, y lo que hace es registrar todas las teclas y enviar el reporte de HID.

*/
void hid_task(void)
{

  // Remote wakeup
  if (tud_suspended())
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }
  else
  {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_GAMEPAD);
  }

  return;
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
  // Normalmente en deshuso, es decir, si queremos lanzar un reporte diferente deberíamos utilizar otra task como el HID task.
  (void)instance;
  (void)len;

  uint8_t next_report_id = report[0] + 1u;
  // debug("Estamos enviando el reporte %d, y partimos del reporte %d\r\n", next_report_id, report[0]);

  if (next_report_id < REPORT_ID_COUNT)
  {
    send_hid_report(next_report_id);
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
  (void)instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    return;
  }
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{

  if (tud_cdc_connected())
  {
    // connected and there are data available
    uint16_t size_avialble = 0;
    if ((size_avialble = tud_cdc_available()) >= 4)
    // El resto que recibamos lo consideramos basura, ya que como mínimo tiene que ser un comando encabezado por [PI]
    {

      uint8_t buf[64];

      // read and echo back
      uint32_t count = tud_cdc_read(buf, sizeof(buf));
      debug("------------------TUD available %d y recibido %s---------- \r\n", size_avialble, buf);
      _GbaHardware.CommandExecution(buf, count);
    }
  }
}
// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void)itf;

  // connected
  if (dtr && rts)
  {
    // print initial message when connected
    char str[40];

    sprintf(str, "\r\n[DBG]: RP2040 Version %d.%d.%d\r\n", MAYOR_VERSION, MINOR_VERSION, PATCH_VERSION);
    tud_cdc_write_str(str);
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void)itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+

void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms)
    return;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms)
    return; // not enough time
  start_ms += blink_interval_ms;

  gpio_put(LED_RP2040, led_state);
  led_state = 1 - led_state; // toggle
}
