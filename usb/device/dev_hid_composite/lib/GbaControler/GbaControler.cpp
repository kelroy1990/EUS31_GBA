#include "GbaControler.h"

GbaHardware::GbaHardware()
{
    return;
}

void GbaHardware::InitalizeHw()
{

    // Set of the GPIO input
    for (int h = 0; h < NUMBER_OF_KEYS; h++)
    {
        gpio_init(GbaControler.Keys[h].GPIO_Number);
        gpio_pull_up(GbaControler.Keys[h].GPIO_Number);
        gpio_set_dir(GbaControler.Keys[h].GPIO_Number, GPIO_IN);
    }

    ////////////////////////////////*
    ////* LCD
    ////////////////////////////////*

    gpio_init(RST_LCD);
    gpio_set_dir(RST_LCD, GPIO_OUT);
    gpio_put(RST_LCD, true);

    gpio_init(CS_LCD);
    gpio_pull_up(CS_LCD);
    gpio_set_dir(CS_LCD, GPIO_OUT);
    gpio_put(CS_LCD, true);

    ////////////////////////////////*
    ////* SPI
    ////////////////////////////////*

#if SPI_LCD_START_UP
    spi_init(SPI_GBA, 0.5 * 1000 * 1000);

    gpio_set_function(SCK_LCD, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_LCD, GPIO_FUNC_SPI);
    // gpio_set_function(MISO_LCD,GPIO_FUNC_SPI);

    // Initialize LCD

    sleep_ms(5);

    gpio_put(RST_LCD, false);
    sleep_ms(150);
    gpio_put(RST_LCD, true);
    sleep_ms(10);
    write_register(0x0A, 0x03);
    write_register(0x01, 0xF8); // F8
    write_register(0x0B, 0x77);

    // Contrast Green
    write_register(0x04, 0x35);
    // Contrast RED
    write_register(0x05, 0x30);
    // Contrast BLUE
    write_register(0x06, 0x30);
    // Brightness Green
    write_register(0x07, 0x25);
    // Sub brightness R
    write_register(0x08, 0x20);
    // Sub brightness B
    write_register(0x09, 0x20);

    // GAMMA

    write_register(0x0C, 0xCC);
    write_register(0x0D, 0xCC);
    write_register(0x0E, 0xCC);
    write_register(0x0F, 0x8B);
    write_register(0x10, 0x0A);
    write_register(0x11, 0xCC);
    write_register(0x12, 0xCC);
    write_register(0x13, 0xCC);
    write_register(0x14, 0x8B);
    write_register(0x15, 0x0A);
    write_register(0x16, 0xCC);
    write_register(0x17, 0xCC);
    write_register(0x18, 0xCC);
    write_register(0x19, 0x8B);
    write_register(0x1A, 0x0A);

    /*
        write_register(0x0A, 0x03);
        write_register(0x01, 0xf8); // 0xd8//高两位调翻转// Orientacion display
        write_register(0x0B, 0x77);

        write_register(0x00, 0xf8); // VCOM.

        write_register(0x0c, 0xcc);
        write_register(0x0d, 0xcc);
        write_register(0x0e, 0xcc);

        write_register(0x0F, 0x8b);
        write_register(0x10, 0x0a);
        write_register(0x11, 0xcc);

        write_register(0x12, 0xcc);
        write_register(0x13, 0xcc);
        write_register(0x14, 0x8B);
        write_register(0x15, 0x0a);
        write_register(0x16, 0xcc);
        write_register(0x17, 0xcc);
        write_register(0x18, 0xcc);
        write_register(0x19, 0x8B);
        write_register(0x1A, 0x0A);

    */

#endif

    ////////////////////////////////*
    ////* I2C
    ////////////////////////////////*
    i2c_init(i2c_default, I2C_BUS_SPEED);
    gpio_set_function(SDA_RP, GPIO_FUNC_I2C);
    gpio_set_function(SCL_RP, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_RP);
    gpio_pull_up(SCL_RP);

    ////////////////////////////////*
    ////* INPUTS
    ////////////////////////////////*
    gpio_init(SD_DETECT);
    gpio_set_dir(SD_DETECT, GPIO_IN);

    gpio_init(BTN_PRESS);
    gpio_set_dir(BTN_PRESS, GPIO_IN);

    ////////////////////////////////*
    ////* OUTPUTS
    ////////////////////////////////*
    gpio_init(ENBST);
    gpio_set_dir(ENBST, GPIO_OUT);
    gpio_put(ENBST, false);

    gpio_init(GPIO_STH);
    gpio_set_dir(GPIO_STH, GPIO_OUT);
    gpio_put(GPIO_STH, false);

    gpio_init(LED_RP2040);
    gpio_set_dir(LED_RP2040, GPIO_OUT);
    gpio_put(LED_RP2040, true);

    ////////////////////////////////*
    ////* AUDIO
    ////////////////////////////////*

    gpio_init(MUTE);
    gpio_set_dir(MUTE, GPIO_OUT);
    gpio_put(MUTE, false);

    gpio_init(VOL_UP);
    gpio_set_dir(VOL_UP, GPIO_OUT);
    gpio_put(VOL_UP, false);

    gpio_init(VOL_DOWN);
    gpio_set_dir(VOL_DOWN, GPIO_OUT);
    gpio_put(VOL_DOWN, false);

    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(ADC3_VOLUME_READ);
    adc_select_input(3);

    ////////////////////////////////*
    ////* PI
    ////////////////////////////////*

    gpio_init(RUN_PI);
    gpio_pull_up(RUN_PI);
    gpio_set_dir(RUN_PI, GPIO_OUT);
    gpio_put(RUN_PI, false);

    ////////////////////////////////*
    ////* Backlight
    ////////////////////////////////*

    // gpio_init(WLED_LCD);
    // gpio_set_dir(WLED_LCD, GPIO_OUT);
    // gpio_set_drive_strength(WLED_LCD,GPIO_DRIVE_STRENGTH_12MA);
    GbaControler.Backlight.PWM_Instance = new RP2040_PWM(WLED_LCD, FREQ_WLED_LCD, DEFAULT_DC_WLED);
    GbaControler.Backlight.PWM_Instance->setPWM();
    GbaControler.Backlight.PWM_Instance->disablePWM();
    GbaControler.Backlight.State = WLED_STOPED;

    ////////////////////////////////*
    ////* Battery Fuel Gauge
    ////////////////////////////////*
    /*
        IChgTerm Register (1Eh)
        Register Type: Current
        Initial Value: 0x0640 (250mA on 10mΩ)
        Battery Charger Value : 0x0960 (150mA on 10mΩ)

        0x9B61 Vempty register. 3.1V/3.8V (Vempty/Vfull)
        Current 1.5625μV/RSENSE
        Voltage 78.125μV
    */

    BatDriver.SetBatteryProperties(BATT_CAPACITY, 0x0960, 0x5F05, 0x9B61, 4200);
    BatDriver.FuelGaugeTask();
    GbaControler.FuelGauge = BatDriver.PointerToFuelGaugeValues();

    ////////////////////////////////*
    ////* HUB
    ////////////////////////////////*

    gpio_init(RST_HUB);
    gpio_set_dir(RST_HUB, GPIO_OUT);
    gpio_put(RST_HUB, false);
}

void GbaHardware::ReadFuelGauge()
{
    BatDriver.FuelGaugeTask();

#if 0
    GbaControler.FuelGauge.Capacity = BatDriver.get_max1726x_DesignCap();
    GbaControler.FuelGauge.BatTemp = BatDriver.get_max1726x_BattTemp();
    GbaControler.FuelGauge.SOC = BatDriver.get_max1726x_SOC();
    GbaControler.FuelGauge.Current = BatDriver.get_max1726x_BattCurrent(MILIAMPERE);
    GbaControler.FuelGauge.Voltage = BatDriver.get_max1726x_BattVoltage(MILIVOLT);
    GbaControler.FuelGauge.RemainingCapacity = BatDriver.get_max1726x_RemainCapacity();
    GbaControler.FuelGauge.Cycles = BatDriver.get_max1726x_BattCycles();
#endif
}

void GbaHardware::SampleKeys()
{
    // Vamos a leer un 0 a tecla pulsada, pero lo negamos para la mascara
    // GbaControler.Keys[h].ReadState |= (!(sio_hw->gpio_in & (1u << GbaControler.Keys[h].GPIO_Number))) << i;
    for (int h = 0; h < NUMBER_OF_KEYS; h++)
    {

        for (int i = 0; i < 16; i++)
        {
            GbaControler.Keys[h].ReadState = (GbaControler.Keys[h].ReadState << 1) | !(gpio_get(GbaControler.Keys[h].GPIO_Number));
        }
    }
}

void GbaHardware::UpdateKeys()
{
    GbaGamepadReport.buttons = 0;
    GbaGamepadReport.hat = 0;
    uint8_t _hat = 0;
    bool shortcut = false;

    // Se escanea cada uno de los botones, y un total de 16 veces.
    for (int h = 0; h < NUMBER_OF_KEYS; h++)
    {

        // Primero actualizamos el last state
        GbaControler.Keys[h].StateLast = GbaControler.Keys[h].StateNow;
        // printf("Leemos estado de tecla %d con %#02x\r\n", h, GbaControler.Keys[h].ReadState);

        if (GbaControler.Keys[h].ReadState >= 0x07FF && GbaControler.Keys[h].StateLast == RELEASED)
        {
            GbaControler.Keys[h].StateNow = PRESSED;
            // printf("##--KEY %d pressed \r\n", h);
        }
        else if (GbaControler.Keys[h].ReadState <= 0xC000 && GbaControler.Keys[h].StateLast == PRESSED)
        {
            GbaControler.Keys[h].StateNow = RELEASED;
        }

        // Following Linux input codes events from hid.h
        switch (GbaControler.Keys[h].GPIO_Number)
        {
        case UP:
            _hat = _hat | GbaControler.Keys[h].StateNow;
            break;
        case DOWN:
            _hat = _hat | GbaControler.Keys[h].StateNow << 1;
            break;
        case LEFT:
            _hat = _hat | GbaControler.Keys[h].StateNow << 2;
            break;
        case RIGHT:
            _hat = _hat | GbaControler.Keys[h].StateNow << 3;
            break;

        case A:
            GbaGamepadReport.buttons |= GbaControler.Keys[h].StateNow << POS_LINUX_BTN_A;
            break;
        case B:
            GbaGamepadReport.buttons |= GbaControler.Keys[h].StateNow << POS_LINUX_BTN_B;
            break;
        case L:
            GbaGamepadReport.buttons |= GbaControler.Keys[h].StateNow << POS_LINUX_BTN_L;
            break;
        case R:
            GbaGamepadReport.buttons |= GbaControler.Keys[h].StateNow << POS_LINUX_BTN_R;
            break;
        case START:
            GbaGamepadReport.buttons |= GbaControler.Keys[h].StateNow << POS_LINUX_BTN_START;
            break;
        case SELECT:
            GbaGamepadReport.buttons |= GbaControler.Keys[h].StateNow << POS_LINUX_BTN_SELECT;
            break;

        default:
            break;
        }
    }

    /*
    Ahora el hat


            0 = UP
            1 = DOWN
            2 = LEFT
            3 = RIGHT
                    R L D U
            X X X X 0 0 0 0 DPAD_CENTERED
            X X X X 0 0 0 1 DPAD_UP
            X X X X 1 0 0 1 DPAD_UP_RIGHT
            X X X X 1 0 0 0 DPAD_RIGHT
            X X X X 1 0 1 0 DPAD_DOWN_RIGHT
            X X X X 0 0 1 0 DPAD_DOWN
            X X X X 0 1 1 0 DPAD_DOWN_LEFT
            X X X X 0 1 0 0 DPAD_LEFT
            X X X X 0 1 0 1 DPAD_UP_LEFT

    */
    switch (_hat)
    {

    case MASK_DPAD_CENTERED:
        GbaGamepadReport.hat = GAMEPAD_HAT_CENTERED;
        break;
    case MASK_DPAD_UP:
        GbaGamepadReport.hat = GAMEPAD_HAT_UP;
        break;
    case MASK_DPAD_UP_RIGHT:
        GbaGamepadReport.hat = GAMEPAD_HAT_UP_RIGHT;
        break;
    case MASK_DPAD_RIGHT:
        GbaGamepadReport.hat = GAMEPAD_HAT_RIGHT;
        break;
    case MASK_DPAD_DOWN_RIGHT:
        GbaGamepadReport.hat = GAMEPAD_HAT_DOWN_RIGHT;
        break;
    case MASK_DPAD_DOWN:
        GbaGamepadReport.hat = GAMEPAD_HAT_DOWN;
        break;
    case MASK_DPAD_DOWN_LEFT:
        GbaGamepadReport.hat = GAMEPAD_HAT_DOWN_LEFT;
        break;
    case MASK_DPAD_LEFT:
        GbaGamepadReport.hat = GAMEPAD_HAT_LEFT;
        break;
    case MASK_DPAD_LEFT_UP:
        GbaGamepadReport.hat = GAMEPAD_HAT_UP_LEFT;
        break;
    default:
        // printf("No deberíamos llegar aquí... \r\n");
        GbaGamepadReport.hat = GAMEPAD_HAT_CENTERED;
        break;
    }

    switch (GbaGamepadReport.buttons)
    {

    case MASK_BACKLIGHT_UP:
        if (GbaControler.Backlight.State == WLED_RUNING)
            GbaControler.Backlight.State = WLED_CHANGE_DC_UP;
        shortcut = true;
        break;

    case MASK_BACKLIGHT_DOWN:
        if (GbaControler.Backlight.State == WLED_RUNING)
            GbaControler.Backlight.State = WLED_CHANGE_DC_DOWN;
        shortcut = true;
        break;

    case MASK_BOOST_ENABLE:
        GbaControler.MaximCharger.EnbstOutput = MAX_BOOST;
        shortcut = true;
        break;

    case MASK_BOOST_DISABLE:
        GbaControler.MaximCharger.EnbstOutput = MAX_NORMAL;
        shortcut = true;
        break;

    case MASK_VOLUME_UP:
        GbaControler.AudioControl.MD_Status = MD_VOLUME_UP;
        UpdateMedia = 1;
        shortcut = true;
        break;

    case MASK_VOLUME_DOWN:
        GbaControler.AudioControl.MD_Status = MD_VOLUME_DOWN;
        UpdateMedia = 1;
        shortcut = true;
        break;

    case MASK_VOLUME_MUTE:
        GbaControler.AudioControl.MD_Status = MD_MUTE;
        UpdateMedia = 1;
        shortcut = true;
        break;

    default:
        // Si no hay ningún shortcut nos ponemos en false y enviamos el HID report
        shortcut = false;
        break;
    }

    if (shortcut) // Solo reportamos si no hay un shortcut en funcionamiento.
    {
        GbaGamepadReport.buttons = 0;
        GbaGamepadReport.hat = GAMEPAD_HAT_CENTERED;
    }

    // tud_hid_gamepad_report(REPORT_ID_GAMEPAD,0,0,0,0,0,0,GbaGamepadReport.hat,GbaGamepadReport.buttons);

    tud_hid_report(REPORT_ID_GAMEPAD, &GbaGamepadReport, sizeof(GbaGamepadReport));

#if DEBUG_KEYS
    debug("Estado actual de las teclas 0b'" BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN " y estado actual del hat 0b'" BYTE_TO_BINARY_PATTERN "\r\n",
          BYTE_TO_BINARY(GbaGamepadReport.buttons >> 24), BYTE_TO_BINARY(GbaGamepadReport.buttons >> 16), BYTE_TO_BINARY(GbaGamepadReport.buttons >> 8), BYTE_TO_BINARY(GbaGamepadReport.buttons), BYTE_TO_BINARY(GbaGamepadReport.hat));
#endif
}

void GbaHardware::SystemControl()
{

    tud_hid_report(REPORT_ID_SYSTEM_CONTROL, &status, 1);
}

void GbaHardware::MediaControl()
{

    // Event: time 1683669168.443066, type 4 (EV_MSC), code 4 (MSC_SCAN), value c00e9
    // Event: time 1683669168.443066, type 1 (EV_KEY), code 115 (KEY_VOLUMEUP), value 1
    // Event: time 1683669168.443066, -------------- SYN_REPORT ------------

    uint16_t Report[1];

    switch (GbaControler.AudioControl.MD_Status)
    {
    case MD_DO_NOTHING:
        Report[0] = 0x00; // Event type 3

        break;

    case MD_VOLUME_UP:
        if (UpdateMedia == 1)
        {
            Report[0] = HID_USAGE_CONSUMER_VOLUME_INCREMENT;

            UpdateMedia = false;
        }
        else
        {
            Report[0] = 0x00;
        }

        break;

    case MD_VOLUME_DOWN:
        if (UpdateMedia == 1)
        {
            Report[0] = HID_USAGE_CONSUMER_VOLUME_DECREMENT; // Event type 3
                                                             // Event code 32 ABS volume
            UpdateMedia = false;
        }
        else
        {
            Report[0] = 0x00;
        }
        break;

    case MD_MUTE:
        if (UpdateMedia == 1)
        {

            Report[0] = HID_USAGE_CONSUMER_MUTE; // Event type 3
                                                 // Event code 32 ABS volume
            UpdateMedia = false;
        }
        else
        {
            Report[0] = 0x00; // Event type 3

            GbaControler.AudioControl.MD_Status = MD_DO_NOTHING;
        }
        break;

    default:
        break;
    }

    tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &Report, sizeof(Report));
}
void GbaHardware::HardwareControlTask()
{ // Aquí vamos a ejecutar las tareas de control y revisión, como setear los estados y control del hardware.

    // uint8_t data = 0x00;
    // read_registers(0x0A, &data, 0);
    // printf("Leemos el registo 0xA0 0x%X", data);
    switch (GbaControler.StatusMachineHardware)
    {

    case DEINIT:

        // Debemos ver el estdo de la tarjeta, si hay tarjeta SD o no metida dentro. Cuando hay tarjeta leemos un 0, cuando no hay tarjeta leemos un 1.

        ReadFuelGauge();

        // 1.- Actualizamos estado del cargador, para poder actualizarlo tenemos el caso de ENBOOST o NORMAL.
        UpdateMax();
        // Acualizamos el estado del backlight también.
        ReadNVMFlash();

        //   2.- Registramos estado de la PI, inicialmente cuando encendemos debería estar apagado y queremos encenderlo.
        //! Condición de encendido, que previamente esté en STOPED (que es el valor default) y que la batería sea mayor al 5%, o en su
        //! defecto, que esté cargando la batería.

        if (((GbaControler.PiZero2W.RunState == PI_STOPED && GbaControler.FuelGauge->SoC > 5) || (GbaControler.PiZero2W.RunState == PI_STOPED && GbaControler.FuelGauge->Current > 0.00)) && gpio_get(SD_DETECT) == 0)
        {
            GbaControler.PiZero2W.RunState = PI_RUNINIG;
            //! Escoger uno de los dos métodos, o hacemos GND - HIGHZ, o GND y HIGH.
            gpio_put(RUN_PI, true);

            if (GbaControler.Backlight.State == WLED_STOPED)
            {
                // GbaControler.Backlight.PWM_Instance->enablePWM();
                GbaControler.Backlight.State = WLED_STARTING;
            }
            // Cambiamos de estado
            GbaControler.StatusMachineHardware = RUNING;
        } // Si la batería es baja y tenemos la SD, es que queremos encender, no podemos por batería baja, encendemos led y apagamos.
        else if (GbaControler.FuelGauge->SoC < 5 && gpio_get(SD_DETECT) == 0)
        {
            debug("BAT LOW, no encendemos la consola y vamos al estado 3... \r\n");
            gpio_put(RUN_PI, false);
            time_interval = ON_PERMANENT_LED;
            GbaControler.PiZero2W.RunState = PI_STOPED;
            TimeToShutdown = to_ms_since_boot(get_absolute_time()) + SHUTDOWN_WAIT_PI_TIME;
            GbaControler.StatusMachineHardware = SHUTTING_DOWN_PI;
        }

        // Este estado debe contemplar dos casos, batería baja y no SD.
        else if (gpio_get(SD_DETECT) == 1)
        {
            GbaControler.PiZero2W.RunState = PI_NO_SDCARD;
            GbaControler.StatusMachineHardware = PI_WITHOUT_SDCARD_STATE;
            timer_to_shutdown = to_ms_since_boot(get_absolute_time()) + NO_SDCARD_SHUTDOWN_TIME;
            time_interval = BLINK_NOSDCARD;
        }

        break;

    case RUNING:
        /*
         *   Una vez está en funcionamiento aquí lo que tenemos será la gestión con el sistema operativo en funcionamiento, así como la gestión
         *   de los periféricos, debemos ver si hay una pulsación
         */

        if (gpio_get(BTN_PRESS) || _shutdown_halt)
        {
            // Shutdown completo
            SaveNVMFlash(); //! Solo se va a ejecutar una vez, ya que tiene flag de saved.

            GbaControler.PiZero2W.RunState = PI_SHUTDOWN;
            GbaControler.StatusMachineHardware = SHUTTING_DOWN_PI;
            TimeToShutdown = to_ms_since_boot(get_absolute_time()) + SHUTDOWN_WAIT_PI_TIME;
            time_interval = ON_PERMANENT_LED;
        }

        else
        {

            //! TASK NORMAL STATE
            // debug("TASK PRINCIPAL\r\n");
            UpdateRPIState();
            ReadFuelGauge();
            UpdateMax();
            UpdateBacklight();
            VolumeTask();

            if (CheckBattery())
            {
                //? Guardamos en la Flash lo necesario.
                SaveNVMFlash();
                debug("Apagado iniciado por baja VBAT o por fallo en booteo.\r\n");
                //! Ejecutamos apagado de la pi por bateria.
                TimeToShutdown = to_ms_since_boot(get_absolute_time()) + SHUTDOWN_WAIT_PI_TIME;
                time_interval = ON_PERMANENT_LED;
                GbaControler.StatusMachineHardware = SHUTTING_DOWN_PI;
                GbaControler.PiZero2W.RunState = PI_SHUTDOWN;
            }
        }

        break;

        /*
        //! En este caso, el led si que se queda en una posición de apagado¡, por lo que podemos usarlo para saber que ha terminado y poner el RUN PI a GND.

        */

    case SHUTTING_DOWN_PI:

                // Ejecutamos el apagado mediante el USB HID SYSTEM CONTROL.
        status = POWER_OFF;
        //! Ejecutamos tareas de control.
        UpdateRPIState();
        ReadFuelGauge();
        UpdateMax();
        UpdateBacklight();

        // Volvemos al main, y vamos a esperar de manera continua a que la PI nos diga un shutdown, si pasado 30 segundos no ocurre, entonces apagamos el sistema por nuestra cuenta.

        if (_shutdown_halt == true || TimeToShutdown < to_ms_since_boot(get_absolute_time()))
        {

            gpio_put(LED_RP2040, true);
            gpio_put(RST_LCD, false);
            gpio_put(GPIO_STH, true); //! APAGADO
            sleep_ms(4500);
            while (1)
                ;
            //?------------------------------------------
            //!..... never reach, todo apagado.
            //?------------------------------------------
        }
        else if ((TimeToShutdown - (SHUTDOWN_WAIT_PI_TIME - SHUTDOWN_BACKLIGHT)) < to_ms_since_boot(get_absolute_time()))
        {

            GbaControler.Backlight.State = WLED_STOPED;
        }

        break;

    case PI_IN_FAIL_STATE:
        // TODO, cuando esté listo el servicio si este no contesta a un ACK sabemos que la consola está dummy, por lo que podemos dar un tiempo prudencial y reiniciarla con el RUN_PIN

        break;

    case PI_WITHOUT_SDCARD_STATE:

        // Mientras no sea tiempo de apagarnos no hacemos nada.
        if (timer_to_shutdown < to_ms_since_boot(get_absolute_time()))
        {
            gpio_put(LED_RP2040, true);
            gpio_put(RST_LCD, false);
            gpio_put(GPIO_STH, true); //! APAGADO
            sleep_ms(4500);
            while (1)
                ;
            //?------------------------------------------
            //!..... never reach, todo apagado.
            //?------------------------------------------
        }

        break;

    default:
        break;
    }
}

void GbaHardware::UpdateMax()
{
    if (GbaControler.FuelGauge->Current < 0.00 && GbaControler.MaximCharger.EnbstOutput == MAX_NORMAL && GbaControler.MaximCharger.StatusMAX != REVERSE_BOOST)
        GbaControler.MaximCharger.StatusMAX = DISCHARGING;
    else if (GbaControler.FuelGauge->Current > 0.00 && GbaControler.MaximCharger.EnbstOutput == MAX_NORMAL && GbaControler.MaximCharger.StatusMAX != REVERSE_BOOST)
        GbaControler.MaximCharger.StatusMAX = CHARGING;

    //! Ahora gestión de estados especiales.
    if (GbaControler.MaximCharger.EnbstOutput == MAX_BOOST)
    {
        // Entonces podemos hacer el boost inverter
        //! 1.- Estar en discharging, no podemos habilitarlo si estamos con un cargador conectado.
        if (GbaControler.MaximCharger.StatusMAX == DISCHARGING && GbaControler.FuelGauge->Current < 0.00 && GbaControler.FuelGauge->SoC > 15)
        {
            debug("## -- Iniciamos modo Reverse Boost --\r\n");
            GbaControler.MaximCharger.StatusMAX = REVERSE_BOOST;
            gpio_put(ENBST, true);
            time_interval = BLINK_REVERSE_BOST_INTERVAL;
        }
        else
        {
            debug("## -- No se puede iniciar el modo Reverse Boost, por que está el dispositivo en estado de carga");
            // Volvemos al estado normal
            GbaControler.MaximCharger.EnbstOutput = MAX_NORMAL;
        }
    }
    else if (GbaControler.MaximCharger.EnbstOutput == MAX_NORMAL && GbaControler.MaximCharger.StatusMAX == REVERSE_BOOST)
    {
        debug("## -- Desactivamos modo Reverse Boost --\r\n");
        // Esto quiere decir que queremos volver al estado normal.
        gpio_put(ENBST, false);
        time_interval = OFF_LED_RP2040;
        GbaControler.MaximCharger.StatusMAX = UNKNOW;
    }
}

void GbaHardware::BlinkingTask()
{

    // blink is disabled
    if (time_interval == 0)
    {
        gpio_put(LED_RP2040, 1);
        led_state = 1;
        return;
    }
    else if (time_interval == ON_PERMANENT_LED)
    {

        gpio_put(LED_RP2040, 0);
        led_state = 0;
        return;
    }

    // Blink every interval ms
    if (board_millis() - start_ms < time_interval)
        return; // not enough time
    start_ms += time_interval;

    gpio_put(LED_RP2040, led_state);

    led_state = 1 - led_state; // toggle
}

bool GbaHardware::CheckBattery()
{
    if (GbaControler.MaximCharger.StatusMAX == DISCHARGING && GbaControler.FuelGauge->Voltage > SHUTDOWN_VOLTAGE_MV && GbaControler.FuelGauge->SoC > 5)
    {
        //! Estamos descargando la batería, por lo que debemos comparar el voltaje y el SoC.
        if (GbaControler.FuelGauge->SoC < 30 && GbaControler.FuelGauge->SoC >= 16)
        {
            time_interval = BLINK_LESS_30_PERCENT;
        }
        else if (GbaControler.FuelGauge->SoC < 15 && GbaControler.FuelGauge->SoC >= 11)
        {
            time_interval = BLINK_LESS_15_PERCENT;
        }
        else if (GbaControler.FuelGauge->SoC < 10 && GbaControler.FuelGauge->SoC >= 5)
        {
            time_interval = BLINK_LESS_10_PERCENT;
        }
        else if (GbaControler.FuelGauge->SoC > 31)
        {
            time_interval = OFF_LED_RP2040;
            gpio_put(LED_RP2040, true);
        }
    }
    else if (GbaControler.MaximCharger.StatusMAX == REVERSE_BOOST)
    {
        time_interval = BLINK_REVERSE_BOOST;
    }
    else if (GbaControler.MaximCharger.StatusMAX == CHARGING)
    {
        time_interval = OFF_LED_RP2040;
        gpio_put(LED_RP2040, true);
    }
    else if ((GbaControler.MaximCharger.StatusMAX == DISCHARGING && GbaControler.FuelGauge->Voltage < SHUTDOWN_VOLTAGE_MV) || GbaControler.FuelGauge->SoC <= 5)
    {
        // No hay batería, encendemos el LED y procedemos con el apagado del dispositivo.
        time_interval = ON_PERMANENT_LED;
        //! Enviamos el comando de apagado a la raspberry pi
        TimeToShutdown = to_ms_since_boot(get_absolute_time()) + SHUTDOWN_WAIT_PI_TIME;
        GbaControler.PiZero2W.RunState = PI_SHUTDOWN;
        return true;
    }

    return false;
}

void GbaHardware::PrintStructValue()
{
    debug("################################################\r\n");
    debug("Fuel Gauge: \tCapacity %dmAh \t\tSOC %f% \tRemCapacity %fmAh \tVoltage %fmv \tCurrent %fmA \tBatTemp %fºC \tCycles %d\r\n",
          GbaControler.FuelGauge->FullRepCap, GbaControler.FuelGauge->SoC, GbaControler.FuelGauge->RemainingCapacity, GbaControler.FuelGauge->Voltage,
          GbaControler.FuelGauge->Current, GbaControler.FuelGauge->Temp, GbaControler.FuelGauge->Cycles);
    debug("Pizero2w: \tRunState %d  Led time %dms\r\n", GbaControler.PiZero2W.RunState, time_interval);
    debug("Backlight: \tFrequency %fKhz \t\tDutyCycle %d \t\tState %d\r\n", GbaControler.Backlight.PWM_Instance->getActualFreq(), GbaControler.Backlight.DutyCycle, GbaControler.Backlight.State);
    debug("MaximCharger: \tStatusMax %d \t\t\tEnbstOutput %d\r\n", GbaControler.MaximCharger.StatusMAX, GbaControler.MaximCharger.EnbstOutput);
    debug("StatusMachine: \tProcess %d\r\n", GbaControler.StatusMachineHardware);
    debug("Volume: \t\tADCvalue %fmV \tValueVolume %d\r\n", GbaControler.AudioControl.PotValuemV, (int)GbaControler.AudioControl.VolumeValueDesired * 3);
    debug("################################################--\r\n");
}

void GbaHardware::UpdateBacklight()
{

    if (GbaControler.Backlight.State == WLED_CHANGE_DC_UP)
    {
        if (GbaControler.Backlight.DutyCycle >= 95)
            GbaControler.Backlight.DutyCycle = 95;
        else
            GbaControler.Backlight.DutyCycle += 5;
    }
    else if (GbaControler.Backlight.State == WLED_CHANGE_DC_DOWN)
    {
        if (GbaControler.Backlight.DutyCycle <= 5)
            GbaControler.Backlight.DutyCycle = 0;
        else
            GbaControler.Backlight.DutyCycle -= 5;
    }

    //! Sección de regulación backlight
    // Esta tarea no se puede llamar de manera continuada, se cambiará solo cada 750ms. Si se pulsa varias veces se acumulará y se hará un incremento de mayor intensidad de golpe.

    if (((GbaControler.Backlight.State == WLED_CHANGE_DC_DOWN) || (GbaControler.Backlight.State == WLED_CHANGE_DC_UP)) && (to_ms_since_boot(get_absolute_time()) > timer_wled_update))
    {
        timer_wled_update = to_ms_since_boot(get_absolute_time()) + TIME_TO_NEXT_UPDATE_BACKLIGTH;
        GbaControler.Backlight.PWM_Instance->setPWM(WLED_LCD, FREQ_WLED_LCD, GbaControler.Backlight.DutyCycle);
        GbaControler.Backlight.State = WLED_RUNING;

        UpdateMedia = true;
    }

    //! Sección de control ON/OFF
    // Control del encendido y apagado del Backlight.
    if (GbaControler.Backlight.State == WLED_STOPED && GbaControler.Backlight.PWM_Instance->get_enable_disable_status() == true)
    {
        GbaControler.Backlight.PWM_Instance->setPWM(WLED_LCD, FREQ_WLED_LCD, 0);
        GbaControler.Backlight.PWM_Instance->disablePWM();
    }
    else if (GbaControler.Backlight.State == WLED_STARTING && GbaControler.Backlight.PWM_Instance->get_enable_disable_status() == false && to_ms_since_boot(get_absolute_time()) > TIME_TO_DELAY_START_DISPLAY)
    {
        GbaControler.Backlight.PWM_Instance->setPWM(WLED_LCD, FREQ_WLED_LCD, GbaControler.Backlight.DutyCycle);
        GbaControler.Backlight.PWM_Instance->enablePWM();
        GbaControler.Backlight.State = WLED_RUNING;
    }
}

void GbaHardware::VolumeTask()
{

    //! Rango máximo 31 posiciones, por lo tanto 3,22% de subida cada uno, ~3%. Maximo valor 96%.
    GbaControler.AudioControl.LastPotValuemV = GbaControler.AudioControl.PotValuemV;
    GbaControler.AudioControl.PotValuemV = adc_to_voltage_mv(adc_read());

    if ((GbaControler.AudioControl.PotValuemV > GbaControler.AudioControl.LastPotValuemV + POT_DEAD_ZONE_MV) || (GbaControler.AudioControl.PotValuemV < GbaControler.AudioControl.LastPotValuemV - POT_DEAD_ZONE_MV))
    {
        //! Si tenemos una variación en el potenciómetro ajustamos el volumen.
        // Ahora convertimos el valor leido de 0 a 3,15mV de 0x00 a 0x22, donde 0x00 es 0% y 0x1F+1 es 93%.
        GbaControler.AudioControl.LastVolumeValueDesired = GbaControler.AudioControl.VolumeValueDesired;
        GbaControler.AudioControl.VolumeValueDesired = (uint16_t)map(GbaControler.AudioControl.PotValuemV, MIN_POT_READ, MAX_POT_READ, MINIMUN_REGISTER_VALUE, MAX_REGISTER_VALUE);
        debug("Obtenemos de Potenciometro %fmV y valor en Hex %#04x\r\n", GbaControler.AudioControl.PotValuemV, GbaControler.AudioControl.VolumeValueDesired);

        // No sabemos cuanto baja el HID del texas, por lo que... o vamos por comando o vamos mediante el HID del TI
        // TODO: Test solucion comando consumer HID y por texas

        //? Solución comando.

        char str[40];

        if (GbaControler.AudioControl.VolumeValueDesired > GbaControler.AudioControl.LastVolumeValueDesired ||
            GbaControler.AudioControl.VolumeValueDesired < GbaControler.AudioControl.LastVolumeValueDesired)
        {
            //! No se si el comando es correcto¡
            snprintf(str, sizeof(str), "sudo amixer -c 1 set PCM %d%%\r\n", (int)GbaControler.AudioControl.VolumeValueDesired * 3);
            command("%s", str);
        }

        //? Funciona con el digital, el tema es que, si somos muy rápidos avanza más la rueda que el HID task, habría que pensar una forma de que veamos el total e invoquemos X veces el HID CONSUMER.
        // Añadimos parte del digital

        if (GbaControler.AudioControl.VolumeValueDesired > GbaControler.AudioControl.LastVolumeValueDesired)
        {
            GbaControler.AudioControl.MD_Status = MD_VOLUME_DOWN;
            UpdateMedia = 1;
        }
        else if (GbaControler.AudioControl.VolumeValueDesired < GbaControler.AudioControl.LastVolumeValueDesired)
        {
            GbaControler.AudioControl.MD_Status = MD_VOLUME_UP;
            UpdateMedia = 1;
        }
    }
}

void GbaHardware::UpdateRPIState()
{

    return;

    //! TODO No hay lectura de led, hay que preparar un servicio que responda a un ACK.
}

void GbaHardware::CommandExecution(uint8_t *buffer, size_t len)
{
    // IF received "[PI]Bootloader", reiniciamos.

    // Pop back up as an MSD drive
    if (len <= 1)
        return; // No podemos hacer nada si enviamos basura.

    if (!strncmp((char *)buffer, REBOOT_USB, len - 1))
    {
        debug("----> Entrando en modo bootloader ....\r\n");
        reset_usb_boot(0, 0);
    }

    else if (!strncmp((char *)buffer, HALT_NOW, len - 1))
    {
        debug("----> Entrando en ciclo de apagado ....\r\n");
        _shutdown_halt = true; //! Solo ocurre cuando esta en proceso de apagado, y termina el apagado.
    }
}