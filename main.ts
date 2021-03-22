/*
Riven
modified from pxt-servo/servodriver.ts
load dependency
"teddybot": "file:../tb_coda_pca"
*/


//% color="#b58900" weight=10 icon="\uf2db"
namespace teddybot {
    const PCA9685_ADDRESS = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09
    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    // HT16K33 commands
    const HT16K33_ADDRESS = 0x70
    const HT16K33_BLINK_CMD = 0x80
    const HT16K33_BLINK_DISPLAYON = 0x01
    const HT16K33_BLINK_OFF = 0
    const HT16K33_BLINK_2HZ = 1
    const HT16K33_BLINK_1HZ = 2
    const HT16K33_BLINK_HALFHZ = 3
    const HT16K33_CMD_BRIGHTNESS = 0xE0

    export enum Servos {
        S1 = 0x01,
        S2 = 0x02
        // S3 = 0x03,
        // S4 = 0x04,
        // S5 = 0x05,
        // S6 = 0x06,
        // S7 = 0x07,
        // S8 = 0x08
    }

    export enum Motors {
        Left = 0x01,
        Right = 0x02
    }

    export enum Leds {
        Left = 0x01,
        Right = 0x02,
        Both = 0x03
    }


    export enum LineSensor {
        //% block="left"
        Left,
        //% block="right"
        Right
    }



    export enum SonarVersion {
        V1 = 0x1,
        V2 = 0x2
    }

    export enum Turns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }


/**
  * Pre-Defined RGB LED colors
  */
    export enum TBColors {
        //% block=red
        Red = 0xff0000,
        //% block=orange
        Orange = 0xffa500,
        //% block=yellow
        Yellow = 0xffff00,
        //% block=green
        Green = 0x00ff00,
        //% block=blue
        Blue = 0x0000ff,
        //% block=indigo
        Indigo = 0x4b0082,
        //% block=violet
        Violet = 0x8a2be2,
        //% block=purple
        Purple = 0xff00ff,
        //% block=white
        White = 0xffffff,
        //% block=black
        Black = 0x000000
    }

    /**
    * Ping unit for sensor.
    */
     export enum TBPingUnit {
        //% block="cm"
        Centimeters,
        //% block="inches"
        Inches,
        //% block="μs"
        MicroSeconds
    }

    let initialized = false
    let initializedMatrix = false
    let neoStrip: neopixel.Strip;
    let matBuf = pins.createBuffer(17);
    let distanceBuf = 0;

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADDRESS, MODE1, 0x00)
        setFreq(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0);
        }
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        //serial.writeValue("ch", channel)
        //serial.writeValue("on", on)
        //serial.writeValue("off", off)

        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
    }


    function setStepper(index: number, dir: boolean): void {
        if (index == 1) {
            if (dir) {
                setPwm(0, STP_CHA_L, STP_CHA_H);
                setPwm(2, STP_CHB_L, STP_CHB_H);
                setPwm(1, STP_CHC_L, STP_CHC_H);
                setPwm(3, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(3, STP_CHA_L, STP_CHA_H);
                setPwm(1, STP_CHB_L, STP_CHB_H);
                setPwm(2, STP_CHC_L, STP_CHC_H);
                setPwm(0, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(4, STP_CHA_L, STP_CHA_H);
                setPwm(6, STP_CHB_L, STP_CHB_H);
                setPwm(5, STP_CHC_L, STP_CHC_H);
                setPwm(7, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(7, STP_CHA_L, STP_CHA_H);
                setPwm(5, STP_CHB_L, STP_CHB_H);
                setPwm(6, STP_CHC_L, STP_CHC_H);
                setPwm(4, STP_CHD_L, STP_CHD_H);
            }
        }
    }

    function stopMotor(index: number) {
        setPwm((index - 1) * 2, 0, 0);
        setPwm((index - 1) * 2 + 1, 0, 0);
    }

    function matrixInit() {
        i2ccmd(HT16K33_ADDRESS, 0x21);// turn on oscillator
        i2ccmd(HT16K33_ADDRESS, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (0 << 1));
        i2ccmd(HT16K33_ADDRESS, HT16K33_CMD_BRIGHTNESS | 0xF);
    }

    function matrixShow() {
        matBuf[0] = 0x00;
        pins.i2cWriteBuffer(HT16K33_ADDRESS, matBuf);
    }


    // create a neopixel strip if not got one already. Default to brightness 40
    function neo(): neopixel.Strip {
        if (!neoStrip) {
            neoStrip = neopixel.create(DigitalPin.P15, 2, NeoPixelMode.RGB);
            neoStrip.setBrightness(40);
        }
        return neoStrip;
    }

    // update LEDs if _updateMode set to Auto
    function updateLEDs(): void {

        neo().show();
    }


    //% blockId=teddybot_motor_run block="Motor|%index|speed %speed"
    //% weight=85
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory=Motors   
    export function MotorRun(index: Motors, speed: number): void {
        if (!initialized) {
            initPCA9685() // 이 부분 때문에 먼저 켜둔  led가 motor run을 넣으면 꺼지구나!!
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }

        if (index == 0x01) {

            if (speed >= 0) {

                setPwm(3, 0, 4095)
                setPwm(4, 0, 0)

                setPwm(2, 0, speed)

            } else {
                setPwm(3, 0, 0)
                setPwm(4, 0, 4095)

                setPwm(2, 0, -speed)


            }


        }


        if (index == 0x02) {

            if (speed >= 0) {
                setPwm(6, 0, 4095)
                setPwm(7, 0, 0)

                setPwm(5, 0, speed)

            } else {

                setPwm(6, 0, 0)
                setPwm(7, 0, 4095)

                setPwm(5, 0, -speed)


            }

        }


    }




    /**
     * Drive motor(s) forward or reverse.
     * @param leds motor to drive.
     * @param brightness brightness of motor (0 to 255). eg: 180
     */
    //% blockId="LED" block="Light %Leds|led(s) at brightness %brightness"
    //% weight=80    
    //% brightness.min=0 brightness.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory=LEDS   
    export function leds(leds: Leds, brightness: number): void {
        if (!initialized) {
            initPCA9685() // 이 부분 때문에 먼저 켜둔  led가 motor run을 넣으면 꺼지구나!!
        }

        brightness = brightness * 16; // map 255 to 4096
        if (brightness >= 4096) {
            brightness = 4095
        }


        if ((leds == 0x01)) {

            //analog_write(pin: Pin, value: number)

            setPwm(8, 0, brightness)

            //  serial.writeValue("leds1", leds1)



        }

        if ((leds == 0x02)) {
            //  analog_write(2, 0)
            setPwm(9, 0, brightness)

            //  serial.writeValue("leds1", leds1)

        }

        if ((leds == 0x03)) {
            setPwm(8, 0, brightness)

            setPwm(9, 0, brightness)
            // serial.writeValue("leds1", leds1)

        }




    }



    /**
     * Read line sensor.
     * @param sensor Line sensor to read.
     */
    //% blockId="teddybot_read_line" block="%sensor|Front sensor"
    //% weight=85
    //% subcategory=Sensors
    export function readLine(sensor: LineSensor): number {

        if (sensor == LineSensor.Left)
            return pins.digitalReadPin(DigitalPin.P13);
        else
            return pins.digitalReadPin(DigitalPin.P14);


    }

    /**
      * Read light sensor.
      * @param sensor Light sensor to read.
      */
    //% blockId="teddybot_read_light" block="%sensor|Line sensor"
    //% weight=80
    //% subcategory=Sensors
    export function readLight(sensor: LineSensor): number {


        if (sensor == LineSensor.Left)
            return pins.analogReadPin(AnalogPin.P1);
        else
            return pins.analogReadPin(AnalogPin.P2);

    }





    /**
     * Execute two motors at the same time
     * @param motor1 First Motor; eg: M1A, M1B
     * @param speed1 [-255-255] speed of motor; eg: 150, -150
     * @param motor2 Second Motor; eg: M2A, M2B
     * @param speed2 [-255-255] speed of motor; eg: 150, -150
    */
    //% blockId=teddybot_motor_dual block="Motor L|speed %speed1|Motor R|speed %speed2"
    //% weight=84
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255    
    //% speed2.min=-255 speed2.max=255    
    //% motor2.defl=Right
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory=Motors  
    export function MotorRunDual(speed1: number, speed2: number): void {

        if (!initialized) {
            initPCA9685() 
        }
        speed1 = speed1 * 16; // map 255 to 4096
        if (speed1 >= 4096) {
            speed1 = 4095
        }
        if (speed1 <= -4096) {
            speed1 = -4095
        }

        speed2 = speed2 * 16; // map 255 to 4096
        if (speed2 >= 4096) {
            speed2 = 4095
        }
        if (speed2 <= -4096) {
            speed2 = -4095
        }


        if (speed1 >= 0) {

            setPwm(3, 0, 4095)
            setPwm(4, 0, 0)

            setPwm(2, 0, speed1)

        } else {
            setPwm(3, 0, 0)
            setPwm(4, 0, 4095)

            setPwm(2, 0, -speed1)

        }



        if (speed2 >= 0) {
            setPwm(6, 0, 4095)
            setPwm(7, 0, 0)

            setPwm(5, 0, speed2)

        } else {

            setPwm(6, 0, 0)
            setPwm(7, 0, 4095)

            setPwm(5, 0, -speed2)


        }


    }

    /**
     * Execute single motors with delay
     * @param index Motor Index; eg: M1A, M1B, M2A, M2B
     * @param speed [-255-255] speed of motor; eg: 150, -150
     * @param delay seconde delay to stop; eg: 1
    */
    //% blockId=teddybot_motor_rundelay block="Motor|%index|speed %speed|delay %delay|s"
    //% weight=81
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% subcategory=Motors  
    export function MotorRunDelay(index: Motors, speed: number, delay: number): void {
        MotorRun(index, speed);
        basic.pause(delay * 1000);
        MotorRun(index, 0);
    }



    //% blockId=teddybot_stop block="Motor Stop|%index|"
    //% weight=80
    //% subcategory=Motors  
    export function MotorStop(index: Motors): void {
        MotorRun(index, 0);
    }

    //% blockId=teddybot_stop_all block="Motor Stop All"
    //% weight=79
    //% blockGap=50
    //% subcategory=Motors  
    export function MotorStopAll(): void {
        if (!initialized) {
            initPCA9685()
        }
        for (let idx = 1; idx <= 4; idx++) {
            stopMotor(idx);
        }
    }




    /**
      * Show LED changes
      */
    //% blockId="teddybot_neo_show" block="show RGB LED changes"
    //% weight=100
    //% subcategory=RGBLeds
    export function neoShow(): void {
        neo().show();
    }

    /**
      * Sets all LEDs to a given color (range 0-255 for r, g, b).
      * @param rgb RGB color of the LED
      */
    //% blockId="teddybot_neo_set_color" block="set all RGB LEDs to %rgb=tb_colors"
    //% weight=95
    //% subcategory=RGBLeds
    export function neoSetColor(rgb: number) {
        neo().showColor(rgb);
        updateLEDs();
    }

    /**
      * Clear all leds.
      */
    //% blockId="teddybot_neo_clear" block="clear all RGB LEDs"
    //% weight=90
    //% subcategory=RGBLeds
    export function neoClear(): void {
        neo().clear();
        updateLEDs();
    }

    /**
     * Set LED to a given color (range 0-255 for r, g, b).
     *
     * @param ledId position of the LED (0 to 11)
     * @param rgb RGB color of the LED
     */
    //% blockId="teddybot_neo_set_pixel_color" block="set RGB LED at %ledId|to %rgb=tb_colors"
    //% weight=85
    //% subcategory=RGBLeds
    export function neoSetPixelColor(ledId: number, rgb: number): void {
        neo().setPixelColor(ledId, rgb);
        updateLEDs();
    }

    /**
      * Shows a rainbow pattern on all LEDs.
      */
    //% blockId="teddybot_neo_rainbow" block="set RGB led rainbow"
    //% weight=80
    //% subcategory=RGBLeds
    export function neoRainbow(): void {
        neo().showRainbow(1, 360);
        updateLEDs()
    }

    /**
     * Rotate LEDs forward.
     */
    //% blockId="teddybot_neo_rotate" block="rotate RGB LEDs"
    //% weight=75
    //% subcategory=RGBLeds
    export function neoRotate(): void {
        neo().rotate(1);
        updateLEDs()
    }

    /**
     * Shift LEDs forward and clear with zeros.
     */
    //% blockId="teddybot_neo_shift" block="shift RGB LEDs"
    //% weight=70
    //% subcategory=RGBLeds
    export function neoShift(): void {
        neo().shift(1);
        updateLEDs()
    }



    /**
       * Read distance from sonar module connected to accessory connector.
       * @param unit desired conversion unit
       */
    //% blockId="teddybot_sonar" block="read sonar as %unit"
    //% weight=90
    //% subcategory=Sensors
    export function sonar(unit: TBPingUnit): number {
        // send pulse
        let trig = DigitalPin.P8;
        let echo = DigitalPin.P12;
        let maxCmDistance = 500;
        let d = 10;
        pins.setPull(trig, PinPullMode.PullNone);
        for (let x = 0; x < 10; x++) {
            pins.digitalWritePin(trig, 0);
            control.waitMicros(2);
            pins.digitalWritePin(trig, 1);
            control.waitMicros(10);
            pins.digitalWritePin(trig, 0);
            // read pulse
            d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);
            if (d > 0)
                break;
        }
        switch (unit) {
            case TBPingUnit.Centimeters: return Math.round(d / 58);
            case TBPingUnit.Inches: return d / 148;
            default: return d;
        }
    }





    // Advanced blocks

    /**
     * Set the brightness of the LEDs
     * @param brightness a measure of LED brightness in 0-255. eg: 40
     */
    //% blockId="teddybot_neo_brightness" block="set RGB LED brightness %brightness"
    //% brightness.min=0 brightness.max=255
    //% weight=60
    //% advanced=true
    export function neoBrightness(brightness: number): void {
        neo().setBrightness(brightness);
        updateLEDs();
    }

    /**
      * Get numeric value of colour
      *
      * @param color Standard RGB Led Colours
      */
    //% blockId="tb_colors" block=%color
    //% weight=55
    //% advanced=true
    export function TBColours(color: TBColors): number {
        return color;
    }

    /**
      * Convert from RGB values to colour number
      *
      * @param red Red value of the LED (0 to 255)
      * @param green Green value of the LED (0 to 255)
      * @param blue Blue value of the LED (0 to 255)
      */
    //% blockId="teddybot_convertRGB" block="convert from red %red| green %green| blue %blue"
    //% weight=50
    //% advanced=true
    export function convertRGB(r: number, g: number, b: number): number {
        return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
    }


    
    /**
     * Servo Execute
     * @param index Servo Channel; eg: S1
     * @param degree [0-180] degree of servo; eg: 0, 90, 180
    */
    //% blockId=teddybot_servo block="Servo|%index|degree %degree"
    //% weight=100
    //% degree.min=0 degree.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    //% advanced=true
    export function Servo(index: Servos, degree: number): void {
        if (!initialized) {
            initPCA9685()
        }
        // 50hz: 20,000 us
        let v_us = (degree * 1800 / 180 + 600) // 0.6 ~ 2.4
        let value = v_us * 4096 / 20000
        //setPwm(index + 7, 0, value)
        setPwm(index, 0, value)
    }





}