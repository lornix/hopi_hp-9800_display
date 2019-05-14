// Using Arduino Pro Mini, 5vdc, 16MHz
//
// HC-05 bluetooth module on softserial pins 4/5
// HC-05 uses pins 4(rx)/5(tx)/6(status)/7(power)
//
// 2.8" TFT color touch screen
// ILI9431 uses pins 8(rst)/9(dc)/10(cs)/11(miso)/12(mosi)/13(clk)
// Required Libraries:
//      Adafruit_ILI9341
//      Adafruit_GFX
//      SoftwareSerial
//      SPI

const char* VERSION="v0.2";

#include <Adafruit_ILI9341.h>
#include <SoftwareSerial.h>
#include "font.h"

#define RST      8
#define DC       9
#define CS       10
#define DIN      11
#define DOUT     12
#define CLK      13

#define BTRX     4
#define BTTX     5
#define BTSTATUS 6
#define BTPWR    7

#define MAX_STR_LEN 14

#define DBLTOSTR(val) dtostrf((double)val,8,3,hopi.str)
#define I16TOSTR(val) dtostrf((double)val,8,0,hopi.str)

// please remember offsets are into 16-bit WORDS!
enum offsets {
    POWER=0,
    CURRENT=2,
    VOLTAGE=4,
    FREQ=6,
    PFACT=8,
    ANNUAL=10,
    ACTIVE=12,
    REACTIVE=14,
    LOAD_TIME=16,
    WORK_HOURS=18,  // 1 word
    DEVADDR=19,     // 1 word
    TOTAL_WORDS
};

// holds all the data fumbled from hopi device
struct hopi {
    // Power being used, Current * Voltage basically
    double power;
    // Current being drawn
    double current;
    // Voltage at input
    double voltage;
    // Frequency of source, 50 or 60 Hz usually
    double freq;
    // Power Factor, 1.0 = pure resistive
    double pfactor;
    // total predicted annual consumption KWH
    double annual;
    // Active Power Consumption
    double active;
    // Reactive Power Consumption
    double reactive;
    // total time with non-zero load (in minutes?! Weird!)
    double load_time;
    // hours per day in use
    uint16_t work_hours;
    // modbus device ID, usually 1
    uint16_t devaddr;
    // raw data from hopi modbus request
    uint16_t raw[TOTAL_WORDS];
    // temporary storage for double->string conversions
    char str[MAX_STR_LEN+1];
} hopi;

SoftwareSerial bt(BTRX,BTTX); // RX pin, TX pin
Adafruit_ILI9341 tft(CS,DC,RST);

bool checkBluetooth()
{
    // are we connected?
    return (digitalRead(BTSTATUS)!=0);
}
void enableBluetooth()
{
    // yeah, cheesy function, but helps readability of code
    pinMode(BTPWR, OUTPUT);
    digitalWrite(BTPWR, HIGH);
}
void disableBluetooth()
{
    // readability.. yeah... that's it!
    pinMode(BTPWR, OUTPUT);
    digitalWrite(BTPWR, LOW);
}

void tftWriteCmd(byte cmd)
{
    // digitalWrite(DC, LOW); //DC pin is low for commands
    // digitalWrite(CS, LOW);
    // shiftOut(DIN, CLK, MSBFIRST, cmd); //transmit serial data
    // digitalWrite(CS, HIGH);
}

void tftWriteData(byte dat)
{
    // digitalWrite(DC, HIGH); //DC pin is high for data
    // digitalWrite(CS, LOW);
    // shiftOut(DIN, CLK, MSBFIRST, dat); //transmit serial data
    // digitalWrite(CS, HIGH);
}

void tftChar(unsigned char character)
{
    // anything not pure ascii, replace with '?'
    if ((character<32)or(character>127)) {
        character='?';
    }
    // Write the 5 bytes for character
    for (int i=0; i<5; i++) {
        tftWriteData(ASCII[character - 0x20][i]);
    }
    // and the single dot spacer
    tftWriteData(0x00);
}

void tftXYString(int x, int y, char *characters)
{
    tftWriteCmd(0x80 | x);
    tftWriteCmd(0x40 | y);
    while (*characters) {
        tftChar(*characters++);
    }
}
void tftInit()
{
    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(ILI9341_BLACK);
    tft.fillScreen(ILI9341_RED);
    tft.fillScreen(ILI9341_GREEN);
    tft.fillScreen(ILI9341_BLUE);
    tft.fillScreen(ILI9341_BLACK);
}

void tftClear()
{
    tft.fillScreen(ILI9341_BLACK);
    // reset 'cursor' to home
    tftWriteCmd(0x80);
    tftWriteCmd(0x40);
}

double decode_float_dcba(int offset)
{
    union {
        double dbl;
        uint8_t b[4];
    } u;
    u.b[3]=(hopi.raw[offset+1]>>8)&0xFF;
    u.b[2]=(hopi.raw[offset+1]>>0)&0xFF;
    u.b[1]=(hopi.raw[offset+0]>>8)&0xFF;
    u.b[0]=(hopi.raw[offset+0]>>0)&0xFF;
    // a couple sanity checks
    if (u.dbl<0.0) { u.dbl=0.0f; }
    // if you've got more than 9999.9 AMPS flowing, congratulations!
    if (u.dbl>9999.9) { u.dbl=9999.9f; }
    return u.dbl;
}
uint16_t decode_uint16_ba(int offset)
{
    unsigned int high=(hopi.raw[offset]>>8)&0xff;
    unsigned int low=(hopi.raw[offset]>>0)&0xff;
    unsigned int val=(low<<8)+high;
    // no sanity checks needed, it's unsigned, and short int (65535!)
    return val;
}

void updateModbusValues()
{
#define MAX_RESPONSE 256
    static long lastpoll=0;
    static boolean tftCleared=false;
    // string to send to BT hopi
    // I assume device ID 1, rather unlikely your Hopi will be different
    // if you know how to change the Hopi, you can change this to match
    uint8_t request[]={
        0x01,       // ask device #1
        0x03,       // request holding registers
        0x00,0x00,  // from 0x0000
        0x00,0x14,  // for 0x0014 registers (20 words, 40 bytes)
        0x45,0xc5   // CRC checksum
    };
    uint8_t response[MAX_RESPONSE];
    unsigned int i;

    // if no bluetooth connection, complain, and return
    while (!checkBluetooth()) {
        if (!tftCleared) {
            // something wrong? drop power to BT module
            disableBluetooth();
            // inverse screen
            tftWriteCmd(0x0D);
            tftClear();
            tftXYString(30,2,(char*)"Lost");
            tftXYString(15,3,(char*)"Bluetooth");
            tftXYString(12,4,(char*)"Connection");
            tftCleared=true;
            Serial.println("Bluetooth Connection Lost");
            enableBluetooth();
        }
        delay(100);
    }
    if (tftCleared) {
        Serial.println("Bluetooth Connection Restored");
        tftCleared=false;
        // normal screen
        tftWriteCmd(0x0C);
        // force a poll to update
        lastpoll=millis();
    }

    // only poll once in a while
    if ((millis()-lastpoll)<100) {
        return;
    }

    // send the MODBUS request
    for (i=0; i<sizeof(request); ++i) {
        bt.write(request[i]);
    }
    // and receive the reply.
    i=0;
    while ((i<MAX_RESPONSE)&&(bt.available()>0)) {
        response[i++]=bt.read();
    }
    // no, I'm not checking for a proper CRC or anything, winging it!

    // copy data to raw storage, skip first 3 bytes (CMD), and last two (CRC)
    memcpy(hopi.raw,response+3,40);

    hopi.current=    decode_float_dcba(CURRENT);
    hopi.voltage=    decode_float_dcba(VOLTAGE);
    hopi.power=      decode_float_dcba(POWER);
    hopi.freq=       decode_float_dcba(FREQ);
    hopi.pfactor=    decode_float_dcba(PFACT);
    hopi.annual=     decode_float_dcba(ANNUAL);
    hopi.active=     decode_float_dcba(ACTIVE);
    hopi.reactive=   decode_float_dcba(REACTIVE);
    // load_time is stored in minutes, convert to hours
    hopi.load_time=  decode_float_dcba(LOAD_TIME)/60.0f;
    hopi.work_hours= decode_uint16_ba(WORK_HOURS);
    hopi.devaddr=    decode_uint16_ba(DEVADDR);

    lastpoll=millis();
}

void waitForBluetooth()
{
    // turn on BT module
    enableBluetooth();

    Serial.print("Waiting for Bluetooth: ");
    tftXYString(0,5,(char*)"Wait Bluetooth");

    while (!checkBluetooth()) {
        delay(50);
    }

    tftXYString(0,5,(char*)"  Connected   ");
    Serial.println("Connected");
}

void splashScreen()
{
    Serial.print("Splash Screen: ");
    tftClear();
    tftXYString(6,0,(char*)"Hopi HP-9800");
    tftXYString(15,1,(char*)"Bluetooth");
    tftXYString(21,2,(char*)"Display");
    tftXYString((MAX_STR_LEN-min(strlen(VERSION),14))*3,3,(char*)VERSION);
    tftXYString(24,4,(char*)"lornix");
    delay(2000);
    Serial.println("Done");
}

void setup()
{
    Serial.begin(9600);
    bt.begin(9600);
    bt.listen();

    Serial.println("\n\rStarting Hopi HP-9800bt display");

    // turn off BT module
    disableBluetooth();

    tftInit();

    splashScreen();

    waitForBluetooth();

    // zero hopi data, start out with all zeros
    memset(&hopi,0,sizeof(hopi));
}

void loop()
{
    static int updateTick=0;

    updateModbusValues();

    updateTick--;
    if (updateTick<0) {
        tftXYString(0,0,DBLTOSTR(hopi.power));   tftXYString(48,0,(char*)" Watts");
        tftXYString(0,1,DBLTOSTR(hopi.current)); tftXYString(48,1,(char*)" Amps ");
        tftXYString(0,2,DBLTOSTR(hopi.voltage)); tftXYString(48,2,(char*)" Volts");
        tftXYString(0,3,DBLTOSTR(hopi.pfactor)); tftXYString(48,3,(char*)" pfact");
        tftXYString(0,4,DBLTOSTR(hopi.freq));    tftXYString(48,4,(char*)" Hz   ");
        tftXYString(0,5,DBLTOSTR(hopi.annual));  tftXYString(48,5,(char*)" KW Hr");
        // only update screen every 'updateTick' loops through here
        updateTick=1000;
    }

    /*
     * if (Serial.available()) {
     *     bt.write(Serial.read());
     * }
     * if (bt.available()) {
     *     Serial.write(bt.read());
     * }
     */
}
