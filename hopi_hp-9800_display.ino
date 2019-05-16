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
#include <Fonts/FreeMonoBold12pt7b.h>

// enable serial output
// #define SEROUT

#define FONT_HEIGHT 22
#define FONT_WIDTH  14

// hard coded for landscape mode 320x240
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define ROTATION 1

// bluetooth module (HC-05) digital pin connections
#define BTRX     4
#define BTTX     5
#define BTSTATUS 6
#define BTPWR    7

// TFT color display pin connections
#define RST      8
#define DC       9
#define CS       10
#define MISO     11
#define MOSI     12
#define CLK      13

// determine pixel offset to center string on screen
#define CENTER(y,str) (max(0,((SCREEN_WIDTH-(strlen(str)*FONT_WIDTH))/2))),y,str
// make fillRect do better job of surrounding text boundary
#define FILLRECT_Y_ADJ (-4)
#define FILLRECT_W_ADJ (1)

// format number string, 3 decimals, or none
#define DBLTOSTR(val) dtostrf((double)val,8,3,hopi.str)
#define I16TOSTR(val) dtostrf((double)val,8,0,hopi.str)

// maximum string length on a line
#define MAX_STR_LEN ((SCREEN_WIDTH+FONT_WIDTH-1)/FONT_WIDTH)

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
    // temporary storage for double->string conversions
    char str[MAX_STR_LEN+1];
} hopi;

// bluetooth module @ 9600,8N1
SoftwareSerial bt(BTRX,BTTX); // RX pin, TX pin

// TFT 2.8" Display, blah, blah...
Adafruit_ILI9341 tft(CS,DC,RST);

void btInit()
{
    // set up the various BT pins
    // power off!
    pinMode(BTPWR, OUTPUT);
    digitalWrite(BTPWR, LOW);
    pinMode(BTSTATUS, INPUT);
    pinMode(BTRX, INPUT);
    pinMode(BTTX, OUTPUT);
    digitalWrite(BTTX, HIGH);
    bt.begin(9600);
    bt.listen();
}

bool checkBluetooth()
{
    // are we connected?
    return (digitalRead(BTSTATUS)!=0);
}
void enableBluetooth()
{
#ifdef SEROUT
    Serial.println("Bluetooth Power: ON");
#endif
    // yeah, cheesy function, but helps readability of code
    digitalWrite(BTPWR, HIGH);
}
void disableBluetooth()
{
#ifdef SEROUT
    Serial.println("Bluetooth Power: OFF");
#endif
    // readability.. yeah... that's it!
    digitalWrite(BTPWR, LOW);
}
void invalidateHopiData()
{
#ifdef SEROUT
    Serial.println("Invalidate Hopi Data");
#endif
    // invalidate hopi data
    memset(&hopi,0xff,sizeof(hopi));
}

void XYString(int x, int y, char *str, int c=0, int b=0)
{
    int16_t x1,y1;
    uint16_t w,h;
    // optional color parameters
    // c==0 uses current color
    if (c!=0) {
        // color was specified?
        tft.setTextColor(c);
    }
    if (b==0) {
        // if background not specified, use black
        // yes, I know that BLACK (0x0000!) is a weird corner case here
        b=ILI9341_BLACK;
    }
    // allow for easy line choices, 0-9 = lines, 10-239 = pixels
    if (y<10) {
        y=y*FONT_HEIGHT;
    }
    // compensate for varying baselines
    y+=FONT_HEIGHT;
    // determine size of string as pixel rectangle
    tft.getTextBounds(str,x,y,&x1,&y1,&w,&h);
    // blank that rectangle with background color
    tft.fillRect(x1,y1+FILLRECT_Y_ADJ,w+FILLRECT_W_ADJ,FONT_HEIGHT,b);
    // display string at x,y position
    tft.setCursor(x,y);
    tft.print(str);
}

void screenClear(unsigned int color=ILI9341_BLACK)
{
    tft.fillScreen(color);
    tft.setCursor(0,0);
}

void screenInit()
{
    tft.begin();
    tft.setRotation(ROTATION);
    tft.setFont(&FreeMonoBold12pt7b);
    screenClear();
}

double decode_float_dcba(uint8_t* response,int offset)
{
    union {
        double dbl;
        uint8_t b[4];
    } u;
    // compensate for raw data in reponse
    offset=(offset*2)+3;
    // rearrange bytes to convert from network format (big-endian) to double
    u.b[3]=response[offset+3];
    u.b[2]=response[offset+2];
    u.b[1]=response[offset+1];
    u.b[0]=response[offset+0];
    // a couple sanity checks - no negative values
    if (u.dbl<0.0f) { u.dbl=0.0f; }
    // if you've got more than 9999.999 AMPS flowing, congratulations!
    if (u.dbl>9999.999f) { u.dbl=9999.999f; }
    return u.dbl;
}
uint16_t decode_uint16_ba(uint8_t* response,int offset)
{
    // compensate for raw data in response
    offset=(offset*2)+3;
    unsigned int high=response[offset+0];
    unsigned int low= response[offset+1];
    unsigned int val=(low<<8)+high;
    // no sanity checks needed, it's unsigned, and short int (65535!)
    return val;
}
void serialHex(uint8_t h)
{
#ifdef SEROUT
    // dumb arduino HEX output doesn't pad to 2 bytes, or give a precision knob
    if (h<0x10) { Serial.print("0"); }
    Serial.print(h,HEX);
#endif
}

void showAlive()
{
    static bool spinnerTick=false;

    // 'alive' indicator, toggles dots in corner
    tft.fillRect(SCREEN_WIDTH-5,0,5,5,(spinnerTick)?ILI9341_RED:ILI9341_GREEN);
    spinnerTick=!spinnerTick;
}

void updateModbusValues()
{
    // command string to send to Hopi via BT
    // I assume device has ID 1, rather unlikely your Hopi will be different.
    // If you change the Hopi ID, you can change this to match (it *is*
    // possible to change, in case you desire to chain many of these on the
    // same bus.  Uh huh... )
    uint8_t request[8]={
        0x01,       // query device #1
        0x03,       // request holding registers
        0x00,0x00,  // from 0x0000
        0x00,0x14,  // for 0x0014 registers (20 words, 40 bytes)
        0x45,0xc5   // CRC checksum
    };
    // space to hold the response received
    // max MODBUS response length from hopi
    // short because we're not expecting much back
#define MAX_RESPONSE_BYTES 50
#define RESPONSE_LEN 45
    uint8_t response[MAX_RESPONSE_BYTES];
    static unsigned long nextpoll=0;

    // if no bluetooth connection, complain
    if (!checkBluetooth()) {
#ifdef SEROUT
        Serial.println("Bluetooth Connection Lost");
#endif
        // Ha! Windows Solution: Turn it off and back on!
        disableBluetooth();
        screenClear(ILI9341_RED);
        XYString(CENTER(3,(char*)"Lost"),ILI9341_BLUE,ILI9341_RED);
        XYString(CENTER(4,(char*)"Bluetooth"),ILI9341_BLUE,ILI9341_RED);
        XYString(CENTER(5,(char*)"Connection"),ILI9341_BLUE,ILI9341_RED);
        delay(200);
        enableBluetooth();
        while (!checkBluetooth()) {
            // slow loop down while it waits for reconnection
            delay(200);
            showAlive();
        }
        // bluetooth has returned
#ifdef SEROUT
        Serial.println("Bluetooth Connection Restored");
#endif
        screenClear();
        // make display redraw everything
        invalidateHopiData();
        // force poll to occur
        nextpoll=millis();
    }

#define NEXTMILLIS 150

    // only poll once in a while
    if (millis()<nextpoll) { return; }

    nextpoll=millis()+NEXTMILLIS;

    // force 3.5ms inter-frame delay
    delay(4);
    // send the MODBUS request
    bt.write((uint8_t*)request,sizeof(request));

    // and receive the reply.
    bt.setTimeout(5);
    // min function to prevent me making stupid errors and buffer overruns
    int cnt=bt.readBytes((char*)response,min(RESPONSE_LEN,MAX_RESPONSE_BYTES));
#ifdef SEROUT
    Serial.print(cnt);
    Serial.print(": ");
    for (int t=0; t<cnt; t++) {
        serialHex(response[t]);
    }
    Serial.println();
#endif

    // nothing received?  try again later
    if (cnt<1) { return; }


    // no, I'm not checking for a proper CRC or anything, winging it!
    // but we'll at least look for proper response length via byte 2

    // did we get packet with 40 bytes of data?
    if (response[2]!=40) { return; }

    // voltage still near zero?  try again
    if (decode_float_dcba(response,VOLTAGE)<1.0f) { return; }

    hopi.current=    decode_float_dcba(response,CURRENT);
    hopi.voltage=    decode_float_dcba(response,VOLTAGE);
    hopi.power=      decode_float_dcba(response,POWER);
    hopi.freq=       decode_float_dcba(response,FREQ);
    hopi.pfactor=    decode_float_dcba(response,PFACT);
    hopi.annual=     decode_float_dcba(response,ANNUAL);
    hopi.active=     decode_float_dcba(response,ACTIVE);
    hopi.reactive=   decode_float_dcba(response,REACTIVE);
    // load_time is stored in minutes, convert to hours
    hopi.load_time=  decode_float_dcba(response,LOAD_TIME)/60.0f;
    hopi.work_hours= decode_uint16_ba(response,WORK_HOURS);
    hopi.devaddr=    decode_uint16_ba(response,DEVADDR);
}

void waitForBluetooth()
{
    // turn on BT module
    enableBluetooth();

#ifdef SEROUT
    Serial.print("Connecting to Bluetooth: ");
#endif
    XYString(CENTER(9,(char*)"Wait for Bluetooth"),ILI9341_BLUE);

    // slow loop down a bit while we wait
    while (!checkBluetooth()) {
        delay(200);
        showAlive();
    }

#ifdef SEROUT
    Serial.println("Linked");
#endif
}

void splashScreen()
{
#ifdef SEROUT
    Serial.print("Splash Screen: ");
#endif
    screenClear();
    tft.drawRect(0,0,SCREEN_WIDTH,SCREEN_HEIGHT,ILI9341_RED);
    tft.drawRect(2,2,SCREEN_WIDTH-4,SCREEN_HEIGHT-4,ILI9341_GREEN);
    XYString(CENTER(0,(char*)"Hopi HP-9800"),ILI9341_RED);
    XYString(CENTER(1,(char*)"Bluetooth Display"),ILI9341_BLUE);
    XYString(CENTER(4,(char*)VERSION),ILI9341_GREEN);
    XYString(CENTER(7,(char*)"lornix@lornix.com"),ILI9341_ORANGE);
    delay(750);
#ifdef SEROUT
    Serial.println("Splashed");
#endif
}

void updateValue(int y,double val,int vcolor,char* str,int scolor)
{
    // to align values
#define VALUECOL 32
#define LEGENDCOL 150

    XYString(VALUECOL,y,DBLTOSTR(val),vcolor);
    XYString(LEGENDCOL,y,str,scolor);
}
void updateDisplay()
{
    static unsigned long updateTick=0;

    // loop speed, very coarse
#define UPDATEMILLIS 900

    // full loop takes about 300ms (ugh! slow!  SPI bus?)
    if (millis()<updateTick) { return; }

    updateValue(0, hopi.power,   ILI9341_GREEN, (char*)"Watts", ILI9341_BLUE);
    updateValue(1, hopi.current, ILI9341_GREEN, (char*)"Amps",  ILI9341_BLUE);
    updateValue(2, hopi.voltage, ILI9341_GREEN, (char*)"Volts", ILI9341_BLUE);
    updateValue(3, hopi.pfactor, ILI9341_GREEN, (char*)"pfact", ILI9341_BLUE);
    updateValue(4, hopi.freq,    ILI9341_GREEN, (char*)"Hz",    ILI9341_BLUE);
    updateValue(5, hopi.annual,  ILI9341_GREEN, (char*)"KWH",   ILI9341_BLUE);

    updateTick=millis()+UPDATEMILLIS;
}

void setup()
{
#ifdef SEROUT
    Serial.begin(9600);

    Serial.println("\n\rStarting Hopi HP-9800 Bluetooth display");
#endif

    btInit();

    screenInit();

    splashScreen();

    // turns bluetooth on
    waitForBluetooth();

    screenClear();

    invalidateHopiData();
}

void loop()
{
    // update Hopi values from bluetooth
    updateModbusValues();

    // redraw screen
    updateDisplay();

    // twiddle bits to indicate we're still alive
    showAlive();
}
