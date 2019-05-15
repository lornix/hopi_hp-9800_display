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

// max MODBUS response length from hopi
#define MAX_RESPONSE_BYTES 256

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
// s_xxx is saved value for display flicker reduction
struct hopi {
    // Power being used, Current * Voltage basically
    double power, s_power;
    // Current being drawn
    double current, s_current;
    // Voltage at input
    double voltage, s_voltage;
    // Frequency of source, 50 or 60 Hz usually
    double freq, s_freq;
    // Power Factor, 1.0 = pure resistive
    double pfactor, s_pfactor;
    // total predicted annual consumption KWH
    double annual, s_annual;
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

struct {
    // command string to send to BT Hopi
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
    // and space to hold the response received
    uint8_t response[MAX_RESPONSE_BYTES];
} hopi2;

// bluetooth module @ 9600,8N1
SoftwareSerial bt(BTRX,BTTX); // RX pin, TX pin

// TFT 2.8" Display, blah, blah...
Adafruit_ILI9341 tft(CS,DC,RST);

bool checkBluetooth()
{
    // are we connected?
    return (digitalRead(BTSTATUS)!=0);
}
void enableBluetooth()
{
    Serial.println("Bluetooth Power: ON");
    // yeah, cheesy function, but helps readability of code
    pinMode(BTPWR, OUTPUT);
    digitalWrite(BTPWR, HIGH);
}
void disableBluetooth()
{
    Serial.println("Bluetooth Power: OFF");
    // readability.. yeah... that's it!
    pinMode(BTPWR, OUTPUT);
    digitalWrite(BTPWR, LOW);
}
void invalidateHopiData()
{
    Serial.println("Invalidate Hopi Data");
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
        // specified color?
        tft.setTextColor(c);
    }
    if (b==0) {
        // if background not specified, use black
        // yes, I know that BLACK (=0!) is a weird corner case here
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

double decode_float_dcba(int offset)
{
    union {
        double dbl;
        uint8_t b[4];
    } u;
    // compensate for raw data in reponse
    offset=(offset*2)+3;
    // rearrange bytes to convert from network format (big-endian) to double
    u.b[3]=hopi2.response[offset+3];
    u.b[2]=hopi2.response[offset+2];
    u.b[1]=hopi2.response[offset+1];
    u.b[0]=hopi2.response[offset+0];
    // a couple sanity checks - no negative values
    if (u.dbl<0.0f) { u.dbl=0.0f; }
    // if you've got more than 9999.999 AMPS flowing, congratulations!
    if (u.dbl>9999.999f) { u.dbl=9999.999f; }
    return u.dbl;
}
uint16_t decode_uint16_ba(int offset)
{
    // compensate for raw data in response
    offset=(offset*2)+3;
    unsigned int high=hopi2.response[offset+0];
    unsigned int low= hopi2.response[offset+1];
    unsigned int val=(low<<8)+high;
    // no sanity checks needed, it's unsigned, and short int (65535!)
    return val;
}
void serialHex(uint8_t h)
{
    // dumb arduino HEX output doesn't pad to 2 bytes, or give a precision knob
    if (h<0x0f) { Serial.print("0"); }
    Serial.print(h,HEX);
}

void updateModbusValues()
{
    static long lastpoll=0;
    static boolean bluetoothLost=false;
    unsigned int i;

    // if no bluetooth connection, complain
    while (!checkBluetooth()) {
        if (!bluetoothLost) {
            Serial.println("Bluetooth Connection Lost");
            // Ha! Windows Solution: Turn it off and back on!
            disableBluetooth();
            screenClear(ILI9341_RED);
            XYString(CENTER(3,(char*)"Lost"),ILI9341_BLUE,ILI9341_RED);
            XYString(CENTER(4,(char*)"Bluetooth"),ILI9341_BLUE,ILI9341_RED);
            XYString(CENTER(5,(char*)"Connection"),ILI9341_BLUE,ILI9341_RED);
            bluetoothLost=true;
            delay(200);
            enableBluetooth();
        }
        // just slow loop down while it waits for connection
        delay(100);
    }

#define POLLDELAY 250
    // only poll once in a while
    if ((millis()-lastpoll)<POLLDELAY) { return; }

    // did bluetooth go missing and has returned?
    if (bluetoothLost) {
        Serial.println("Bluetooth Connection Restored");
        bluetoothLost=false;
        screenClear();
        // make display redraw everything
        invalidateHopiData();
    }

    // send the MODBUS request
    for (i=0; i<sizeof(hopi2.request); ++i) {
        bt.write(hopi2.request[i]);
    }

    // interframe delay
    delay(1);

    // and receive the reply.
    int cnt;
    long wait=millis();
    while (((millis()-wait)<3000)&&((cnt=bt.available())<45)) { }
    for (int i=0; i<cnt; i++) {
        hopi2.response[i]=bt.read();
    }
    Serial.print(cnt);
    Serial.print(": ");
    for (int t=0; t<cnt; t++) {
        serialHex(hopi2.response[t]);
    }
    Serial.println();

    lastpoll=millis();

    // no, I'm not checking for a proper CRC or anything, winging it!
    // but we'll at least look for proper response length via byte 2

    // did we get packet with 40 bytes of data?
    if (hopi2.response[2]!=40) {
        return;
    }
    // voltage still near zero?  try again
    if (decode_float_dcba(VOLTAGE)<1.0f) {
        return;
    }

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
}

void waitForBluetooth()
{
    // turn on BT module
    enableBluetooth();

    Serial.print("Connecting to Bluetooth: ");
    XYString(CENTER(9,(char*)"Wait for Bluetooth"),ILI9341_BLUE);

    // slow loop down a bit while we wait
    while (!checkBluetooth()) { delay(50); }

    Serial.println("Linked");
}

void splashScreen()
{
    Serial.print("Splash Screen: ");
    screenClear();
    tft.drawRect(0,0,SCREEN_WIDTH,SCREEN_HEIGHT,ILI9341_RED);
    XYString(CENTER(0,(char*)"Hopi HP-9800"),ILI9341_RED);
    XYString(CENTER(1,(char*)"Bluetooth Display"),ILI9341_BLUE);
    XYString(CENTER(4,(char*)VERSION),ILI9341_GREEN);
    XYString(CENTER(7,(char*)"lornix@lornix.com"),ILI9341_ORANGE);
    delay(750);
    Serial.println("Splashed");
}

void setup()
{
    Serial.begin(9600);

    bt.begin(9600);
    bt.listen();

    Serial.println("\n\rStarting Hopi HP-9800bt display");

    // turn off BT module
    disableBluetooth();

    screenInit();

    splashScreen();

    waitForBluetooth();

    screenClear();

    invalidateHopiData();
}

void loop()
{
    static unsigned long updateTick=0;
    static bool spinnerTick=false;

    // to align values
#define VALUECOL 32
#define LEGENDCOL 150

    // loop speed, very coarse
#define MILLISLOOP 200

    // update Hopi values from bluetooth
    updateModbusValues();

    // update after MILLISLOOP ms
    // full loop takes about 300ms (ugh! slow!  SPI bus?)
    if ((millis()-updateTick)>MILLISLOOP) {
        // only update display if value has changed since last time
        if (hopi.power!=hopi.s_power) {
            XYString(VALUECOL,0,DBLTOSTR(hopi.power),   ILI9341_GREEN);
            XYString(LEGENDCOL,0,(char*)"Watts", ILI9341_BLUE);
            hopi.s_power=hopi.power;
        }
        if (hopi.current!=hopi.s_current) {
            XYString(VALUECOL,1,DBLTOSTR(hopi.current),   ILI9341_GREEN);
            XYString(LEGENDCOL,1,(char*)"Amps",  ILI9341_BLUE);
            hopi.s_current=hopi.current;
        }
        if (hopi.voltage!=hopi.s_voltage) {
            XYString(VALUECOL,2,DBLTOSTR(hopi.voltage), ILI9341_GREEN);
            XYString(LEGENDCOL,2,(char*)"Volts", ILI9341_BLUE);
            hopi.s_voltage=hopi.voltage;
        }
        if (hopi.pfactor!=hopi.s_pfactor) {
            XYString(VALUECOL,3,DBLTOSTR(hopi.pfactor), ILI9341_GREEN);
            XYString(LEGENDCOL,3,(char*)"pfact",        ILI9341_BLUE);
            hopi.s_pfactor=hopi.pfactor;
        }
        if (hopi.freq!=hopi.s_freq) {
            XYString(VALUECOL,4,DBLTOSTR(hopi.freq), ILI9341_GREEN);
            XYString(LEGENDCOL,4,(char*)"Hz",        ILI9341_BLUE);
            hopi.s_freq=hopi.freq;
        }
        if (hopi.annual!=hopi.s_annual) {
            XYString(VALUECOL,5,DBLTOSTR(hopi.annual), ILI9341_GREEN);
            XYString(LEGENDCOL,5,(char*)"KW Hr",       ILI9341_BLUE);
            hopi.s_annual=hopi.annual;
        }
        updateTick=millis();
    }

    // 'alive' indicator, toggles dots in upper left corner
    tft.fillRect(SCREEN_WIDTH-5,0,5,5,(spinnerTick)?ILI9341_RED:ILI9341_GREEN);
    spinnerTick=!spinnerTick;

    /*
     * if (Serial.available()) {
     *     bt.write(Serial.read());
     * }
     * if (bt.available()) {
     *     Serial.write(bt.read());
     * }
     */
}
