#include "MHZ19.h"
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

const int ssRX = 9;
const int ssTX = 8;
SoftwareSerial ss(ssRX, ssTX);
LiquidCrystal lcd(6, 7, 2, 3, 4, 5);
MHZ19 sensor;
bool pc_present = false;

void lcd_blankout(uint8_t line = 0xFF);

void setup()
{
    pinMode(13, OUTPUT);
    pinMode(ssRX, INPUT_PULLUP);
    pinMode(ssTX, INPUT);
	Serial.begin(9600);
    lcd.begin(8, 2);
    lcd.clear();
    lcd.print("Init...");
    delay(100);
    uint8_t counter = 0xFF;
    while (counter--)
    {
        if (counter % 25 == 0) PORTB ^= _BV(5);
        if (Serial.available())
        {
            pc_present = true;
            break;
        }
        delay(25); //~= 6s
    }
    PORTB &= ~_BV(5);
    Serial.end();
    lcd.clear();
    delay(100);
    if (pc_present)
    {
        lcd.setCursor(8 - 3, 0);
        lcd.print("^PC");
    }
    else
    {
        pinMode(ssTX, OUTPUT);
    }
    ss.begin(9600);
    lcd.setCursor(8 - 3, 1);
    lcd.print("ppm");
}

void loop()
{
    static uint8_t inBytes[MHZ19_DATA_LEN];
    static int counter;

    if (!pc_present) //Standalone mode: request data
    {
        static Command_Type standalone_req_cmds[] =
        {
            Command_Type::CO2UNLIM,
            Command_Type::CO2LIM
        };
        static uint8_t current_cmd_idx = 0;

        PORTB ^= _BV(5);
        delay(1000);
        send_req(standalone_req_cmds[current_cmd_idx++ % (sizeof(standalone_req_cmds) / sizeof(standalone_req_cmds[0]))]);
        PORTB ^= _BV(5);
    }

    counter = 0;
    while (ss.available() < MHZ19_DATA_LEN)
    {
        const int timeout = 5000;

        if (++counter > timeout) 
        {
            lcd_blankout();
            return;
        }
        delay(1);
    }
    ss.readBytes(inBytes, MHZ19_DATA_LEN);
    if (inBytes[0] != 0xFF)
    {
        lcd_blankout();
        lcd.setCursor(0, 1);
        lcd.print("SYNC");
        while (ss.available()) ss.read(); //Resync
        return;
    }
    if (inBytes[1] == 0x01) return; //Skip input commands
    if (MHZ19::getCRC(inBytes) != inBytes[8])
    {
        lcd_blankout();
        lcd.setCursor(0, 1);
        lcd.print("CRC!");
        return;
    }
    switch (inBytes[1])
    {
        case Command_Type::CO2UNLIM:
            lcd_blankout(0);
            lcd.setCursor(0, 0);
            lcd.print(MHZ19::makeInt(inBytes[4], inBytes[5]));
            break;
        case Command_Type::CO2LIM:
            lcd_blankout(1);
            lcd.setCursor(0, 1);
            lcd.print(MHZ19::makeInt(inBytes[2], inBytes[3]));
            break;
        default:
            lcd_blankout();
            lcd.setCursor(0, 0);
            lcd.print((int)inBytes[1]);
            lcd.setCursor(0, 1);
            lcd.print("CMD!");
        break;
    }
}

void send_req(Command_Type cmd)
{
    static uint8_t buf[MHZ19_DATA_LEN];

    sensor.constructCommand(cmd, 0, buf);
    ss.write(buf, MHZ19_DATA_LEN);
    ss.flush();
}

void lcd_blankout(uint8_t line = 0xFF)
{
    if (line == 0 || line == 0xFF)
    {
        lcd.setCursor(0, 0);
        lcd.print("     ");
    }
    if (line == 1 || line == 0xFF)
    {
        lcd.setCursor(0, 1);
        lcd.print("     ");
    }
}