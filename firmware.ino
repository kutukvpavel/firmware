#include <MHZ19.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(6, 7, 2, 3, 4, 5);
MHZ19 sensor;
bool pc_present = false;

void setup()
{
	Serial.begin(9600);
    lcd.begin(8, 2);
    lcd.print("Init...");
    uint8_t counter = 0xFF;
    while (counter--)
    {
        if (Serial.available())
        {
            pc_present = true;
            break;
        }
        delay(20); //20mS * 255 ~= 5s
    }
    lcd.clear();
    lcd.setCursor(8 - 3, 1);
    lcd.print("ppm");
}

void loop()
{
    static uint8_t inBytes[MHZ19_DATA_LEN];

    if (!pc_present) //Standalone mode: request data
    {
        static Command_Type standalone_req_cmds[] =
        {
            Command_Type::CO2UNLIM,
            Command_Type::CO2LIM
        };
        static uint8_t current_cmd_idx = 0;

        send_req(standalone_req_cmds[current_cmd_idx++ % (sizeof(standalone_req_cmds) / sizeof(standalone_req_cmds[0]))]);
    }

    while (Serial.available() < MHZ19_DATA_LEN);

    Serial.readBytes(inBytes, MHZ19_DATA_LEN);
    if (inBytes[0] != 0xFF)
    {
        while (Serial.available()) Serial.read(); //Resync
    }
    if (inBytes[1] == 0x01) return; //Skip input commands
    if (MHZ19::getCRC(inBytes) != inBytes[8]) return;
    switch (inBytes[1])
    {
        case Command_Type::CO2UNLIM:
            lcd.setCursor(0, 0);
            lcd.print(MHZ19::makeInt(inBytes[4], inBytes[5]));
            break;
        case Command_Type::CO2LIM:
            lcd.setCursor(0, 1);
            lcd.print(MHZ19::makeInt(inBytes[2], inBytes[3]));
            break;
        default: break;
    }
}

void send_req(Command_Type cmd)
{
    static uint8_t buf[MHZ19_DATA_LEN];

    sensor.constructCommand(cmd, 0, buf);
    Serial.write(buf, MHZ19_DATA_LEN);
    Serial.flush();
}