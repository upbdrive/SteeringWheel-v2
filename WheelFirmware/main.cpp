/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iostream>
#include "pico/stdlib.h"
#include <geniePicoDEV.h>
// #include <Adafruit_NeoPixel.h>   TODO: needs manual rewrite
#include <MCP2515_nb.h>          // Library for using CAN Communication
#include "pico/multicore.h"
#include <unordered_map>
#include <chrono>
#include <string>


#define PIN 3
#define NUMPIXELS 16

 // Define pin numbers
#define buttonRightPin  26
#define buttonLeftPin   27 
#define displayResetPin  22
#define displayTransPin  4
#define displayRecvPin   5


MCP2515 mcp2515;
short data[40];
int iters = 0;

// Push Buttons variables
int pageNum = 1;
int inputButtonRight = 0;
int inputButtonLeft = 0;
int buttonState = 0;
int buttonStateLeft = 0;
int buttonStateRight = 0;


Genie genie;

unsigned long long delta;


void checkRightButton() {
    inputButtonRight = gpio_get(buttonRightPin);
    if ((inputButtonRight == true ) && ( pageNum < 4 ) && ( to_ms_since_boot(get_absolute_time()) - delta > 500 )) {
        if (buttonStateRight == 0) {
            pageNum += 1;
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0); // Change to Form i+1
            delta = to_ms_since_boot(get_absolute_time());
            std::cout << "MoveRight ";
            std::cout << pageNum << '\n';
        }
        buttonStateRight = 1;
    } else {
        buttonStateRight = 0;
    }
}

void checkLeftButton() {
    inputButtonLeft = gpio_get(buttonLeftPin);
    if (( inputButtonLeft ) && ( pageNum > 1) && ( to_ms_since_boot(get_absolute_time()) - delta > 500 )) {
        if (buttonStateLeft == 0) {
            pageNum -= 1;
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0); // Change to Form i-1
            delta = to_ms_since_boot(get_absolute_time());
            std::cout << "MoveLeft ";
            std::cout << pageNum << '\n';
        }
        buttonStateLeft = 1;
    } else {
        buttonStateLeft = 0;
    }
}

void showLogo() {
    sleep_ms(500);
    genie.WriteObject(GENIE_OBJ_FORM, 1, 0);
}

void setGear(int cGear) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, cGear);
}

void setSpd(int cSpeed) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, cSpeed);
}

void setRpm(int cRpm) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, cRpm);
    /*switch (cRpm) {
        case 0:
            pixels.show();
            pixels.clear();
            break;
        case 1 ... 625:
            pixels.fill(pixels.Color(250, 0, 0), 0, 1);
            pixels.show();
            pixels.clear();
            break;
        case 626 ... 1250:
            pixels.fill(pixels.Color(250, 0, 0), 0, 2);
            pixels.show();
            pixels.clear();
            break;
        case 1251 ... 1875:
            pixels.fill(pixels.Color(250, 0, 0), 0, 3);
            pixels.show();
            pixels.clear();
            break;
        case 1876 ... 2500:
            pixels.fill(pixels.Color(250, 0, 0), 0, 4);
            pixels.show();
            pixels.clear();
            break;
        case 2501 ... 3125:
            pixels.fill(pixels.Color(250, 0, 0), 0, 5);
            pixels.show();
            pixels.clear();
            break;
        case 3126 ... 3750:
            pixels.fill(pixels.Color(250, 0, 0), 0, 6);
            pixels.show();
            pixels.clear();
            break;
        case 3751 ... 4375:
            pixels.fill(pixels.Color(250, 0, 0), 0, 7);
            pixels.show();
            pixels.clear();
            break;
        case 4376 ... 5000:
            pixels.fill(pixels.Color(250, 250, 250), 0, 8);
            pixels.show();
            pixels.clear();
            break;
        default:
            break;       // TODO: needs rewrite
    }*/
}

void setOilTemp(int oilTemp) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, oilTemp);
}

void setAirTemp(int airTemp) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, airTemp);
}

void setAlarmLED(int fuelPres, float batVolt, int waterTemp) {
    if ((fuelPres < 241) || (fuelPres < 440) || (batVolt < 118) || (waterTemp > 105)) {
        //genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 1);
    } else {
        //genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 0);
    }
}

void setFuelPres(int fuelPres) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, fuelPres);
    if (fuelPres < 241) {
        //genie.WriteObject(GENIE_OBJ_STRINGS, 1, 0);
        //genie.WriteStr(1, "W: fuel pressure < 241 kPa");
    } else if (fuelPres > 440) {
        //genie.WriteStr(1, "W: fuel pressure > 440 kPa");
    } else {
        //genie.WriteObject(GENIE_OBJ_STRINGS, 1, -1);
    }
}

void setOilPres(int oilPres) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, oilPres);
}

void setLambda(int lambda) {
    genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 0x00, lambda);
}

void setMAP(int mapVal) {
    genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 0x01, mapVal);
}

void setFuelConsumption(int fuelCons) {
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, fuelCons);
}

void setBatVoltage(float batVolt) {
    genie.WriteObject(GENIE_OBJ_STRINGS, 11, 0);
    genie.WriteStr(11, batVolt / 10);
    genie.WriteObject(GENIE_OBJ_GAUGE, 0x00, batVolt / 10);
    if (batVolt < 118) {
        genie.WriteObject(GENIE_OBJ_STRINGS, 0, 0);
        genie.WriteStr(0, "W: battery voltage < 11.8");
    } else {
        genie.WriteObject(GENIE_OBJ_STRINGS, 0, -1);
    }
}

void setTPS(int throttlePS) {
    genie.WriteObject(GENIE_OBJ_METER, 0x00, throttlePS);
}

void setWaterTemp(int waterTemp) {
    //genie.WriteObject(GENIE_OBJ_STRINGS, 9, 0);
    genie.WriteStr(9, std::to_string(waterTemp));
    if (waterTemp > 240) {
        genie.WriteObject(GENIE_OBJ_STRINGS, 2, 0);
        //genie.WriteStr(2, "W: water temp > 240");
    } else {
        genie.WriteObject(GENIE_OBJ_STRINGS, 2, -1);
    }
}


// NEW FUNCTIONS / CODE
#define MISO 16
#define MOSI 19
#define CS   17
#define SCK  18
#define INT  20



static bool flag = false;
MCP2515 can = MCP2515();

struct ECU_Packet
{
    int16_t data1 = -1;
    int16_t data2 = -1;
    int16_t data3 = -1;
    int16_t data4 = -1;
};

static std::unordered_map <uint16_t, ECU_Packet> packets =
    {
        {0x2000, ECU_Packet()},
        {0x2001, ECU_Packet()},
        {0x2002, ECU_Packet()},
        {0x2003, ECU_Packet()},
        {0x2004, ECU_Packet()},
        {0x2005, ECU_Packet()},
        {0x2006, ECU_Packet()},
        {0x2007, ECU_Packet()}
    };



bool refresh_display()
{
    for( auto const& [identifier, packet] : packets )
    {
        switch(identifier)
        {
            case 0x2000:
            {
                setRpm(packet.data1);
                setTPS(packet.data2);
                setWaterTemp(packet.data3);
                setAirTemp(packet.data4);
                break;
            }
            case 0x2001:
            {
                setMAP(packet.data1);
                setLambda(packet.data2);
                setSpd(packet.data3 / 10);
                setOilPres(packet.data4);
                break;
            }
            case 0x2002:
            {
                setFuelPres(packet.data1);
                setOilTemp(packet.data2);
                setBatVoltage(packet.data3);
                setFuelConsumption(packet.data4);
                break;
            }
            case 0x2003:
            {
                setGear(packet.data1);
                setFuelConsumption(packet.data4);
                break;
            }

        }
    }
    return true;
}

void poll_CAN()
{
    int active = false;
    CANPacket packet = CANPacket();
    while( true )
    {
        if (can.receivePacket(&packet) == 0) {
            ECU_Packet p;
            if ( packet.getRtr() )
            {
                std::cout << "Got remote transmit request \n";
                std::cout << "Requested size is " << packet.getDlc() << '\n';
            }
            p.data1 = ( packet.getData() [1] << 8 ) | packet.getData()[0];          // I'm sorry. This looks horrible.
            p.data2 = ( packet.getData() [3] << 8 ) | packet.getData()[2];          // But it's the only way I got this working.
            p.data3 = ( packet.getData() [5] << 8 ) | packet.getData()[4];
            p.data4 = ( packet.getData() [7] << 8 ) | packet.getData()[6];
            packets[packet.getId()] = p;
            std::cout << "Got packet: " << p.data1 << '\n';
        }

        std::cout.flush();
    }
}

void primary_loop() {
    
    checkRightButton();
    checkLeftButton();
}

void secondary_loop()
{
    poll_CAN();
}

int main() {
    // SETUP PHASE

    // A NOTE: UART 1 is used for actual communication with other the display,
    // UART 0 is ONLY used for debugging

    stdio_init_all();
    
    sleep_ms(5000);                   //Debug timer: allows you time to open the port on the pc to read the full sequence, otherwise not needed
    genie.Ping(100);
    uart_init(uart1, 38400);
    gpio_set_function(displayTransPin, GPIO_FUNC_UART);
    gpio_set_function(displayRecvPin, GPIO_FUNC_UART);
    std::cout << "Preparing genie\n";
    genie.Begin(uart1);
    //genie.AttachDebugStream(uart0);
    std::cout << "Genie Init done\n";
    std::cout.flush();
    
    gpio_init(displayResetPin);
    gpio_set_dir(displayResetPin, GPIO_OUT);
    gpio_put(displayResetPin, false);
    sleep_ms(100);
    gpio_put(displayResetPin, true);
    sleep_ms(100);
    std::cout << "Display reset\n";std::cout.flush();


    gpio_init(buttonLeftPin);
    gpio_init(buttonRightPin);

    gpio_set_dir(buttonLeftPin, GPIO_IN);
    gpio_set_dir(buttonRightPin, GPIO_IN);

    gpio_pull_down(buttonLeftPin);
    gpio_pull_down(buttonRightPin);
    

    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    spi_init(spi0, 500E3); // Initialise spi0 at 500kHz

    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    gpio_init(CS); // Initialise CS Pin
    gpio_set_dir(CS, GPIO_OUT); // Set CS as output
    gpio_put(CS, true); // Set CS High to indicate no current SPI communication    

    can.setClockFrequency(8e6);
    can.begin(500E3, MISO, MOSI, SCK, CS, spi0);

    multicore_launch_core1(secondary_loop);
    //sleep_ms(1000);

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    std::cout << "Ready to rock!\n";
    genie.WriteObject(GENIE_OBJ_FORM, 1, 0);    // page 0: electric diagnostics
                                                // page 1: main display
                                                // page 2: mechanical diagnostics

    uint32_t refresh_delta = to_ms_since_boot(get_absolute_time());
    while(true)
    {
        primary_loop();
        if(to_ms_since_boot(get_absolute_time()) - refresh_delta  > 500)
        {
            //refresh_display();
            refresh_delta = to_ms_since_boot(get_absolute_time());
        }
    }
    return 0;
}






