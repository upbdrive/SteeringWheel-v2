#include <iostream>
#include "pico/stdlib.h"
#include <geniePicoDEV.h>
#include <MCP2515_nb.h>
#include "pico/multicore.h"
#include <unordered_map>
#include <chrono>
#include <string>

// Define pin numbers
#define buttonRightPin 27
#define buttonLeftPin 26
#define displayResetPin 28
//#define displayTransPin 17
//#define displayRecvPin 16

#define displayTransPin 4
#define displayRecvPin 5

#define MISO 4
#define MOSI 3
#define CS 5
#define SCK 2

Genie genie;

int pageNum = 1;

unsigned long long button_delta;                // used in sw debouncing

void checkRightButton()
{
    if(gpio_get(buttonRightPin) && pageNum < 4 && time_us_64() - button_delta > 500e3)
    {
        pageNum++;
        genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0);
        std::cout << "Page " << pageNum << '\n';
        button_delta = time_us_64();
    }
}

void checkLeftButton()
{
    if(gpio_get(buttonLeftPin) && pageNum < 4 && time_us_64() - button_delta > 500e3)
    {
        pageNum--;
        genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0);
        std::cout << "Page " << pageNum << '\n';
        button_delta = time_us_64();
    }
}

/*void checkRightButton()
{
    inputButtonRight = gpio_get(buttonRightPin);
    if ((inputButtonRight == true) && (pageNum < 4) && (to_ms_since_boot(get_absolute_time()) - delta > 500))
    {
        if (buttonStateRight == 0)
        {
            pageNum += 1;
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0); // Change to Form i+1
            delta = to_ms_since_boot(get_absolute_time());
            std::cout << "MoveRight ";
            std::cout << pageNum << '\n';
        }
        buttonStateRight = 1;
    }
    else
    {
        buttonStateRight = 0;
    }
}*/

/*void checkLeftButton()
{
    inputButtonLeft = gpio_get(buttonLeftPin);
    if ((inputButtonLeft) && (pageNum > 1) && (to_ms_since_boot(get_absolute_time()) - delta > 500))
    {
        std::cout << "MoveLeft ";
        std::cout << pageNum << '\n';
        if (buttonStateLeft == 0)
        {
            pageNum -= 1;
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0); // Change to Form i-1
            delta = to_ms_since_boot(get_absolute_time());
        }
        buttonStateLeft = 1;
    }
    else
    {
        buttonStateLeft = 0;
    }
}*/

#pragma region GUI_Functions

inline void showLogo()
{
    sleep_ms(500);
    genie.WriteObject(GENIE_OBJ_FORM, 1, 0);
}

inline void setGear(int cGear)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, cGear);
}

inline void setSpd(int cSpeed)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, cSpeed);
}

inline void setOilTemp(int oilTemp)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, oilTemp);
}

inline void setAirTemp(int airTemp)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, airTemp);
}

inline void setOilPres(int oilPres)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, oilPres);
}

inline void setLambda(int lambda)
{
    genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 0x00, lambda);
}

inline void setMAP(int mapVal)
{
    genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 0x01, mapVal);
}

inline void setFuelConsumption(int fuelCons)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, fuelCons);
}

inline void setBatVoltage(float batVolt)
{
    genie.WriteObject(GENIE_OBJ_STRINGS, 11, 0);
    genie.WriteStr(11, batVolt / 10);
    genie.WriteObject(GENIE_OBJ_GAUGE, 0x00, batVolt / 10);
    if (batVolt < 118)
    {
        genie.WriteObject(GENIE_OBJ_STRINGS, 0, 0);
        genie.WriteStr(0, "W: battery voltage < 11.8");
    }
    else
    {
        genie.WriteObject(GENIE_OBJ_STRINGS, 0, -1);
    }
}

inline void setWaterTemp(int waterTemp)
{
    genie.WriteStr(9, std::to_string(waterTemp));
    if (waterTemp > 240)
        genie.WriteObject(GENIE_OBJ_STRINGS, 2, 0);
    else
        genie.WriteObject(GENIE_OBJ_STRINGS, 2, -1);
}

#pragma endregion

MCP2515 can = MCP2515();
struct ECU_Packet
{
    int16_t data[4];
};

static std::unordered_map<uint16_t, ECU_Packet> packets =           // stores the packets constantly thrown onto the CAN bus by the ecu
    {                                                               // |   data1  |  data2  | data3 |  data4  |
        {0x2000, ECU_Packet()},                                     // |    RPM   |   TPS   | water |   air   |
        {0x2001, ECU_Packet()},                                     // |    MAP   | Lambda  |  SPD  | OilPres |
        {0x2002, ECU_Packet()},                                     // | FuelPres | OilTemp |  VTG  | FuelCon |
        {0x2003, ECU_Packet()},                                     // |   Gear   | Advance |  INJ  | FuelCon |
    };

bool refresh_display()                                              // function used to refresh the display
{
    for (auto const &[identifier, packet] : packets)                // parse the packets
    {
        switch (identifier)                                         // and set the required values onto the display
        {
        case 0x2000:
        {
            setWaterTemp(packet.data[3]);
            setAirTemp(packet.data[4]);
            break;
        }
        case 0x2001:
        {
            setMAP(packet.data[1]);
            setLambda(packet.data[2]);
            setSpd(packet.data[3] / 10);
            setOilPres(packet.data[4]);
            break;
        }
        case 0x2002:
        {
            setOilTemp(packet.data[2]);
            setBatVoltage(packet.data[3]);
            setFuelConsumption(packet.data[4]);
            break;
        }
        case 0x2003:
        {
            setGear(packet.data[1]);
            setFuelConsumption(packet.data[4]);
            break;
        }
        }
    }
    return true;
}

void poll_CAN()                                                     // get data from the CAN bus
{
    CANPacket packet = CANPacket();                                 // create a packet

    if (can.receivePacket(&packet) == 0)                            // read into it
    {
        ECU_Packet p;
        uint8_t* data = packet.getData();                           // the packet comes in 8 8-bit parts, which must be 
        for ( uint_fast8_t i = 0 ; i < 8 ; i += 2 )                 // combined into 4 16-bit values, so do that
            p.data[i / 2] = (data[i + 1] << 8) | data[i];

        packets[packet.getId()] = p;                                // finally, assign the packet 
    }

}

void mainloop()                                                     // the main loop
{
    checkRightButton();
    checkLeftButton();
}

void secondary_loop()                                               // the second loop
{
    while (true)
    {
        poll_CAN();
    }
}


int main()
{
#pragma region pin_setup

    stdio_init_all();
    sleep_ms(5000);                       // debug delay, allows you enough time to open a serial monitor. Comment this in production!

#pragma region UART
    gpio_init(displayTransPin);
    gpio_init(displayRecvPin);
    gpio_set_function(displayTransPin, GPIO_FUNC_UART);
    gpio_set_function(displayRecvPin, GPIO_FUNC_UART);
    uart_init(uart1, 115200);
#pragma endregion

#pragma region heartbeat
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#pragma endregion

#pragma region other
    gpio_init(displayResetPin);
    gpio_set_dir(displayResetPin, GPIO_OUT);
#pragma endregion

#pragma region buttons
    gpio_init(buttonLeftPin);
    gpio_init(buttonRightPin);

    gpio_set_dir(buttonLeftPin, GPIO_IN);
    gpio_set_dir(buttonRightPin, GPIO_IN);

    gpio_pull_down(buttonLeftPin);
    gpio_pull_down(buttonRightPin);
#pragma endregion

#pragma region SPI
    spi_init(spi0, 1000000); // Initialise spi0 at 10MHz

    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    gpio_init(CS);              // Initialise CS Pin
    gpio_set_dir(CS, GPIO_OUT); // Set CS as output
#pragma endregion

#pragma endregion

#pragma region genie_setup
    genie.Begin(uart1);                   // begin the genie communication, over the selected uart
    genie.AttachDebugStream(uart0);       // if required, attach the debug stream. Comment this in production!
    gpio_put(displayResetPin, false);
    sleep_ms(100);
    gpio_put(displayResetPin, true);
#pragma endregion

    gpio_put(CS, true); // Set CS High to indicate no current SPI communication
    can.setClockFrequency(8e6);                     // set operating freq of the can module to 8mhz
    can.begin(500E3, MISO, MOSI, SCK, CS, spi0);    // and begin at baudrate of 500kbps

    multicore_launch_core1(secondary_loop);         // launch CAN on the second core

    genie.WriteObject(GENIE_OBJ_FORM, 1, 0);        // move the current page to the first display page

    uint32_t refresh_delta = to_ms_since_boot(get_absolute_time());
    uint32_t heartbeat_timer = to_ms_since_boot(get_absolute_time());
    uint8_t heartbeat = false;


    while (true)
    {
        mainloop();                                                     // do the main loop

        if (to_ms_since_boot(get_absolute_time()) - refresh_delta > 500)    // refresh the display every 500ms
        {
            refresh_display();
        }
        if (to_ms_since_boot(get_absolute_time()) - heartbeat_timer > 1000) // and run a heartbeat every 1000ms
        {
            heartbeat_timer = to_ms_since_boot(get_absolute_time());
            gpio_put(PICO_DEFAULT_LED_PIN, heartbeat);
            heartbeat = !heartbeat;
        }
    }
    return 0;
}
