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

#define displayTransPin 17
#define displayRecvPin 16

#define MISO_ECU 4
#define MOSI_ECU 3
#define CS_ECU 5
#define SCK_ECU 2

#define MISO_TEL 12
#define MOSI_TEL 11
#define CS_TEL 13
#define SCK_TEL 10

Genie genie;

int pageNum = 1;

unsigned long long button_delta; // used in sw debouncing

void checkRightButton()
{
    if (gpio_get(buttonRightPin) && time_us_64() - button_delta > 500e3)
    {
        if (pageNum < 2)
        {
            pageNum++;
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0);
            std::cout << "Page " << pageNum << '\n';
            button_delta = time_us_64();
        }else
        {
            pageNum = 0;
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0);
            std::cout << "Page " << pageNum << '\n';
            button_delta = time_us_64();
        }
    }
}

void checkLeftButton()
{
    if (gpio_get(buttonLeftPin) && pageNum > 0 && time_us_64() - button_delta > 500e3)
    {
        pageNum--;
        genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0);
        std::cout << "Page " << pageNum << '\n';
        button_delta = time_us_64();
    }
}

#pragma region GUI_Functions

inline void setGear(int cGear)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, cGear);
}   

inline void setRPM(int cRPM)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, cRPM);
    genie.WriteObject(GENIE_OBJ_IGAUGE, 0, (int)(floor(((double)cRPM / 12500.0f ) * 100)));
}

inline void setSpd(int cSpeed)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, cSpeed);
}

inline void setOilTemp(int oilTemp)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, oilTemp);
}

inline void setAirTemp(int airTemp)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, airTemp);
}

inline void setOilPres(int oilPres)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 10, oilPres);
}

inline void setLambda(int lambda)
{
    //   genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 0x00, lambda);
    // TODO: deprecate!
}

inline void setMAP(int mapVal)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 9, mapVal);
}

inline void setFuelConsumption(int fuelCons)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, fuelCons);
}

inline void setBatVoltage(float batVolt)
{
    genie.WriteObject(GENIE_OBJ_METER, 0, batVolt);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, batVolt);
}

inline void setWaterTemp(int waterTemp)
{
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, waterTemp);
    // todo: alarm
}

#pragma endregion

MCP2515 ecu_can = MCP2515();
MCP2515 tel_can = MCP2515();

struct ECU_Packet
{
    int16_t data[4] = {0, 0, 0, 0};
};

static std::unordered_map<uint16_t, ECU_Packet> packets = // stores the packets constantly thrown onto the CAN bus by the ecu
    {
        // |   data1  |  data2  | data3 |  data4  |
        {0x2000, ECU_Packet()}, // |    RPM   |   TPS   | water |   air   |
        {0x2001, ECU_Packet()}, // |    MAP   | Lambda  |  SPD  | OilPres |
        {0x2002, ECU_Packet()}, // | FuelPres | OilTemp |  VTG  | FuelCon |
        {0x2003, ECU_Packet()}, // |   Gear   | Advance |  INJ  | FuelCon |
};

bool refresh_display() // function used to refresh the display
{
    for (auto const &[identifier, packet] : packets) // parse the packets
    {
        switch (identifier) // and set the required values onto the display
        {
        case 0x2000:
        {
            if(packet.data[0] < 100)
                setRPM(0);
            else
                setRPM(packet.data[0]); 
            setWaterTemp(packet.data[2]);
            setAirTemp(packet.data[3]);
            break;
        }
        case 0x2001:
        {
            setMAP(packet.data[0]);
            setLambda(packet.data[1]);
            //setSpd(packet.data[2] / 10);
            //setOilPres(packet.data[3]);
            break;
        }
        case 0x2002:
        {
            //setOilTemp(packet.data[1]);
            setBatVoltage(packet.data[2]);
            setFuelConsumption(packet.data[3]);
            break;
        }
        case 0x2003:
        {
            //setGear(packet.data[0]);
            //setFuelConsumption(packet.data[3]);
            break;
        }
        }
    }
    return true;
}

void poll_CAN() // get data from the CAN bus
{
    CANPacket packet = CANPacket(); // create a packet

    if (ecu_can.receivePacket(&packet) == 0) // read into it
    {
        ECU_Packet p;
        uint8_t *data = packet.getData();       // the packet comes in 8 8-bit parts, which must be
        for (uint_fast8_t i = 0; i < 8; i += 2) // combined into 4 16-bit values, so do that
            p.data[i / 2] = (data[i + 1] << 8) | data[i];

        packets[packet.getId()] = p; // finally, assign the packet
        std::cout << std::hex << packet.getId() << std::dec << ": ";
        for (int i = 0; i < 4; i++)
        {
            std::cout << p.data[i] << " ";
        }
        std::cout << '\n';
    }
}

void mainloop() // the main loop
{
    // std::cout << "bp1.5\n";
    genie.DoEvents();
    checkRightButton();
    checkLeftButton();
}

void secondary_loop() // the second loop
{
    while (true)
    {
        poll_CAN();
    }
}

void eventHandler()
{
    genieFrame *frame = new genieFrame;
    genie.DequeueEvent(frame);
    delete frame;
}

int main()
{
#pragma region pin_setup

    stdio_init_all();
    sleep_ms(5000); // debug delay, allows you enough time to open a serial monitor. Comment this in production!

#pragma region UART
    gpio_init(displayTransPin);
    gpio_init(displayRecvPin);
    gpio_set_function(displayTransPin, GPIO_FUNC_UART);
    gpio_set_function(displayRecvPin, GPIO_FUNC_UART);
    uart_init(uart0, 38400);
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

    gpio_set_function(MISO_ECU, GPIO_FUNC_SPI);
    gpio_set_function(SCK_ECU, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_ECU, GPIO_FUNC_SPI);

    gpio_init(CS_ECU);              // Initialise CS_ECU Pin
    gpio_set_dir(CS_ECU, GPIO_OUT); // Set CS_ECU as output

    spi_init(spi1, 1000000); // Initialise spi0 at 10MHz

    gpio_set_function(MISO_TEL, GPIO_FUNC_SPI);
    gpio_set_function(SCK_TEL, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_TEL, GPIO_FUNC_SPI);

    gpio_init(CS_TEL);              // Initialise CS_TEL Pin
    gpio_set_dir(CS_TEL, GPIO_OUT); // Set CS_TEL as output
#pragma endregion

#pragma endregion

#pragma region genie_setup
    //genie.AttachDebugStream(uart1);       // if required, attach the debug stream. Comment this in production!
    genie.AttachDebugStream(nullptr);
    gpio_put(displayResetPin, false);
    sleep_ms(100);
    gpio_put(displayResetPin, true);
    while (!genie.Begin(uart0))
    {
        std::cout << "Not ready\n";
        std::cout.flush();
    }
    genie.AttachEventHandler(eventHandler);
    genie.DoEvents();
#pragma endregion

    gpio_put(CS_ECU, true);                                           // Set CS_ECU High to indicate no current SPI communication
    ecu_can.setClockFrequency(8e6);                                   // set operating freq of the can module to 8mhz
    ecu_can.begin(1000E3, MISO_ECU, MOSI_ECU, SCK_ECU, CS_ECU, spi0); // and begin at baudrate of 500kbps

    gpio_put(CS_TEL, true); // Set CS_ECU High to indicate no current SPI communication
    //tel_can.setClockFrequency(8e6);                     // set operating freq of the can module to 8mhz
    // tel_can.begin(500E3, MISO_TEL, MOSI_TEL, SCK_TEL, CS_TEL, spi1);    // and begin at baudrate of 500kbps*/

    multicore_launch_core1(secondary_loop); // launch CAN on the second core

    uint32_t refresh_delta = time_us_32() / 1000;
    uint32_t heartbeat_timer = time_us_32() / 1000;
    uint8_t heartbeat = false;

    genie.SetForm(1);

    while (true)
    {
        mainloop(); // do the main loop

        if (to_ms_since_boot(get_absolute_time()) - refresh_delta > 500) // refresh the display every 500ms
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
