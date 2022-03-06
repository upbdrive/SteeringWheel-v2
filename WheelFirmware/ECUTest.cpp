#include <iostream>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include <unordered_map>
#include "pico/util/queue.h"

#include "MCP2515_nb.h"

#define MISO 16
#define MOSI 19
#define CS   17
#define SCK  18
#define INT  20

//#define SPI spi0

struct ECU_Packet
{
    uint16_t data1 = -1;
    uint16_t data2 = -1;
    uint16_t data3 = -1;
    uint16_t data4 = -1;
};

MCP2515 can = MCP2515();
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

//std::vector< std::unordered_map <uint16_t, ECU_Packet> > queue;
queue_t data_queue;

int value = 0;

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

static bool flag = false;

void poll_CAN()
{
    int active = false;
    
    
    while( true )
    {
        CANPacket packet = CANPacket();
        ECU_Packet p;

        if (can.receivePacket(&packet) == 0) {
            //std::cout << "Received CAN packet!\n";
            //std::cout << "Packet has ID 0x" << packet.getId() << '\n';
            if ( packet.getRtr() )
            {
                std::cout << "Got remote transmit request \n";
                std::cout << "Requested size is " << packet.getDlc() << '\n';
            }
            //for (int i = 0; i < packet.getDlc(); i++) {
			//	std::cout << packet.getData()[i] << " ";    
			//}
            //uint8_t* data = 

            p.data1 = ( packet.getData() [1] << 8 ) | packet.getData()[0];          // I'm sorry. This looks horrible.
            p.data2 = ( packet.getData() [3] << 8 ) | packet.getData()[2];          // But it's the only way I got this working.
            p.data3 = ( packet.getData() [5] << 8 ) | packet.getData()[4];
            p.data4 = ( packet.getData() [7] << 8 ) | packet.getData()[6];
            packets[packet.getId()] = p;
        }

        if( flag == true)
        {
            //std::cout << "Updating";
            
            //packets.at(packet.getId()) = p;
            flag = false;
        }else
        {
           // std::cout << "No update needed \n";
        }
        std::cout.flush();
    }
}

int main()
{
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    stdio_init_all();
    queue_init(&data_queue, sizeof(bool), 10);

    sleep_ms(5000);
    gpio_put(LED_PIN, 1);

    std::cout << "Press Enter to start: " << '\n';
    std::cout.flush();
    while (true)
    {
        char c = getchar_timeout_us(1E3);
        if( c != 255)
            break;
    }
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    
    std::cout << "Initializing...\n";
    std::cout.flush();
    spi_init(spi0, 500000); // Initialise spi0 at 500kHz

    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    gpio_init(CS); // Initialise CS Pin
    gpio_set_dir(CS, GPIO_OUT); // Set CS as output
    gpio_put(CS, 0); // Set CS High to indicate no currect SPI communication

    SPI.setSCK(SCK);   // SCK
    SPI.setTX(MOSI);   // MOSI
    SPI.setRX(MISO);   // MISO
    SPI.setCS(CS);     // CS
    
    
    can.setPins(CS, 20);
    gpio_put(LED_PIN, 1);

    can.setClockFrequency(8e6);
    can.begin(500E3);

    multicore_launch_core1(poll_CAN);
    sleep_ms(1000);

    std::cout << "Ready!\n";
    
    while(true)
    {
        //std::cout << "Passing\n";
        //if(queue_is_empty(&data_queue))
        //{
            //queue_add_blocking(&data_queue, &flag);
            //std::cout << packets.at(0x2000).data1 << " ";
            //std::cout << packets.at(0x2000).data2 << " ";
            //std::cout << packets.at(0x2000).data3 << " ";
            //std::cout << packets.at(0x2000).data4 << '\n';


            //std::cout << queue_get_level(&data_queue) << '\n';
        //}
        

        std::cout << packets.at(0x2000).data1 << " ";
        std::cout << packets.at(0x2000).data2 << " ";
        std::cout << packets.at(0x2000).data3 << " ";
        std::cout << packets.at(0x2000).data4 << '\n';

        flag = false;
        sleep_ms(1000);
        flag = true;
        sleep_ms(100);
        
    }

    return 0;

}