/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iostream>
#include "pico/stdlib.h"
#include <TinyGPSPlus.h>
#include <genieArduino.h>
// #include <Adafruit_NeoPixel.h>   TODO: needs manual rewrite
#include <MCP2515_nb.h>          // Library for using CAN Communication

#define PIN 3
#define NUMPIXELS 16

 // Define pin numbers
#define buttonRightPin  20
#define buttonLeftPin   21 
#define displayResetPin  22
#define displayTransPin  4
#define displayRecvPin   5

// CAN BUS communication variables
//struct can_frame canMsg;

MCP2515 mcp2515;
short data[40];
int iters = 0;

int valueRpm, valueTps, valueWaterTemp, valueAirTemp, valueMAP,
    valueLambda, valueSpeed, valueOilPres, valueFuelPres,
    valueOilTemp, valueFuelCons, valueGear, valueBatVolt;

// Push Buttons variables
int pageNum = 1;
int inputButtonRight = 0;
int inputButtonLeft = 0;
int buttonState = 0;
int buttonStateLeft = 0;
int buttonStateRight = 0;

// Location variables
float previousLat = 0.0; // previous latitude that was measured
float previousLng = 0.0; // previous longitude that was measured
const float latFinal1 = 44.377129; // finish line P1 latitude
const float longFinal1 = 26.167671; // finish line P1 longitude
const float latFinal2 = 44.377112; // finish line P2 latitude
const float longFinal2 = 26.167696; // finish line P2 longitude
const float lineCrossingError = 0.01; // measurement error

// Lap time tracking
bool sessionActive = false;
unsigned long sessionTime = 0;
unsigned long bestSessionTime = 0;
unsigned long currentLap = 0;
unsigned long bestLap = 0;
unsigned long lastLap = 0;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a Software Serial port called "myNode" - NodeMCU communication
//SoftwareSerial myNode(5, 6);          TODO: change for regular serial

// Create a AltSoftSerial port called "mySerial" - GPS communication
//AltSoftSerial mySerial;               TODO: change for regular serial

// Create a genie object - 4D Systems Display communication
Genie genie;

// Create a timer object - call function to send data to NodeMCU once every 10 seconds
//SimpleTimer timer;

void checkRightButton() {
    //inputButtonRight = digitalRead(7);
    inputButtonRight = gpio_get(buttonRightPin);
    if(inputButtonRight)
        std::cout << "Right pressed";
    if ((inputButtonRight == true ) && ( pageNum < 4 )) {
        if (buttonStateRight == 0) {
            pageNum += 1;
            std::cout << "Setting page\n";std::cout.flush();
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0); // Change to Form i+1
            std::cout << "Setting page done\n";std::cout.flush();
        }
        buttonStateRight = 1;
    } else {
        buttonStateRight = 0;
    }
}

void checkLeftButton() {
    inputButtonLeft = gpio_get(buttonLeftPin);
    if(inputButtonLeft)
        std::cout << "Left pressed";
    if (( inputButtonLeft ) == true && ( pageNum > 1) ) {
        if (buttonStateLeft == 0) {
            pageNum -= 1;
            genie.WriteObject(GENIE_OBJ_FORM, pageNum, 0); // Change to Form i-1
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
    /*genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, cRpm);
    switch (cRpm) {
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
            break;*/       // TODO: needs rewrite
    //}
}

void setOilTemp(int oilTemp) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, oilTemp);
}

void setAirTemp(int airTemp) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, airTemp);
}

void setAlarmLED(int fuelPres, float batVolt, int waterTemp) {
    if ((fuelPres < 241) || (fuelPres < 440) || (batVolt < 118) || (waterTemp > 105)) {
        genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 1);
    } else {
        genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 0);
    }
}

void setFuelPres(int fuelPres) {
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, fuelPres);
    if (fuelPres < 241) {
        genie.WriteObject(GENIE_OBJ_STRINGS, 1, 0);
        genie.WriteStr(1, "W: fuel pressure < 241 kPa");
    } else if (fuelPres > 440) {
        genie.WriteStr(1, "W: fuel pressure > 440 kPa");
    } else {
        genie.WriteObject(GENIE_OBJ_STRINGS, 1, -1);
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
    genie.WriteObject(GENIE_OBJ_STRINGS, 9, 0);
    genie.WriteStr(9, waterTemp);
    if (waterTemp > 240) {
        genie.WriteObject(GENIE_OBJ_STRINGS, 2, 0);
        genie.WriteStr(2, "W: water temp > 240");
    } else {
        genie.WriteObject(GENIE_OBJ_STRINGS, 2, -1);
    }
}

void sendNode() {
    /*
    myNode.print(valueRpm); myNode.print("A");
    myNode.print(valueTps); myNode.print("B");
    myNode.print(valueWaterTemp); myNode.print("C");
    myNode.print(valueAirTemp); myNode.print("D");
    myNode.print(valueMAP); myNode.print("E");
    myNode.print(valueLambda); myNode.print("F");
    myNode.print(valueSpeed); myNode.print("G");
    myNode.print(valueOilPres); myNode.print("H");
    myNode.print(valueFuelPres); myNode.print("I");
    myNode.print(valueOilTemp); myNode.print("J");
    myNode.print(valueBatVolt); myNode.print("K");
    myNode.print(valueFuelCons); myNode.print("L");
    myNode.print(valueGear); myNode.print("M");
    myNode.print('\n');
    delay(500);                              TODO: this needs deprecation*/ 
}

//Stabilirea formatului cronometrului (00:00:0)
String formatSessionTime(unsigned long sessionTime) {
  unsigned long minutes = sessionTime / 60000;
  unsigned long seconds = (sessionTime / 1000) - ((sessionTime / 60000) * 60);
  unsigned long tenths = (sessionTime / 100) % 10;
  if (seconds < 10) return String(minutes) + ":0" + String(seconds) + ":" + String(tenths);
  else return String(minutes) + ":" + String(seconds) + ":" + String(tenths);
}

bool hasCrossedFinishLine() { // function for checking finish line crossing
    float carLat = gps.location.lat();
    float carLong = gps.location.lng();

    for (int t = 0 ; t <= 1; t++) {
        float finishLineLat = latFinal1 + t * (latFinal2 - latFinal1);
        float finishLineLong = longFinal1 + t * (longFinal2 - longFinal1);

        if ((carLat >= finishLineLat && carLong >= finishLineLong) &&
            (sqrt((carLat - finishLineLat) * (carLat - finishLineLat) +
            (carLong - finishLineLong) * (carLong - finishLineLong)) <=
            lineCrossingError)) {
            return true;
        }
    }
    return false;
}

void updateScreen(unsigned long currentSession, unsigned long bestSession, unsigned long lastLap, int currentLap) {
    genie.WriteObject(GENIE_OBJ_STRINGS, 7, 0);
    genie.WriteStr(7, formatSessionTime(lastLap));

    genie.WriteObject(GENIE_OBJ_STRINGS, 8, 0);
    genie.WriteStr(8, formatSessionTime(currentSession));
  
    genie.WriteObject(GENIE_OBJ_STRINGS, 6, 0);
    genie.WriteStr(6, formatSessionTime(bestSession));
  
    genie.WriteObject(GENIE_OBJ_STRINGS, 10, 0);
    genie.WriteStr(10, currentLap);
}

void getLatLong(float previousLat, float previousLong) {
    genie.WriteObject(GENIE_OBJ_STRINGS, 4, 0);
    genie.WriteStr(4, previousLat, 6);
    genie.WriteObject(GENIE_OBJ_STRINGS, 5, 0);
    genie.WriteStr(5, previousLng, 6);
}

void getCanMessage() {
    CANPacket canMsg = CANPacket();
    if (mcp2515.receivePacket(&canMsg) == 0) {
        int id = canMsg.getId();
        char idx = id & 0x000F;

        for (int i = 0; i < 4; i++) {
            short val = (canMsg.getData()[i * 2 + 1] << 8) + canMsg.getData()[i * 2];
            data[idx * 4 + i] = val;
        }

        iters += 1;
    }

    if (iters % 5 == 0) {
        setRpm(data[0]);
        valueRpm = data[0];

        setTPS(data[1]);
        valueTps = data[1];

        setWaterTemp(data[2]);
        valueWaterTemp = data[2];
    
        setAirTemp(data[3]);
        valueAirTemp = data[3];
    
        setMAP(data[4]);
        valueMAP = data[4];
    
        setLambda(data[5]);
        valueLambda = data[5];
    
        setSpd(data[6]/10);
        valueSpeed = data[6]/10;
    
        setOilPres(data[7]);
        valueOilPres = data[7];
    
        setFuelPres(data[8]);
        valueFuelPres = data[8];
    
        setOilTemp(data[9]);
        valueOilTemp = data[9];
    
        setBatVoltage(data[10]);
        valueBatVolt = data[10];
    
        setFuelConsumption(data[11]);
        valueFuelCons = data[11];
    
        setGear(data[12]);
        valueGear = data[12];
    
        setAlarmLED(valueFuelPres, valueBatVolt, valueWaterTemp);
    }
}

void loop() {
    
    // GPS serial port checking - lap timer
    /*if (uart_is_readable(uart0)) {
        byte data[1];
        uart_read_blocking(uart0, data, 1);
        if (gps.encode(data[0])) {
            if ((previousLat != gps.location.lat()) || (previousLng != gps.location.lng())) {
                if (hasCrossedFinishLine()) {
                //Session starting, this is the first lap
                    if (!sessionActive) {
                        sessionActive = true;
                        sessionTime = millis();
                    } else {
                        currentLap += 1;
                        // best / first lap
                        if ((bestSessionTime > millis() - sessionTime) || (bestSessionTime == 0)) {
                            //session time , first lap -> bestSesstionTime = 0
                            bestSessionTime = millis() - sessionTime;
                            lastLap = bestSessionTime;
                        } else {
                            lastLap = millis() - sessionTime;
                        }
                    }
                    //reset the sessionTime
                    sessionTime = millis();
                }
                previousLat = gps.location.lat();
                previousLng = gps.location.lng();
            }
            getLatLong(previousLat, previousLng);
        }
    }*/

    // 4D Systems LCD Screen serial port checking - user data update

    //if (uart_is_readable(uart1)) {
        //getCanMessage();
        printf("here");
        if (sessionActive) {
            updateScreen(millis() - sessionTime, bestSessionTime, lastLap, currentLap);
        } else {
            updateScreen(0, 0, 0, 0);
            updateScreen(currentLap, bestSessionTime, lastLap, currentLap);
        }
    //}

    //timer.run();
    checkRightButton();
    checkLeftButton();
}


int main() {
    // SETUP PHASE

    // A NOTE: UART 1 is used for actual communication with other the display,
    // UART 0 is ONLY used for debugging

    stdio_init_all();
    //mcp2515.setPins(PIN_SPI0_SS);
    uart_init(uart1, 38400);            //38400
    gpio_set_function(displayTransPin, GPIO_FUNC_UART);
    gpio_set_function(displayRecvPin, GPIO_FUNC_UART);
    std::cout << "Preparing genie\n";
    genie.Begin(Serial2);
    std::cout << "Genie Init done\n";
    std::cout.flush();
    
    gpio_init(displayResetPin);
    gpio_set_dir(displayResetPin, GPIO_OUT);
    gpio_put(displayResetPin, false);
    sleep_ms(1000);
    gpio_put(displayResetPin, true);
    sleep_ms(2500);
    std::cout << "Display reset\n";std::cout.flush();


    gpio_init(buttonLeftPin);
    gpio_init(buttonRightPin);

    gpio_set_dir(buttonLeftPin, GPIO_IN);
    gpio_set_dir(buttonRightPin, GPIO_IN);

    gpio_pull_down(buttonLeftPin);
    gpio_pull_down(buttonRightPin);

    // Show start screen (FORM 0)
    //showLogo();
    //std::cout << "Logo set\n";std::cout.flush();
    
    

    while(true)
    {
        //loop();
        checkLeftButton();
        checkRightButton();

        
    }

    // Start the AltSoftSerial port at the GPS's set baud rate
    //mySerial.begin(9800);
    //uart_init(uart1, 9800);


    // Shift lights
    //pixels.begin();   // TODO: this

    // CAN communication
    /*SPI.begin();
    mcp2515.begin(CAN_500KBPS);
    mcp2515.setClockFrequency(MCP_8MHZ);
    mcp2515.setNormalMode();

    // Initialize the left and right pushbutton pins as inputs
    gpio_init(buttonLeftPin);gpio_init(buttonRightPin);
    gpio_set_dir(buttonRightPin, GPIO_IN);
    gpio_set_dir(buttonLeftPin, GPIO_IN);
    
    sleep_ms(500);

    // Reset display

    

    // Timer for sending data to NodeMCU
    //timer.setInterval(10000, sendNode);
    
    */
    return 0;
}






