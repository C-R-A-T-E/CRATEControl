//  ================================================================================
//
//  Tach Control
//
//  Reads tachometer analog input (A0) and sends via UDP Ethernet to mission control
//
//  IMPORTANT NOTES
//
//  * Arduino IP Address: 129.2168.2.200
//  * Will send data on receiving any non-zero lenght UDP packet
//
//  ================================================================================

#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <NTPClient.h>

#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#include <stdio.h>

// Network constants

byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x15, 0x32 };
IPAddress ip(192, 168, 2, 200);
unsigned int localPort = 8888;

//  Networking systems

EthernetServer server(80);
EthernetUDP Udp;
//NTPClient timeClient(Udp, "192.168.2.16");
NTPClient timeClient(Udp);

int UTCMinus7 = -7 * 60 * 60;   //  Pacific DST
int UTCMinus8 = -8 * 60 * 60;   //  Pacific Standard

// The 7 segment RPM display

Adafruit_7segment matrix = Adafruit_7segment();

// state values for keeping track of where we are in the rotating blade's cycle
const int state_indeterminate = 0;
const int state_low_A = 1;
const int state_high_A = 2;
const int state_low_B = 3;
const int state_high_B = 4;

int state;
int rpmTest = 0;
String timeDisplay;
int rpmDisplay = 0;

    
// threshold voltage between blocked and unblocked states, initialized for now to a guess value
float vThreshold = 2.725;

// time values
unsigned long t_cycleStart;
unsigned long t_rev;

// 7-segment LED bitmaps used to display CAL0 and CAL1
uint8_t C = 0b00111001;
uint8_t A = 0b01110111;
uint8_t L = 0b00111000;
uint8_t one = 0b00000110;
uint8_t zero = 0b00111111;

// Function that returns current voltage on pin A0 
float v() 
{
    int vInput = analogRead(A0);
    return (vInput * (5.0/1023.0));
}

// Function that samples voltage a bunch of times and returns an average value.
float average_v()
{
    float vbuffer[100]; // buffer used to collect data samples
    int i;
    float total = 0.0;
    
    for (i = 0; i < 100; i++)
    {
        vbuffer[i] = v();
        delay(10);
    }
    
    for (i = 0; i < 100; i++)
    {
        total = total + vbuffer[i];
    }
    
    return (total/100);
}

void setup() 
{
    state = state_indeterminate;

    // ==============================================
    // Hardware Setup
    // ==============================================
    
    //  Setup serial output for debugging output
        
    Serial.begin(115200);
    Serial.println("TachControl");

    // set the red LEDs on the electronics enclosure front panel 
    
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);

    //  For some ethernet shields, if we don't pull pin 4 high (SD Card deselect)
    //  no networking will work.
    
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    // Fire up the 7-segment LED display that shows current RPM
    
    matrix.begin(0x70);
    matrix.blinkRate(1);
    matrix.writeDigitRaw(0, C);
    matrix.writeDigitRaw(1, A);
    matrix.writeDigitRaw(3, L);
    matrix.writeDigitRaw(4, zero);
    matrix.writeDisplay();
    
    // ==============================================
    // Network Setup
    // ==============================================

    Ethernet.begin(mac, ip);    
    server.begin();

    Udp.begin(localPort);
    
    timeClient.begin();
    timeClient.setTimeOffset(UTCMinus7);

    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());

    // ==============================================
    // Calibration
    // ==============================================
    
    // Collect a bunch of samples with the photocell obscured and average them.
    delay(3000);
    Serial.println("collecting data with blade in blocking position...");
    
    float cal0 = average_v();
    
    Serial.println("Average blocked voltage:");
    Serial.println(cal0, DEC);
    
    // Change the red LEDs as a signal that we are entering the second phase of the calibration process
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    matrix.writeDigitRaw(4, one);
    matrix.writeDisplay();
    
    // Collect a bunch of samples with the photocell illuminated and average them.
    delay(3000);
    Serial.println("collecting data with blade in non-blocking position...");
    
    float cal1 = average_v();
    Serial.println("Average unblocked voltage:");
    Serial.println(cal1, DEC);
    
    // If it seems like we got valid data, calculate the threshold that the state machine
    // will use to detect transitions, otherwise stick with the hard coded default value.
    if (cal0 - cal1 >= 1.0)
    {  
        vThreshold = (cal0 + cal1)/2;
        Serial.println("Calculated threshold voltage:");
        Serial.println(vThreshold, DEC);
    } 
    else 
    {
        Serial.println("Calibration failed, using default threshold voltage:");
        Serial.println(vThreshold, DEC);    
    }

    matrix.blinkRate(0);
    matrix.println(0);
    matrix.writeDisplay();
    digitalWrite(3, LOW);  
    
    state = state_low_A;
    t_cycleStart = micros();
}

void loop() 
{
    float rpm;
    bool rpmChanged = false;
    
    static int secCur = 0;
    static int secMilliseconds = 0;
    static int t = 0;
    static int dt = 0;

    int tPrev = t;
    t = millis();
    
    dt = t - tPrev;
    
    timeClient.update();
    timeDisplay = timeClient.getFormattedTime();

    int secPrev = secCur;
    secCur = timeClient.getSeconds();
    if (secPrev != secCur)
    {
        secMilliseconds = 0;
    }
    else
    {
        secMilliseconds += dt;
    }

    char stringMilliseconds[5];
    sprintf(stringMilliseconds, ":%04d", secMilliseconds);
    timeDisplay.concat(stringMilliseconds);
    
    // Check for case where blade isn't moving at all...
    if (micros()- t_cycleStart > 5000000.0)
    {
        // there has been no full cycle in the last 5 seconds so assume blade isn't moving.
        rpm = 0.0;
        rpmChanged = true;
        t_cycleStart = micros();
    
        if (v() > vThreshold)
        {
            state = state_high_A;
        } 
        else 
        {
            state = state_low_A;
        }
    }
    
    // Use simple finite state machine to keep track of RPMs
    
    switch (state)
    {
    case state_low_A:
        if (v() > vThreshold)
        {
            state = state_high_A;
        }
        break;
    
    case state_high_A:
        if (v() < vThreshold)
        {
            state = state_low_B;
        }
        break;
        
    case state_low_B:
        if (v() > vThreshold)
        {
            state = state_high_B;
        }
        break;
        
    case state_high_B:
        if (v() < vThreshold)
        {
            state = state_low_A;
            t_rev = micros()- t_cycleStart;
            t_cycleStart = micros();
            rpm = 60.0*1000000.0/t_rev;
            rpmChanged = true;
        }
        break;
    } // end of switch statement

    if (rpmChanged) 
    {
        Serial.print(timeDisplay);
        Serial.print(": ");
        Serial.println(rpm, DEC);

        rpmDisplay = round(rpm);
        matrix.println(rpmDisplay);
        matrix.writeDisplay();
        rpmChanged = false;
    }  

    handleHTTPRequests();
}

EthernetClient clients[4];
int clientMax = 4;
int clientEnd = 0;

void handleHTTPRequests()
{
    static int debugRPM = 0;
    static int countDirection = 1;

    if (debugRPM > 2000 || debugRPM < 0)
    {
        countDirection *= -1;
    }
    debugRPM += countDirection;

    byte networkBufferMax = 64;
    char networkBuffer[64];
    
    // check for any new client connecting
    
    EthernetClient newClient = server.accept();

    if (newClient && clientEnd < clientMax) 
    {
        Serial.print("New client: ");
        Serial.println(newClient.remoteIP());
        
        clients[clientEnd] = newClient;
        clientEnd++;
    }
    
    // check for incoming data from all clients

    for (byte i = 0; i < clientEnd; i++) 
    {
        if (clients[i] && clients[i].available()) 
        {
            rpmTest++;
            while (clients[i].available() > 0)
            {
                clients[i].read(networkBuffer, networkBufferMax);
            }
                                                        
            // send response
            
            //sprintf(networkBuffer, "{\n\t\"rpm\" : \"%03d\",\n\t\"timestamp\" : \"%s\"\n}\n", rpmDisplay, timeDisplay.c_str());
            sprintf(networkBuffer, "{\n\t\"rpm\" : \"%03d\",\n\t\"timestamp\" : \"%s\"\n}\n", debugRPM, timeDisplay.c_str());
            
            clients[i].println("HTTP/1.1 200");
            clients[i].println("Content-Type: application/json");
            clients[i].print("Content-Length: ");
            clients[i].println(strlen(networkBuffer));
            clients[i].println("Connection: keep-alive");
            clients[i].println();
            clients[i].write(networkBuffer, strlen(networkBuffer));
            clients[i].flush();
            delay(1);
        }
    }

    
    // stop any clients which disconnect

    byte current = 0;

    while (current < clientEnd)
    {
        // if there is a client that is now disconnected, move everyone else down

        if (clients[current] && !clients[current].connected())
        {
            Serial.println("Stopping connection");
            clients[current].stop();

            for (byte j = current; j < clientEnd-1; j++)
            {
                clients[j] = clients[j+1];
            }
            
            clientEnd--;
        }
        else
        {
            current++;
        }
    }
}
