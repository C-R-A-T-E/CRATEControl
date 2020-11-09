//  ================================================================================
//
//  Accelerometer Control
//
//  Reads accelerometer analog input and sends via UDP Ethernet to mission control
//
//  IMPORTANT NOTES
//
//  * Arduino IP Address: 129.2168.2.201
//  * Will send data on receiving any non-zero lenght UDP packet
//
//  ================================================================================

#include <SPI.h>
#include <Wire.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <NTPClient.h>
#include <pins_arduino.h>

#include <stdio.h>

//  ====================================================================
//
//  Global #define
//
//  ====================================================================

#define DEBUG_OUTPUT 1
#define VERBOSE_OUTPUT 0

#define ACCEL_0_VREF_PIN A2
#define ACCEL_0_X_PIN A3
#define ACCEL_0_Y_PIN A4
#define ACCEL_0_Z_PIN A5

#define ACCEL_1_VREF_PIN A6
#define ACCEL_1_X_PIN A7
#define ACCEL_1_Y_PIN A8
#define ACCEL_1_Z_PIN A9

//  ====================================================================
//
//  Global Constants
//
//  ====================================================================

const float k_rangeMax = 1023.0f;
const float k_gRange = 400.0f;
const float k_gHalfRange = 200.0f;

const int k_utcMinus7 = -7 * 60 * 60;   //  Pacific DST
const int k_utcMinus8 = -8 * 60 * 60;   //  Pacific Standard

const int k_maxSerialDelay = 10;

//  ====================================================================
//
//  Global Variables
//
//  ====================================================================

//  deltaTime managment

unsigned long g_dT = 0;

//  Networking

byte g_mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress g_ip(192, 168, 2, 201);
uint32_t g_localPort = 8888;

EthernetUDP g_udpConnection;
EthernetServer g_server(80);

NTPClient g_timeClient(g_udpConnection, "192.168.2.16");
//NTPClient g_timeClient(g_udpConnection);

String g_timeDisplay;

float g_accel_0_X = 0.0f;
float g_accel_0_Y = 0.0f;
float g_accel_0_Z = 0.0f;

float g_accel_1_X = 0.0f;
float g_accel_1_Y = 0.0f;
float g_accel_1_Z = 0.0f;

//  ====================================================================
//
//  Global Methods
//
//  ====================================================================

void setup() 
{
    Serial.begin(115200);

    int delayCount = 0;
    
    while (!Serial)
    {
        if (delayCount > k_maxSerialDelay) break;
        delayCount++;
        delay(1);      
    }
    
    Serial.println("AccelerometerControl");

    //  For some ethernet shields, if we don't pull pin 4 high (SD Card deselect)
    //  no networking will work.

    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    //  We are getting analog reference value from the accelerometer    
    //  analogReference(EXTERNAL);

    pinMode(ACCEL_0_VREF_PIN, INPUT);
    pinMode(ACCEL_0_X_PIN, INPUT);
    pinMode(ACCEL_0_Y_PIN, INPUT);
    pinMode(ACCEL_0_Z_PIN, INPUT);

    pinMode(ACCEL_1_VREF_PIN, INPUT);
    pinMode(ACCEL_1_X_PIN, INPUT);
    pinMode(ACCEL_1_Y_PIN, INPUT);
    pinMode(ACCEL_1_Z_PIN, INPUT);
    
    // ==============================================
    // Network Setup
    // ==============================================

    Ethernet.begin(g_mac, g_ip);    
    g_server.begin();

    g_udpConnection.begin(g_localPort);
    
    g_timeClient.begin();
    g_timeClient.setTimeOffset(k_utcMinus7);

 #if DEBUG_OUTPUT
 
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());

 #endif
}

void loop() 
{
    #if DEBUG_OUTPUT
    
        Serial.print("Loop at: ");
        Serial.println(micros());
    
    #endif

    g_timeClient.update();

    updateAccelerometerInputs();
    updateDeltaTime();
    updateTimeDisplay();

    //  Deal with getting HTTP requests for our data
    
    handleHTTPRequests();
}

void updateAccelerometerInputs()
{
    long rawX;
    long rawY;
    long rawZ;
    long Vref;
    float scaleFactor;
    
    rawX = analogRead(ACCEL_0_X_PIN);    
    rawY = analogRead(ACCEL_0_Y_PIN);    
    rawZ = analogRead(ACCEL_0_Z_PIN);
    Vref = (float)analogRead(ACCEL_0_VREF_PIN); 

    if (Vref < 0.0001f && Vref > -0.0001f)
    {
        Serial.println("DEVIDE BY ZERO");        
    }
    else
    {
    
        scaleFactor = k_gRange/((float)Vref);   
    
        g_accel_0_X = scaleFactor * rawX - k_gHalfRange;
        g_accel_0_Y = scaleFactor * rawY - k_gHalfRange;
        g_accel_0_Z = scaleFactor * rawZ - k_gHalfRange;
    
#if VERBOSE_OUTPUT
    
        Serial.println("============================================");
        Serial.println("Accelerometer 0 (red)");
        Serial.print(Vref);
        Serial.print(":      ");
        Serial.print(rawX);
        Serial.print(", ");
        Serial.print(rawY);
        Serial.print(", ");
        Serial.println(rawZ);
    
        Serial.print(g_accel_0_X);
        Serial.print("G, ");
        Serial.print(g_accel_0_Y);
        Serial.print("G, ");
        Serial.print(g_accel_0_Z);
        Serial.println("G");
    
#endif
    }
    
    rawX = analogRead(ACCEL_1_X_PIN);    
    rawY = analogRead(ACCEL_1_Y_PIN);    
    rawZ = analogRead(ACCEL_1_Z_PIN);    
    Vref = analogRead(ACCEL_1_VREF_PIN);

    if (Vref < 0.0001f && Vref > -0.0001f)
    {
        Serial.println("DEVIDE BY ZERO");        
    }
    else
    {
    
        scaleFactor = k_gRange/((float)Vref);   
    
        g_accel_1_X = scaleFactor * rawX - k_gHalfRange;
        g_accel_1_Y = scaleFactor * rawY - k_gHalfRange;
        g_accel_1_Z = scaleFactor * rawZ - k_gHalfRange;
    
#if VERBOSE_OUTPUT
    
        Serial.println("Accelerometer 1 (blue)");
        Serial.print(Vref);
        Serial.print(":      ");
        Serial.print(rawX);
        Serial.print(", ");
        Serial.print(rawY);
        Serial.print(", ");
        Serial.println(rawZ);
    
        Serial.print(g_accel_1_X);
        Serial.print("G, ");
        Serial.print(g_accel_1_Y);
        Serial.print("G, ");
        Serial.print(g_accel_1_Z);
        Serial.println("G");
     
#endif
    }
}

void updateDeltaTime()
{
    static unsigned long prevTime = 0;
    
    g_dT = micros() - prevTime;
    prevTime = micros();
}

void updateTimeDisplay()
{
    static int secCur = 0;
    static unsigned long secMicroseconds = 0;

    g_timeDisplay = g_timeClient.getFormattedTime();

    int secPrev = secCur;
    secCur = g_timeClient.getSeconds();
    if (secPrev != secCur)
    {
        secMicroseconds = 0;
    }
    else
    {
        secMicroseconds += g_dT;
    }

    char stringMilliseconds[5];
    sprintf(stringMilliseconds, ":%04d", secMicroseconds / 100);
    stringMilliseconds[4]=0;
    
    g_timeDisplay.concat(stringMilliseconds);
}

EthernetClient clients[2];
int clientMax = 4;
int clientEnd = 0;

char accel0XString[10];
char accel0YString[10];
char accel0ZString[10];
char accel1XString[10];
char accel1YString[10];
char accel1ZString[10];

byte inputNetworkBufferMax = 64;
char inputNetworkBuffer[64];

byte outputNetworkBufferMax = 256;
char outputNetworkBuffer[256];

void handleHTTPRequests()
{
    // check for any new client connecting
    
    EthernetClient newClient = g_server.accept();

    if (newClient && clientEnd < clientMax) 
    {
        #if DEBUG_OUTPUT
            
        Serial.print("New client: ");
        Serial.println(newClient.remoteIP());
        
        #endif
                
        clients[clientEnd] = newClient;
        clientEnd++;
    }

    //  Early if we don't have any clients
    
    if (clientEnd <= 0)
    {
        return;
    }

    //  Format the output messages

    dtostrf(g_accel_0_X, 6, 2, accel0XString);
    dtostrf(g_accel_0_Y, 6, 2, accel0YString);
    dtostrf(g_accel_0_Z, 6, 2, accel0ZString);
    dtostrf(g_accel_1_X, 6, 2, accel1XString);
    dtostrf(g_accel_1_Y, 6, 2, accel1YString);
    dtostrf(g_accel_1_Z, 6, 2, accel1ZString);

    memset(outputNetworkBuffer, 0, sizeof(outputNetworkBuffer));
    sprintf(outputNetworkBuffer, "{\n\t\"accel0X\" : \"%s\",\n\t\"accel0Y\" : \"%s\",\n\t\"accel0Z\" : \"%s\",\n\t\"accel1X\" : \"%s\",\n\t\"accel1Y\" : \"%s\",\n\t\"accel1Z\" : \"%s\",\n\t\"timestamp\" : \"%s\"\n}\n", 
        accel0XString, accel0YString, accel0ZString, 
        accel1XString, accel1YString, accel1ZString, 
        g_timeDisplay.c_str());                

#if VERBOSE_OUTPUT
    
    Serial.println(outputNetworkBuffer);
 
#endif

    // check for incoming data from all clients

    for (byte i = 0; i < clientEnd; i++) 
    {
        if (clients[i] && clients[i].available()) 
        {
            
            #if DEBUG_OUTPUT
            
                Serial.print("Reveice message at: ");
                Serial.println(micros());
            
            #endif
            
            //  Receive message from client
            
            while (clients[i].available() > 0)
            {
                clients[i].read((uint8_t*)inputNetworkBuffer, inputNetworkBufferMax);
            }
                                                        
            // send response

 #if DEBUG_OUTPUT

            Serial.print("Send Response to: ");
            Serial.println(clients[i].remoteIP());

 #endif
 
            clients[i].println("HTTP/1.1 200");
            clients[i].println("Content-Type: application/json");
            clients[i].print("Content-Length: ");
            clients[i].println(strlen(outputNetworkBuffer));
            clients[i].println("Connection: keep-alive");
            clients[i].println();
            clients[i].write(outputNetworkBuffer, strlen(outputNetworkBuffer));
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
            #if DEBUG_OUTPUT
            
            Serial.println("Stopping connection");
            clients[current].stop();

            #endif

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
