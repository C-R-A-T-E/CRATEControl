//  ================================================================================
//
//  Pneumatic Control
//
//  Switched relays to extend and then retract pneumatic cylinder
//
//  IMPORTANT NOTES
//
//  * Arduino IP Address: 129.2168.2.205
//  * Will trigger on receiving any non-zero lenght UDP packet
//
//  ================================================================================

#include <SPI.h>
#include <Ethernet.h>

//  ====================================================================
//
//  Global #define
//
//  ====================================================================

#define PNEUMATIC_EXTEND_OUTPUT_PIN  2
#define PNEUMATIC_RETRACT_OUTPUT_PIN  3

//  ====================================================================
//
//  Global Types
//
//  ====================================================================

//  ====================================================================
//
//  Global Constants
//
//  ====================================================================

//  ====================================================================
//
//  Global Variables
//
//  ====================================================================

//  Networking systems

EthernetUDP g_udpConnection;

char g_packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming packet,
char g_replyBuffer[] = "acknowledged";       // a string to send back

// Network constants

byte g_mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD };

IPAddress g_ip(192, 168, 2, 205);
const uint32_t k_localPort = 8888;

//  ====================================================================
//
//  Global Methods
//
//  ====================================================================

void setup() 
{
    //  Setup serial output for debugging output
        
    Serial.begin(115200);
    Serial.println("PneumaticControl");

    //  For some ethernet shields, if we don't pull pin 4 high (SD Card deselect)
    //  no networking will work.
    
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    // Setup both relay control pins as output and drive them low to start

    pinMode(PNEUMATIC_EXTEND_OUTPUT_PIN, OUTPUT);
    pinMode(PNEUMATIC_RETRACT_OUTPUT_PIN, OUTPUT);

    digitalWrite(PNEUMATIC_EXTEND_OUTPUT_PIN, LOW);
    digitalWrite(PNEUMATIC_RETRACT_OUTPUT_PIN, LOW);

    // start the network services
      
    Ethernet.begin(g_mac, g_ip);
    g_udpConnection.begin(k_localPort);    

    Serial.print("Connect to: ");
    Serial.println(g_ip);
}

void loop() 
{
    int packetSize = g_udpConnection.parsePacket();
    
    if (packetSize)
    {
        memset(g_packetBuffer, 0, sizeof(g_packetBuffer));
        g_udpConnection.read(g_packetBuffer, UDP_TX_PACKET_MAX_SIZE);    
        processMessage(g_packetBuffer);

        g_udpConnection.beginPacket(g_udpConnection.remoteIP(), g_udpConnection.remotePort());
        g_udpConnection.write(g_replyBuffer);
        g_udpConnection.endPacket();
    }

    delay(10);
}

void processMessage(char* p_message)
{
    Serial.print("process message: ");
    Serial.println(p_message);
    
    Serial.println("Pin 2 -> HIGH");
    digitalWrite(PNEUMATIC_EXTEND_OUTPUT_PIN, HIGH);
    delay(1000);

    Serial.println("Pin 2 -> LOW");
    digitalWrite(PNEUMATIC_EXTEND_OUTPUT_PIN, LOW);
    delay(1000);
    
    Serial.println("Pin 3 -> HIGH");
    digitalWrite(PNEUMATIC_RETRACT_OUTPUT_PIN, HIGH);
    delay(1000);
    
    Serial.println("Pin 3 -> LOW");
    digitalWrite(PNEUMATIC_RETRACT_OUTPUT_PIN, LOW);
}
