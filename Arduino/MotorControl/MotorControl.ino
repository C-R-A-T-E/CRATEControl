//  ================================================================================
//
//  Motor Control
//
//  Switched relay to turn on and off motor (VFD)
//
//  IMPORTANT NOTES
//
//  * Arduino IP Address: 129.2168.2.206
//  * Will turn on with message "ON", "On" or "on"
//  * Will turn off with ANY other message
//
//  ================================================================================

#include <SPI.h>
#include <Ethernet.h>

//  ====================================================================
//
//  Global #define
//
//  ====================================================================

//  From: https://store.arduino.cc/usa/4-relays-shield
//  Relay 1 = Arduino pin 4 
//  Relay 2 = Arduino pin 7 
//  Relay 3 = Arduino pin 8 
//  Relay 4 = Arduino pin 12

#define MOTOR_RELAY_OUTPUT_PIN  7

//  ====================================================================
//
//  Global Types
//
//  ====================================================================

enum EMotorState
{
    Init,

    Off,
    ReceivedOff,
    On,
    ReceivedOn,

    Invalid = -1
};

//  ====================================================================
//
//  Global Constants
//
//  ====================================================================

const unsigned long k_receiveValidOnMessageTimeout = 5000000;

//  ====================================================================
//
//  Global Variables
//
//  ====================================================================

unsigned long g_dT = 0;

//  Motor State Machine

EMotorState g_motorState = EMotorState::Invalid;
unsigned long g_motorStateEnterTime = 0;


//  Networking systems

EthernetUDP g_udpConnection;

char g_packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming packet,
char g_replyBuffer[] = "acknowledged";       // a string to send back

// Network valuse

byte g_mac[] =  { 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xED };

IPAddress g_ip(192, 168, 2, 206);
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
    Serial.println("Motor Control");

    //  For some ethernet shields, if we don't pull pin 4 high (SD Card deselect)
    //  no networking will work.

    //  BB MJS: The RELAY can not set this high, because this is on of the relay pins
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    
    // Setup both relay control pins as output and drive them low to start

    pinMode(MOTOR_RELAY_OUTPUT_PIN, OUTPUT);
    digitalWrite(MOTOR_RELAY_OUTPUT_PIN, LOW);

    // start the network services
      
    Ethernet.begin(g_mac, g_ip);
    g_udpConnection.begin(k_localPort);    

    Serial.print("Connect to: ");
    Serial.println(g_ip);

    setMotorState(EMotorState::Init);
}

void loop() 
{
    //  Keep track of frame dt

    updateDeltaTime();

    //  Update state machines

    updateMotorState();

    //  Check for message

    handleHTTPRequests();
}

void updateDeltaTime()
{
    static unsigned long prevTime = 0;
    
    g_dT = micros() - prevTime;
    prevTime = micros();
}

void handleHTTPRequests()
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

    if ((p_message[0] == 'O' || p_message[0] == 'o') &&
        (p_message[1] == 'N' || p_message[1] == 'n') &&
        (p_message[2] == 0x00))
    {
        Serial.println("Received ON");
        setMotorState(EMotorState::ReceivedOn);
    }    
    else
    {
        Serial.println("Received OFF");
        setMotorState(EMotorState::ReceivedOff);        
    }
}

//
//  State tracking methods
//

void updateMotorState()
{
    while (true)
    {
        EMotorState prevMotorState = g_motorState;

        switch (g_motorState)
        {
        case EMotorState::Init:
            {
                setMotorState(EMotorState::Off);
            }
            break;

        case EMotorState::ReceivedOff:
            {
                setMotorState(EMotorState::Off);
            }
            break;

        case EMotorState::ReceivedOn:
            {
                setMotorState(EMotorState::On);
            }
            break;

        case EMotorState::On:
            {
                //
                //  !!! SAFETY !!!
                //
                //  TURN MOTOR OFF IF WE DON'T RECEIVE AN "ON" MESSAGE EVERY SO OFFTEN
                //
        
                if (micros() - g_motorStateEnterTime > k_receiveValidOnMessageTimeout)
                {
                    Serial.print(micros());
                    Serial.print(" : ");
                    Serial.println(g_motorStateEnterTime);
                    
                    Serial.println("Safty trigger turn OFF");
                    setMotorState(EMotorState::Off);
                }
            }
            break;
        
        default:
            {
            }
            break;
        }
        
        if (prevMotorState == g_motorState)
        {
            break;
        }
    }
}

void setMotorState(EMotorState p_newMotorState)
{
    if (g_motorState == p_newMotorState)
    {
        return;
    }

    //  Action to take when leaving a state
    
    switch (g_motorState)
    {
    default:
        {
        }
        break;
    }

    //  Set the state
    
    g_motorState = p_newMotorState;
    g_motorStateEnterTime = micros();

    //  Action to take when entering a state
    
    switch (g_motorState)
    {
    case EMotorState::On:
        {
            Serial.println("Turn Motor On");
            digitalWrite(MOTOR_RELAY_OUTPUT_PIN, HIGH);
        }
        break;

    case EMotorState::Off:
        {
            Serial.println("Turn Motor Off");
            digitalWrite(MOTOR_RELAY_OUTPUT_PIN, LOW);
        }
        break;

    default:
        {
        }
        break;
    }
}
