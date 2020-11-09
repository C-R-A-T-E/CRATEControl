//  ================================================================================
//
//  BigRedButtonBoxController
//
//  The controller of all!
//
//  IMPORTANT NOTES
//
//  * Arduino IP Address: 129.2168.2.199
//
//  ================================================================================

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Ethernet.h>
#include <pins_arduino.h>
#include <ArduinoSound.h>

#include <Adafruit_NeoPixel.h>


//  ====================================================================
//
//  Global #define
//
//  ====================================================================

#define LED_COUNT 28

#define NEOPIXEL_OUTPUT_PIN     6
#define SAFTY_SWITCH_INPUT_PIN  7
#define MOTOR_SWITCH_INPUT_PIN 13
#define FIRE_SWITCH_INPUT_PIN  14

#define CHARGING_LED_OUTPUT_PIN 0
#define ETHERNET_CONNECTION_OUTPUT_PIN 1

#define NORMALLY_OPEN HIGH
#define NORMALLY_CLOSED LOW

//  ====================================================================
//
//  Global Types
//
//  ====================================================================

enum EBigRedButtonState
{
    InitBigRedButton,

    Idle,
    Armed,
    Fire,
    CoolDown,

    InvalidBigRedbutton = -1
};

enum EStripState
{
    InitStrip,

    WipeOnStart,
    PowerWipeOnStep,
    SaftyWipeOnStep,
    MotorWipeOnStep,
    WipeOnNext,

    PowerPulse,
    SaftyPulse,
    MotorPulse,

    Off,
    Shutdown,

    InvalidStrip = -1
};

//  ====================================================================
//
//  Global Constants
//
//  ====================================================================

const unsigned long k_bigRedButtonCoolDownDuration = 2000000;
const unsigned long k_onMessageSendFrequency = 4000;

const unsigned long k_wipeOnStepDt = 50000;

const float k_batteryVoltage = 3.3f;

const float k_radiansPerSecond = 0.005f;

const int k_pulseUpdateFrequencyMin = 20000;
const float k_phaseBrightnessMax = 0.5f;
const float k_phaseBrightnessMin = 0.02f;

const uint16_t k_ledHueRed = 0;
const uint16_t k_ledHueYellow = 10922;
const uint16_t k_ledHueGreen = 21845;

const String k_firePneumaticsMessage = "FIRE!";
const String k_turnOnMotorMessage = "On";
const String k_turnOffMotorMessage = "Off";

const int k_maxSerialDelay = 10;
const int k_audioVolume = 35;

const uint32_t k_localPort = 8888;
const uint32_t k_pneumaticsPort = 8888;
const uint32_t k_motorPort = 8888;

//  ====================================================================
//
//  Global Variables
//
//  ====================================================================

//  LED Strip

Adafruit_NeoPixel g_strip(LED_COUNT, NEOPIXEL_OUTPUT_PIN, NEO_GRB + NEO_KHZ800);

//  deltaTime managment

unsigned long g_dT = 0;

//  Networking systems

byte g_mac[] = { 0xAE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAE };
IPAddress g_ip(192, 168, 2, 199);
IPAddress g_ipPneumatics(192, 168, 2, 205);
IPAddress g_ipMotor(192, 168, 2, 206);

EthernetUDP g_udpConnection;

char g_packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char g_replyBuffer[] = "acknowledged";       // a string to send back

//  BigRedButton State Machine

EBigRedButtonState g_bigRedButtonState = EBigRedButtonState::InvalidBigRedbutton;
int g_bigRedButtonStateEnterTime = 0;

//  Switch State

bool g_saftySwitchOn = false;
bool g_motorSwitchOn = false;
bool g_fireSwitchOn = false;

bool g_validSD = true;

unsigned long g_lastOnMessageSent = 0;

//  Strip State Machine

EStripState g_stripState = EStripState::InvalidStrip;
int g_stripStateEnterTime = 0;

int g_wipeOnStep = 0;
float g_pulsePhase = 0.0f;
float g_pulseTheta = PI / 2.0f;

uint16_t g_ledHueCurrent = k_ledHueGreen;

SDWaveFile g_startupSound;
SDWaveFile g_fireSound;

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
    
    Serial.println("BRBB");

    //  For some ethernet shields, if we don't pull pin 4 high (SD Card deselect)
    //  no networking will work.

    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    
    // start the network services

    Ethernet.begin(g_mac, g_ip);
    g_udpConnection.begin(k_localPort);

    if (Ethernet.hardwareStatus() == EthernetNoHardware) 
    {
        Serial.println("Ethernet shield was not found.");
    }
    else if (Ethernet.hardwareStatus() == EthernetW5100) 
    {
        Serial.println("W5100 Ethernet controller detected.");
    }
    else if (Ethernet.hardwareStatus() == EthernetW5200) 
    {
        Serial.println("W5200 Ethernet controller detected.");
    }
    else if (Ethernet.hardwareStatus() == EthernetW5500) 
    {
        Serial.println("W5500 Ethernet controller detected.");
    }

    //  Setup the switch logic

    initSwitches();

    //  Init SD card and Audio that plays from SD card

    g_validSD = true;

    //  Init the SD Card
    
    if (!SD.begin(SDCARD_SS_PIN)) 
    {
        Serial.println("SD Init failed!");

        g_validSD = false;
    }

    Serial.println("SD Init");

    //  Init Arduino Audio

    initAudioFiles();
    AudioOutI2S.volume(k_audioVolume);
      
    //  Setup LED Display

    initLEDs();

    //  Initialize state machines

    setBigRedButtonState(EBigRedButtonState::InitBigRedButton);
    setStripState(EStripState::InitStrip);

    //  Finally, play the startup sound
    playAudioFile(&g_startupSound);
}

void initAudioFiles()
{
    g_startupSound = SDWaveFile("startup.wav");
    
    if (!g_startupSound) 
    {
        Serial.println("could not setup 'startup.wav' soundfile!");
    }  

    g_fireSound = SDWaveFile("fire.wav");
    
    if (!g_fireSound) 
    {
        Serial.println("could not setup 'fire.wav' soundfile!");
    }  
}

void initLEDs()
{
    pinMode(CHARGING_LED_OUTPUT_PIN, OUTPUT);
    pinMode(ETHERNET_CONNECTION_OUTPUT_PIN, OUTPUT);
    
    digitalWrite(CHARGING_LED_OUTPUT_PIN, LOW);
    digitalWrite(ETHERNET_CONNECTION_OUTPUT_PIN, LOW);
}

//
//  Input sensing methods
//

void initSwitches()
{
    pinMode(SAFTY_SWITCH_INPUT_PIN, INPUT);
    pinMode(MOTOR_SWITCH_INPUT_PIN, INPUT);
    pinMode(FIRE_SWITCH_INPUT_PIN, INPUT);

    g_lastOnMessageSent = 0;
}

void loop()
{
    //  Keep track of frame dt

    updateDeltaTime();

    //  Update state machines

    updateBigRedButtonState();
    updateStripState();

    //  Update subsystems

    updateSwitchState();
    updatePulsePhase();
    updateBatteryLevel();
    updateLEDs();
}

void updateDeltaTime()
{
    static unsigned long prevTime = 0;
    
    g_dT = micros() - prevTime;
    prevTime = micros();
}

void updateSwitchState()
{
    bool motorSwitchOnPrev = g_motorSwitchOn;

    g_saftySwitchOn = digitalRead(SAFTY_SWITCH_INPUT_PIN) == NORMALLY_OPEN;
    g_motorSwitchOn = digitalRead(MOTOR_SWITCH_INPUT_PIN) == NORMALLY_OPEN;
    g_fireSwitchOn = digitalRead(FIRE_SWITCH_INPUT_PIN) == NORMALLY_OPEN;

    if (isMotorSwitchOn())
    {
        if (millis() - g_lastOnMessageSent > k_onMessageSendFrequency)
        {
            g_lastOnMessageSent = millis();
            sendUDPMessage(g_ipMotor, k_motorPort, k_turnOnMotorMessage);
        }
    }
    else
    {
        if (motorSwitchOnPrev != g_motorSwitchOn)
        {
            sendUDPMessage(g_ipMotor, k_motorPort, k_turnOffMotorMessage);            
        }
    }
    /*
    Serial.print("SaftySwitch: ");
    Serial.println(g_saftySwitchOn);
    
    Serial.print("MotorSwitch: ");
    Serial.println(g_motorSwitchOn);
   
    Serial.print("FireSwitch: ");
    Serial.println(g_fireSwitchOn);
    */
}

void updatePulsePhase()
{    
    static unsigned long lastUpdateTime = 0;
    static uint32_t lastColor = 0x00;
    
    if (g_stripState != EStripState::PowerPulse && 
        g_stripState != EStripState::SaftyPulse && 
        g_stripState != EStripState::MotorPulse)
    {
        return;
    }

    g_pulseTheta += k_radiansPerSecond / ((float) g_dT);

    if (g_pulseTheta > 2.0f * PI)
    {
        g_pulseTheta -= 2.0f * PI;    
    }


    if (micros() - lastUpdateTime > k_pulseUpdateFrequencyMin)
    {
        g_pulsePhase = constrain((sin(g_pulseTheta) + 1) / 2.0f, 0.0f, 1.0f);
        float phaseBrightness = (g_pulsePhase * (k_phaseBrightnessMax - k_phaseBrightnessMin)) + k_phaseBrightnessMin;
        uint8_t brightness = 255 * phaseBrightness;
        uint8_t saturation = 255;
        uint32_t color = g_strip.ColorHSV(g_ledHueCurrent, saturation, brightness);

        if (color != lastColor)
        {
            g_strip.fill(color, 0, g_strip.numPixels());
            g_strip.show();            
        }
        
        lastColor = color;
        lastUpdateTime = micros();
    }
}

void updateBatteryLevel()
{
    /*
      int sensorValue = analogRead(ADC_BATTERY);
      float voltage = sensorValue * (k_batteryVoltage / 1023.0f);
    */

    // print out the value you read:
    
    /*
    Serial.print(sensorValue);
    Serial.print(": ");
    Serial.print(voltage);
    Serial.println("V");
    */
}

void updateLEDs()
{
    //  BB MJS: For some reason, calling linkStatus when running on battery
    //  is causing massive issues.  Summary issue is loop is called way slower
    //  than normal, but other side effects are also present
/*
    if (Ethernet.linkStatus() == LinkON)
    {
        digitalWrite(ETHERNET_CONNECTION_OUTPUT_PIN, HIGH);
    }
    else
    {
        digitalWrite(ETHERNET_CONNECTION_OUTPUT_PIN, LOW);        
    }
*/
}

void playAudioFile(SDWaveFile *p_file)
{
    if (!AudioOutI2S.canPlay(*p_file)) 
    {
        Serial.println("unable to play wave file using I2S!");
        return;
    }
    
    if (AudioOutI2S.isPlaying())
    {
        AudioOutI2S.stop();
    }

    AudioOutI2S.play(*p_file);
}

void resetPulsePhase()
{
    g_pulseTheta = PI / 2.0f;
}

bool isSaftySwitchOn()
{
    return g_saftySwitchOn;
}

bool isMotorSwitchOn()
{
    return g_motorSwitchOn;
}

bool isFireSwitchOn()
{
    return g_fireSwitchOn;
}

void setPixelsWipeStep()
{
    uint8_t brightness = 255 * k_phaseBrightnessMax;
    uint8_t saturation = 255;
    uint32_t color = g_strip.ColorHSV(g_ledHueCurrent, saturation, brightness);
    
    int pixel1 = (g_strip.numPixels() / 2) + g_wipeOnStep + 1;
    int pixel2 = (g_strip.numPixels() / 2) - g_wipeOnStep;
    
    g_strip.setPixelColor(pixel1, color);
    g_strip.setPixelColor(pixel2, color);
    
    g_strip.show();  
}

//
//  Sound functions
//

//
//  State tracking methods
//

void updateBigRedButtonState()
{
    while (true)
    {
        EBigRedButtonState prevbigRedButtonState = g_bigRedButtonState;

        switch (g_bigRedButtonState)
        {
        case EBigRedButtonState::InitBigRedButton:
            {
                setBigRedButtonState(EBigRedButtonState::Idle);
            }
            break;

        case EBigRedButtonState::Idle:
            {
                if (isSaftySwitchOn())
                {
                    setBigRedButtonState(EBigRedButtonState::Armed);                
                }
            }
            break;
            
        case EBigRedButtonState::Armed:
            {
                if (!isSaftySwitchOn())
                {
                    setBigRedButtonState(EBigRedButtonState::Idle);                
                }
                if (isFireSwitchOn())
                {
                    setBigRedButtonState(EBigRedButtonState::Fire);
                }
            }
            break;
            
        case EBigRedButtonState::Fire:
            {
                setBigRedButtonState(EBigRedButtonState::CoolDown);
            }
            break;
            
        case EBigRedButtonState::CoolDown:
            {
                if (!isFireSwitchOn() && micros() - g_bigRedButtonStateEnterTime > k_bigRedButtonCoolDownDuration)
                {
                    if (isSaftySwitchOn())
                    {
                        setBigRedButtonState(EBigRedButtonState::Armed);
                    }
                    else
                    {
                        setBigRedButtonState(EBigRedButtonState::Idle);
                    }
                }
            }
            break;
            
        default:
            {
            }
            break;
        }
        
        if (prevbigRedButtonState == g_bigRedButtonState)
        {
            break;
        }
    }
}

void setBigRedButtonState(EBigRedButtonState p_newBigRedButtonState)
{
    if (g_bigRedButtonState == p_newBigRedButtonState)
    {
        return;
    }

    //  Action to take when leaving a state
    
    switch (g_bigRedButtonState)
    {
    default:
        {
        }
        break;
    }

    //  Set the state
    
    g_bigRedButtonState = p_newBigRedButtonState;
    g_bigRedButtonStateEnterTime = micros();

    //  Action to take when entering a state
    
    switch (g_bigRedButtonState)
    {

    case EBigRedButtonState::Armed:
        {
            playAudioFile(&g_fireSound);
         }
        break;
        
    case EBigRedButtonState::Fire:
        {
            sendUDPMessage(g_ipPneumatics, k_pneumaticsPort, k_firePneumaticsMessage);
        }
        break;
        
    default:
        {
        }
        break;
    }
}

void updateStripState()
{
    while (true)
    {
        EStripState prevStripState = g_stripState;

        switch (g_stripState)
        {
        case EStripState::InitStrip:
            {
                setStripState(EStripState::Off);
            }
            break;

        case EStripState::Off:
            {
                setStripState(EStripState::WipeOnStart);
            }
            break;

        case EStripState::WipeOnStart:
            {
                if (isMotorSwitchOn())
                {
                    setStripState(EStripState::MotorWipeOnStep);
                }
                else if (isSaftySwitchOn())
                {
                    setStripState(EStripState::SaftyWipeOnStep);
                }
                else
                {
                    setStripState(EStripState::PowerWipeOnStep);
                }
            }
            break;

            case EStripState::PowerWipeOnStep:
            {
                if (isSaftySwitchOn() || isMotorSwitchOn())
                {
                    setStripState(EStripState::Off);
                }
                else if (micros() - g_stripStateEnterTime > k_wipeOnStepDt)
                {
                    setStripState(EStripState::WipeOnNext);
                }
            }
            break;            

        case EStripState::SaftyWipeOnStep:
            {
                if (!isSaftySwitchOn())
                {
                    setStripState(EStripState::Off);
                }
                else if (micros() - g_stripStateEnterTime > k_wipeOnStepDt)
                {
                    setStripState(EStripState::WipeOnNext);
                }
            }
            break;            

        case EStripState::MotorWipeOnStep:
            {
                if (!isMotorSwitchOn())
                {
                    setStripState(EStripState::Off);                        
                }
                else if (micros() - g_stripStateEnterTime > k_wipeOnStepDt)
                {
                    setStripState(EStripState::WipeOnNext);
                }
            }
            break;            

        case EStripState::WipeOnNext:
            {
                if (g_wipeOnStep >= (g_strip.numPixels() / 2))
                {
                    if (isMotorSwitchOn())
                    {
                        setStripState(EStripState::MotorPulse);
                    }
                    else if (isSaftySwitchOn())
                    {
                        setStripState(EStripState::SaftyPulse);                        
                    }
                    else
                    {
                        setStripState(EStripState::PowerPulse);
                    }
                }
                else
                {
                    if (isMotorSwitchOn())
                    {
                        setStripState(EStripState::MotorWipeOnStep);
                    }
                    else if (isSaftySwitchOn())
                    {
                        setStripState(EStripState::SaftyWipeOnStep);                        
                    }
                    else
                    {
                        setStripState(EStripState::PowerWipeOnStep);
                    }
                }
            }
            break;
            
         case EStripState::PowerPulse:
            {
                if (isMotorSwitchOn() || isSaftySwitchOn())
                {
                    setStripState(EStripState::Off);
                }
            }
            break;

         case EStripState::SaftyPulse:
            {
                if (isMotorSwitchOn() || !isSaftySwitchOn())
                {
                    setStripState(EStripState::Off);
                }
            }
            break;
            
         case EStripState::MotorPulse:
            {
                if (!isMotorSwitchOn())
                {
                    setStripState(EStripState::Off);
                }
            }
            break;
            
        default:
            {
            }
            break;
        }
        
        if (prevStripState == g_stripState)
        {
            break;
        }
    }
}

void setStripState(EStripState p_newStripState)
{
    if (g_stripState == p_newStripState)
    {
        return;
    }

    //  Action to take when leaving a state
    
    switch (g_stripState)
    {
    default:
        {
        }
        break;
    }

    //  Set the state
    
    g_stripState = p_newStripState;
    g_stripStateEnterTime = micros();

    //  Action to take when entering a state
    
    switch (g_stripState)
    {
    case EStripState::InitStrip:
        {
            g_strip.begin();
            g_strip.show();
            g_strip.setBrightness(255);
        }
        break;
        
    case EStripState::Off:
        {
            g_strip.clear();  
            g_strip.show();
        }
        break;

    case EStripState::WipeOnStart:
        {
            g_wipeOnStep = 0;    
        }
        break;

    case EStripState::PowerWipeOnStep:
        {
            g_ledHueCurrent = k_ledHueGreen;
            setPixelsWipeStep();
        }
        break;
    
    case EStripState::SaftyWipeOnStep:
        {
            g_ledHueCurrent = k_ledHueYellow;
            setPixelsWipeStep();
        }
        break;

    case EStripState::MotorWipeOnStep:
        {
            g_ledHueCurrent = k_ledHueRed;
            setPixelsWipeStep();
        }
        break;

    case EStripState::WipeOnNext:
        {
            g_wipeOnStep++;
        }
        break;

    case EStripState::PowerPulse:
        {
            resetPulsePhase();
        }
        break;
        
    case EStripState::SaftyPulse:
    case EStripState::MotorPulse:
        {
            resetPulsePhase();
        }
        break;
        
    default:
        {
        }
        break;
    }
}

// Networking Methods


void sendUDPMessage(IPAddress p_remoteIp, unsigned int p_remotePort, String p_message)
{
    Serial.print("Send UDP Message ");
    Serial.println(p_message.c_str());

    if (g_udpConnection.beginPacket(p_remoteIp, p_remotePort))
    {
        int sentBytes = g_udpConnection.write(p_message.c_str(), p_message.length());

        if (sentBytes > 0)
        {
            Serial.println("call endPacket");
            g_udpConnection.endPacket();
            Serial.println("sendUDPMessage complete");

            delay(1);
            
            int packetSize = g_udpConnection.parsePacket();
            Serial.print("parsePacket packetSize: ");
            Serial.println(packetSize);
                
            if (packetSize)
            {
                g_udpConnection.read(g_packetBuffer, UDP_TX_PACKET_MAX_SIZE);    
                Serial.println(g_packetBuffer);
            }
        }
        else
        {
            Serial.println("Could not send message");
        }
    }
}
