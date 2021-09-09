/*******************************************************************************
* Código do lilygo para comunicação com o OBD2 via bluetooth e capta os dados
* após comunicação faz comunicação via LoRa para envio de informação para o
* servidor.
*******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include <TinyGPS++.h>                       


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif


/*****
 * Configuração de variáveis para o bluetooth do OBD2
 */
BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial
TinyGPSPlus gps;
ELM327 myELM327;

//Endereço MAC do bluetooth do OBD2 e as variáveis que serão
//recebidas do OBD2
uint8_t BTMAC[6] = {0x7D, 0xE6, 0x95, 0x19, 0xD1, 0xA2};
uint32_t rpm = 0;
uint32_t kph = 0;

uint8_t payload[10];

//Flag para enviar msg LoRa somente se GPS e OBD2 estiverem conectados
int flagGPS=0;
int flagOBD=0;
int flag_TXCOMPLETE = 0;

/*
 * Configuração de variáveis do GPS
 */
#define SERIAL1_RX 34 // GPS_TX -> 34
#define SERIAL1_TX 12 // 12 -> GPS_RX
String read_sentence;


/*
 * Configuração de variáveis LoRa
 */

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x63, 0x7c, 0x7f, 0xcf, 0x5d, 0x0a, 0x6a, 0xa9, 0xca, 0x84, 0x28, 0x22, 0x6d, 0x83, 0xdc, 0x6b };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xd1, 0x33, 0x3a, 0xd4, 0x90, 0x4a, 0x50, 0xee, 0xad, 0x93, 0xee, 0xd7, 0x1e, 0xe9, 0x76, 0x01 };

// LoRaWAN end-device address (DevAddr)
// The library converts the address to network byte order as needed.
#ifndef COMPILE_REGRESSION_TEST
static const u4_t DEVADDR = 0x00d7d32c;
#else
static const u4_t DEVADDR = 00d7d32c;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// payload to send to gateway
static uint8_t tx_payload[9];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

//LoRa pin mapping ESP32 (LILYGO Board V1.1)
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {26, 33, 32},
};


/*
 * Função para eventos de comunicação LoRa
 */
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            flag_TXCOMPLETE = 1;
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}


/*
 * Função para envio de Pacote LoRa
 */
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      Serial.println("Enviando Payload...");

      // prepare upstream data transmission at the next possible time.
      // transmit on port 13 (the first parameter); you can use any value from 1 to 223 (others are reserved).
      // request an ack (the last parameter, if not zero, requests an ack from the network).
      // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
      LMIC_setTxData2(13, payload, sizeof(payload)-1, 1);
    }
}


/*
 * Função para conectar ao OBD2 via bluetooth
 */
void OBDConnect(){
  SerialBT.setPin("1234");
  ELM_PORT.begin("EspOBD", true);

  if (!ELM_PORT.connect(BTMAC))
  //if (!ELM_PORT.connect("OBDII"))
  {
    DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
    //while(1);
  }
  else if (!myELM327.begin(ELM_PORT, true, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");
   // while (1);
  }
  else{
    Serial.println("Connected to ELM327");
  }
}

/*
 * Setup de configuração do hardware
 */
void setup() {
    delay(5000);
    while (!Serial);
    Serial.begin(115200);
    delay(100);
    Serial.println(F("Starting"));
    Serial1.begin(9600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    //SPI pin mapping ESP32 (LILYGO Board V1.1)
    SPI.begin(5, 19, 27, 18);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Função para conexão com o OBD2
    OBDConnect();
    
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

    // We'll disable all 72 channels used
    for (int c = 0; c < 72; c++){
      LMIC_disableChannel(c);
    }

    // Seta canais para envio de pacote LoRa
    LMIC_enableChannel(0);
    LMIC_enableChannel(1);
    LMIC_enableChannel(2);
    LMIC_enableChannel(3);
    LMIC_enableChannel(4);
    LMIC_enableChannel(5);
    LMIC_enableChannel(6);
    LMIC_enableChannel(7);
    LMIC_enableChannel(8);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    //do_send(&sendjob);
}

/*
 * Função de loop
 */
void loop() {

  read_sentence = Serial1.readStringUntil(13); //13 = return (ASCII)
  read_sentence.trim();
  if(flagOBD == 1){
    // Função para conexão com o OBD2
    OBDConnect();
  }
  
  if (Serial1.available()) {
    Serial.write(Serial1.read());
    Serial1.println();
  }
  if (read_sentence.startsWith("$GPGGA")) {

      Serial.print("Latitude  : ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude : ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude  : ");
      Serial.println(gps.altitude.feet() / 3.2808);
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
  
      gps_payload(gps.location.lat(), gps.location.lng(), (gps.altitude.feet()/3.2808));
      flagGPS = 0;
      Serial.println("GPS Conectado");
    }
    else{
      Serial.println("GPS Não Conectado");
      flagGPS = 1;
    }
        
    if (myELM327.status == ELM_SUCCESS)
     {
        float tempRPM = myELM327.rpm();
        float veloci = myELM327.kph();
        rpm = (uint32_t)tempRPM;
        Serial.print("RPM: "); Serial.println(rpm);
        kph = (uint32_t)veloci;
        Serial.print("Velocidade: "); Serial.print(kph); Serial.println("Km/h");
        obd_payload(kph);
        flagOBD = 0;
     }
     else{
        myELM327.printError();
        //tx_payload[10] = 24;
        flagOBD = 1;
    }
    if(flagOBD == 0){
      Serial.println("Enviei aqui mané");
      do_send(&sendjob);
      Serial.println("Sending...");
      //Run LMIC loop until he as finish
      while(flag_TXCOMPLETE == 0  )
      {
        os_runloop_once();
      }
      flag_TXCOMPLETE = 0;
    }
    delay(10000);
}

/*
 * Função para alocação das informações do 
 * GPS em pacotes do payload LoRa
 */
void gps_payload(double lat, double lon, int alt) {
  uint32_t LatitudeBinary = 0;
  uint32_t LongitudeBinary = 0;

  if (lat != 0){
    LatitudeBinary = (lat  * 10000);
    LongitudeBinary = (lon  * 10000);
    alt = alt * 100;
  }

  payload[0] = LatitudeBinary >> 24 ;
  payload[1] = LatitudeBinary >> 16;
  payload[2] = LatitudeBinary >> 8;
  payload[3] = LatitudeBinary;

  payload[4] = LongitudeBinary >> 24;
  payload[5] = LongitudeBinary >> 16;
  payload[6] = LongitudeBinary >> 8;
  payload[7] = LongitudeBinary;

  payload[8] = alt >> 8;
  payload[9] = alt;

  int x = 0;
  for (x=0; x <=10; x++){
    Serial.print(payload[x], HEX);
  }
  Serial.println(" ");
  }

/*
 * Função para alocação das informações do 
 * OBD2 em pacotes do payload LoRa
 */
void obd_payload(int kmh) {
  tx_payload[8] = kmh;
}
