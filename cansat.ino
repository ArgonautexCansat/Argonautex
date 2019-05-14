// ---------------------------------
// Código CANSAT VIERA
// ---------------------------------

// Librerías
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <Crypto.h>
#include <Speck.h>
#include <HardwareSerial.h>
#include <LoRa.h>

// Pines
#define DHTPIN 25
#define RXPIN 16
#define TXPIN 17
#define MQ135PIN 4
#define MQ9PIN 15

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6

// Constantes
#define SEALEVELPRESSURE_HPA (1013.25)

// Objetos de sensores
Adafruit_BME280 bme;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// GPS
TinyGPSPlus GPS;
HardwareSerial GPSSerial(1);

// Datos guardados
float bme_pressure = 0;

float bme_humidity = 0;
float dht_humidity = 0;

double GPS_latitude = 0;
double GPS_longitude = 0;
float GPS_altitude = 0;
float bme_altitude = 0;

float mq_co2 = 0;
float mq_ch4 = 0;

float magX, magY, magZ;

float bme_temperature = 0;

// Criptografía
Speck speck;
byte key[16] = {0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08,
                0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00};

// Macros
#ifdef DEBUG
#define Sprint(a) (Serial.print(a))
#define Sprintln(a) (Serial.println(a))
#else
#define Sprint(a)
#define Sprintln(a)
#endif

// ----------------------------------------
// Funciones de lectura para los sensores
// ----------------------------------------
void read_bme()
{
  float pres, temp, humi, alti;
  pres = bme.readPressure() / 100.0F;
  temp = bme.readTemperature();
  humi = bme.readHumidity();
  alti = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  Sprint("Presión: ");
  Sprint(pres);
  Sprintln(" hPa");

  Sprint("Temperatura: ");
  Sprint(temp);
  Sprintln(" C"); 

  bme_pressure = pres;
  bme_temperature = temp;
  bme_humidity = humi;
  bme_altitude = alti;
}

void read_GPS()
{
  while (GPSSerial.available() > 0)
  {  
    if (GPS.encode(GPSSerial.read()))
    {           
      if(GPS.location.isValid())
      {
        double latitud = GPS.location.lat();
        double longitud = GPS.location.lng();
        Sprint("Latitud: "); Sprint(latitud);
        Sprint("Longitud: "); Sprintln(longitud);

        GPS_latitude = latitud;
        GPS_longitude = longitud;
      }

      if(GPS.altitude.isValid())
      {
        float altitud = GPS.altitude.value();
        Sprint("Altitud: "); Sprintln(altitud);

        GPS_altitude = altitud;
        // esto es variación de altitud, hay que ver cada cuanto coge los datos
        //GPS_fallSpeed = altitud - GPS_altitude;
      }

      if(GPS.time.isValid())
      {
        Sprint("Tiempo: "); Sprintln(GPS.time.value());
      }
    }
  }
}

void read_mq()
{
  int adc_MQ135 = analogRead(MQ135PIN);
  float MQ135voltage = adc_MQ135 * (5.0 / 1023.0);

  int adc_MQ9 = analogRead(MQ9PIN);
  float MQ9voltage = adc_MQ9 * (5.0 / 1023.0);

  Sprint("adc1: "); Sprint(adc_MQ135);
  Sprint("voltaje1: "); Sprintln(MQ135voltage);
  Sprint("adc2: "); Sprint(adc_MQ9);
  Sprint("voltaje2: "); Sprintln(MQ9voltage);

  mq_co2 = MQ135voltage;
  mq_ch4 = MQ9voltage;
  // !!!queda pasar voltaje a concentración!!!
}

void read_compass()
{
  sensors_event_t event;
  mag.getEvent(&event);

  magX = event.magnetic.x;
  magY = event.magnetic.y;
  magZ = event.magnetic.z;

  Sprint("X: "); Sprint(magX); Sprint(" ");
  Sprint("Y: "); Sprint(magY); Sprint(" ");
  Sprint("Z: "); Sprint(magZ); Sprint(" "); Sprintln("uT");
}

// ----------------------------------------
// Criptografía y transmisión de datos
// ----------------------------------------

String get_packet()
{
  String packet = "";
  packet += millis(); packet += ";"; 
  packet += bme_pressure; packet += ";";
  packet += bme_temperature; packet += ";"; 
  packet += bme_humidity; packet += ";"; 
  packet += GPS_latitude; packet += ";"; 
  packet += GPS_longitude; packet += ";"; 
  packet += GPS_altitude; packet += ";";
  packet += bme_altitude; packet += ";"; 
  packet += mq_co2; packet += ";"; 
  packet += mq_ch4; packet += ";"; 
  packet += magX; packet += ";"; 
  packet += magY; packet += ";"; 
  packet += magZ;
  return packet;
}

void encrypt_data(String packet, byte blocksEnc[5][16])
{
  byte pac[100] = {0};
  byte blocksDec[5][16] = {0};
  packet.getBytes(pac, packet.length());

  // Dividimos el paquete en bloques de 16 bytes.
  for (int b = 0; b < 5; b++)
  {
    for (int i = 0; i < 16; i++)
    {
      blocksDec[b][i] = pac[b*16+i];
    }
    
    speck.encryptBlock(blocksEnc[b], blocksDec[b]);
  }
}

void transmit_data(byte blocksEnc[5][16])
{
  Sprintln("Emitiendo paquete...");
  LoRa.beginPacket();
  for(int b = 0; b < 5; b++)
  {
    for(int i = 0; i < 16; i++)
    {
      LoRa.write(blocksEnc[b][i]);
      Sprint(blocksEnc[b][i]);
    }                                                                                  
  }  
  LoRa.endPacket();
  Sprintln("Paquete emitido."); 
}

void send_data()
{
  byte blocksEnc[5][16];
  String packet = get_packet();                                    
  encrypt_data(packet, blocksEnc);
  
  transmit_data(blocksEnc);
}

// ----------------------------------------
// Funciones Arduino
// ----------------------------------------
void setup()
{
  pinMode(MQ135PIN, INPUT);
  pinMode(MQ9PIN, INPUT);
  
  Serial.begin(115200);
  Sprintln("CANSAT Viera v2.0.0");
  Sprintln("Comenzando inicialización...");
  GPSSerial.begin(9600, SERIAL_8N1, RXPIN, TXPIN);

  //Iniciar LORA
  //SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);

  Sprintln("Iniciando LoRa...");
  if(!LoRa.begin(BAND)) 
  {
    Sprintln("LoRa no ha podido iniciarse.");
    while(true) { }
  }
  Sprintln("LoRa iniciado.");

  // Iniciar los sensores
  Sprintln("Iniciando BME280...");
  if(!bme.begin())
  {
    Sprintln("Error al iniciar BME280");
  }
  Sprintln("BME280 iniciado.");

  Sprintln("Iniciando HMC5883L...");
  if(!mag.begin())
  {
    Sprintln("Error al iniciar HMC5883L");
  }
  Sprintln("HMC5883L iniciado.");

  // Criptografía
  speck.setKey(key, 16);
}

void loop()
{  
  Sprintln("(BME280)");
  read_bme();
  Sprintln("====================");

  Sprintln("(Gas MQ-9 y MQ-135)");
  read_mq();
  Sprintln("====================");

  Sprintln("(GPS GY-NEO6MV2)");
  read_GPS();
  Sprintln("====================");
  
  Sprintln("(Brújula HMC5883L)");
  read_compass();
  Sprintln("====================");
  
  send_data();
  delay(100);
  
  Sprintln("*********************");
}
