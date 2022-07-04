// ATENÇAO lembrar de ligaçao GPIO <-> RST

#include <ESP8266WiFi.h>

extern "C" {
#include <espnow.h>
}

#include <DallasTemperature.h>
#include <OneWire.h>

#include "I2Cdev.h"
#include "SPI.h"
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;
const int alimentacaoSensores = 15;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

LSM6DS3 myIMU(SPI_MODE, 5);

// this is the MAC Address of the remote ESP server which receives these sensor
// readings MAC ADDRESS ESP32 24:6f:28:d1:23:40 MAC ADDRESS ESP32_2
// 80:7D:3A:D5:42:99
uint8_t macAddr[] = {0x24, 0x6F, 0x28, 0xD1, 0x23, 0x40};
// uint8_t macAddr[] = {0x80, 0x7D, 0x3A, 0xD5, 0x42, 0x98};

#define WIFI_CHANNEL 4
#define LONG_SLEEP 3600   // 60 minutes
#define SHORT_SLEEP 60    // 1 minutes
#define SEND_TIMEOUT 245  // 245 millis seconds timeout
#define ACK_TIMEOUT 5000
#define INSPECTION_SIZE 8000
#define NUMBER_OF_PARTS INSPECTION_SIZE / 40
#define PACK_SIZE 40

int16_t tempVibration[3][INSPECTION_SIZE];

struct __attribute__((packed)) SENSOR_DATA {
  short int identifier;             // 2 bytes
  float temp;                       // 4 bytes
  float sample_rate;                // 4 bytes
  int16_t vibration[3][PACK_SIZE];  // 2 bytes * 3 * 40
} tempSensor;

uint8_t ack;
unsigned long ack_time;
short int ack_trials = 0;

// Create a struct_message called myData
// struct_message msgTest;

volatile boolean callbackCalled = false;
volatile boolean acknowledged = false;
volatile boolean packedSolicitation = false;
volatile boolean endSession = false;
volatile short int packageIdentifier = 0;

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  // Serial.print("****************************Bytes received:
  // ******************************"); Serial.println(len);

  // Serial.println("Recebi mensagem");
  if (len == 1) {
    if (*incomingData == 254) {
      // Serial.println("Reconhecido");
      acknowledged = true;
    } else if (*incomingData == 253) {
      // endPackageAssembly
      endSession = true;
    } else {
      // Serial.print("packedSolicitation: ");
      // Serial.println(*incomingData);
      packedSolicitation = true;
      packageIdentifier = *incomingData;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(alimentacaoSensores, OUTPUT);
  digitalWrite(alimentacaoSensores, HIGH);
  delay(100);
  Serial.println("Start the DS18B20 sensor");
  // Start the DS18B20 sensor
  sensors.begin();
  myIMU.settings.gyroEnabled = 0;  // Can be 0 or 1
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 8;  // Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 6660;  // Hz.  Can be: 13, 26, 52, 104, 208,
                                          // 416, 833, 1666, 3332, 6660, 13330
  myIMU.settings.accelFifoEnabled =
      0;  // Set to include accelerometer in the FIFO
  myIMU.settings.tempEnabled = 0;
  myIMU.settings.commMode = 1;

  WiFi.mode(WIFI_STA);  // Station mode for esp-now sensor node
  WiFi.disconnect();

  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());
  Serial.printf("target mac: %02x%02x%02x%02x%02x%02x", macAddr[0], macAddr[1],
                macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  // Serial.printf(", channel: %i\n", WIFI_CHANNEL);

  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    gotoSleep(SHORT_SLEEP);
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(macAddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);

  esp_now_register_send_cb([](uint8_t *mac, uint8_t sendStatus) {
    // Serial.printf("send_cb, send done, status = %i\n", sendStatus);
    callbackCalled = true;
  });

  esp_now_register_recv_cb(OnDataRecv);

  callbackCalled = false;

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(myIMU.begin() == IMU_SUCCESS ? "LSM6DS3 connection successful"
                                              : "LSM6DS3 connection failed");

  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  tempSensor.temp = temperatureC;
  Serial.print(temperatureC);
  Serial.println(" ºC");
  Serial.println("Coletando acelerômetro!");
  unsigned long initTime = millis();
  int samples = 0;
  int16_t ax = 0, ay = 0, az = 0;
  status_t readStatus = IMU_HW_ERROR;
  uint8_t tempReadByte = 0;

  // Coleta INSPECTION_SIZE amostras do acelerometro
  while (samples < INSPECTION_SIZE) {
    do {
      readStatus =
          myIMU.readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_STATUS_REG);
    } while (readStatus == IMU_SUCCESS &&
             (tempReadByte & 0b00000001) == 0);  // wait for reading

    if (readStatus == IMU_SUCCESS) {
      tempReadByte &= 0b00000001;
      if (tempReadByte == 1) {
        tempVibration[0][samples] = myIMU.readRawAccelX();
        tempVibration[1][samples] = myIMU.readRawAccelX();
        tempVibration[2][samples] = myIMU.readRawAccelX();
      }
    }

    samples++;
    // Serial.println(samples);
  }

  unsigned long finalTime = millis();
  digitalWrite(alimentacaoSensores, LOW);

  Serial.println(" Sample Rate!");
  Serial.print((samples * 1000.0) / (finalTime - initTime));
  Serial.println(" Hz");
  Serial.println("Iniciando Envio");

  // send ack to start Package assembly
  Serial.println("start Package assembly");
  uint8_t CMD = 254;
  esp_now_send(NULL, &CMD, 1);
  while (!callbackCalled) {
    delay(1);
  }
  callbackCalled = false;

  Serial.println("Esperando ack");
  ack_time = millis();
  while (!acknowledged && ((millis() - ack_time) < ACK_TIMEOUT)) {
    delay(1);
  }
  if (!acknowledged) {
    Serial.println("Not Ack, indo mimir short");
    gotoSleep(SHORT_SLEEP);
  }

  Serial.println("acked");

  while (!endSession) {
    if (packedSolicitation) {
      packedSolicitation = false;
      Serial.print("Solicitação de pacote ");
      Serial.println(packageIdentifier);
      tempSensor.identifier = packageIdentifier;
      tempSensor.sample_rate = ((1000 * samples) / (finalTime - initTime));
      uint16_t multiple = packageIdentifier * PACK_SIZE;
      // Serial.print(multiple);
      // Serial.println(" múltiplo!");
      for (short int i = 0; i < PACK_SIZE; i++) {
        tempSensor.vibration[0][i] = tempVibration[0][multiple + i];
        tempSensor.vibration[1][i] = tempVibration[1][multiple + i];
        tempSensor.vibration[2][i] = tempVibration[2][multiple + i];
      }

      uint8_t bs[sizeof(tempSensor)];
      memcpy(bs, &tempSensor, sizeof(tempSensor));
      Serial.print("Vou mandar: ");
      Serial.println(tempSensor.identifier);
      esp_now_send(NULL, bs,
                   sizeof(tempSensor));  // NULL means send to all peers
      // Waint until send is complete
      while (!callbackCalled) {
        delay(1);
      }
      callbackCalled = false;
    }
    delay(1);
  }

  Serial.println("Finalizado Envio");
}

void loop() {
  if (callbackCalled || (millis() > SEND_TIMEOUT)) {
    gotoSleep(LONG_SLEEP);
  }
}

void gotoSleep(int sleepTime) {
  // add some randomness to avoid collisions with multiple devices
  int sleepSecs = sleepTime + ((uint8_t)RANDOM_REG32 / 32);
  Serial.printf("Up for %i ms, going to sleep for %i secs...\n", millis(),
                sleepSecs);
  ESP.deepSleep(sleepSecs * 1000000);
}
