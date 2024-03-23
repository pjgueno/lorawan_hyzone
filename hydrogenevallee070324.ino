#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <HardwareSerial.h>

#include <Wire.h>               
#include "HT_SSD1306Wire.h"

#include "./bmx280_i2c.h"
#include "./utils.h"
#include "./defines.h"
#include "./ext_def.h"

SSD1306Wire Display(0x3c,700000, I2C_PIN_SDA, I2C_PIN_SCL);

long int sample_count = 0;
bool htu21d_init_failed = false;
bool bmp_init_failed = false;
bool bmx280_init_failed = false;
bool sht3x_init_failed = false;
bool dnms_init_failed = false;
bool gps_init_failed = false;
bool airrohr_selftest_failed = false;

#define serialSDS (Serial1)

BMX280 bmx280;

boolean trigP1 = false;
boolean trigP2 = false;
unsigned long trigOnP1;
unsigned long trigOnP2;

unsigned long lowpulseoccupancyP1 = 0;
unsigned long lowpulseoccupancyP2 = 0;

unsigned long starttime;
unsigned long time_point_device_start_ms;
unsigned long starttime_SDS;
unsigned long act_micro;
unsigned long act_milli;
unsigned long last_micro = 0;
unsigned long min_micro = 1000000000;
unsigned long max_micro = 0;

bool is_SDS_running = true;
enum {
  SDS_REPLY_HDR = 10,
  SDS_REPLY_BODY = 8
} SDS_waiting_for;

bool start_SDS = false;

unsigned long sending_time = 0;
int last_update_returncode;
int last_sendData_returncode;

float last_value_BMX280_T = -128.0;
float last_value_BMX280_P = -1.0;
float last_value_BME280_H = -1.0;

uint32_t sds_pm10_sum = 0;
uint32_t sds_pm25_sum = 0;
uint32_t sds_val_count = 0;
uint32_t sds_pm10_max = 0;
uint32_t sds_pm10_min = 20000;
uint32_t sds_pm25_max = 0;
uint32_t sds_pm25_min = 20000;

float last_value_SDS_P1 = -1.0;
float last_value_SDS_P2 = -1.0;
String last_data_string;
int last_signal_strength;
int last_disconnect_reason;

String esp_chipid;
String last_value_SDS_version;

unsigned long SDS_error_count;

unsigned long last_page_load = millis();

bool wificonfig_loop = false;

unsigned long count_sends = 0;
unsigned long last_display_millis = 0;
uint8_t next_display_count = 0;

#define msSince(timestamp_before) (act_milli - (timestamp_before))

bool SDS_read_OK = false;
bool BME_read_OK = false;

static String SDS_version_date() {

  if (!last_value_SDS_version.length()) {
    is_SDS_running = SDS_cmd(PmSensorCmd::Start);
    delay(250);
    serialSDS.flush();
    // Query Version/Date
    SDS_rawcmd(0x07, 0x00, 0x00);
    delay(400);
    const constexpr uint8_t header_cmd_response[2] = { 0xAA, 0xC5 };
    while (serialSDS.find(header_cmd_response, sizeof(header_cmd_response))) {
      uint8_t data[8];
      unsigned r = serialSDS.readBytes(data, sizeof(data));
      if (r == sizeof(data) && data[0] == 0x07 && SDS_checksum_valid(data)) {
        char tmp[20];
        snprintf_P(tmp, sizeof(tmp), PSTR("%02d-%02d-%02d(%02x%02x)"),
                   data[1], data[2], data[3], data[4], data[5]);
        last_value_SDS_version = tmp;
        break;
      }
    }
  }

  return last_value_SDS_version;
}

static void display_debug(const String& text1, const String& text2) {
  Serial.printf("output debug text to displays...\n");
  Display.clear();
  Display.drawString(20, 0, text1);
  Display.drawString(20, 10, text2);
  Display.display();
}

static void sensor_restart() {
  serialSDS.end();
  Serial.printf("Restart...\n");
  delay(500);
  ESP.restart();
  // should not be reached
  while (true) {
    yield();
  }
}


/*****************************************************************
   read BMP280/BME280 sensor values
//  *****************************************************************/
 static void fetchSensorBMX280() {
   const char* const sensor_name = (bmx280.sensorID() == BME280_SENSOR_ID) ? "BME280" : "BMP280";
   bmx280.takeForcedMeasurement();
   const auto t = bmx280.readTemperature();
   const auto p = bmx280.readPressure();
   const auto h = bmx280.readHumidity();
   if (isnan(t) || isnan(p)) {
     last_value_BMX280_T = -128.0;
     last_value_BMX280_P = -1.0;
     last_value_BME280_H = -1.0;
     Serial.printf("BMP/BME280 read failed...\n");
   } else {
     last_value_BMX280_T = t;
     last_value_BMX280_P = p;
     if (bmx280.sensorID() == BME280_SENSOR_ID) {
       last_value_BME280_H = h;
     }
     Serial.printf("Temperature : %s \n", String(last_value_BMX280_T));
     Serial.printf("Pression : %s \n", String(last_value_BMX280_P));
     Serial.printf("Humidite : %s \n", String(last_value_BME280_H));
   }
 }

/*****************************************************************
   read SDS011 sensor values
 *****************************************************************/
static void fetchSensorSDS() {

  if (start_SDS) {

    if  (! is_SDS_running) {
      Serial.printf("Start SDS...\n");
      is_SDS_running = SDS_cmd(PmSensorCmd::Start);
      SDS_waiting_for = SDS_REPLY_HDR;
    }

    while (serialSDS.available() >= SDS_waiting_for) {
      const uint8_t constexpr hdr_measurement[2] = { 0xAA, 0xC0 };
      uint8_t data[8];

      switch (SDS_waiting_for) {
        case SDS_REPLY_HDR:
          if (serialSDS.find(hdr_measurement, sizeof(hdr_measurement)))
            SDS_waiting_for = SDS_REPLY_BODY;
          break;
        case SDS_REPLY_BODY:
          if (serialSDS.readBytes(data, sizeof(data)) == sizeof(data) && SDS_checksum_valid(data)) {
            uint32_t pm25_serial = data[0] | (data[1] << 8);
            uint32_t pm10_serial = data[2] | (data[3] << 8);

            if (msSince(starttime) > WARMUPTIME_SDS_MS && msSince(starttime) < (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)) {
              Serial.printf("Measuring...\n");
              sds_pm10_sum += pm10_serial;
              sds_pm25_sum += pm25_serial;
              UPDATE_MIN_MAX(sds_pm10_min, sds_pm10_max, pm10_serial);
              UPDATE_MIN_MAX(sds_pm25_min, sds_pm25_max, pm25_serial);
              Serial.printf("PM10 (sec.) : %s \n", String(pm10_serial / 10.0f));
              Serial.printf("PM2.5 (sec.): %s \n", String(pm25_serial / 10.0f));
              sds_val_count++;
            } else {
              Serial.printf("Warming up...\n");
            }
          }
          SDS_waiting_for = SDS_REPLY_HDR;
          break;
      }
    }

    if (msSince(starttime) > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)) {
      last_value_SDS_P1 = -1;
      last_value_SDS_P2 = -1;
      if (sds_val_count > 2) {
        sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
        sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
        sds_val_count = sds_val_count - 2;
      }
      if (sds_val_count > 0) {
        last_value_SDS_P1 = float(sds_pm10_sum) / (sds_val_count * 10.0f);
        last_value_SDS_P2 = float(sds_pm25_sum) / (sds_val_count * 10.0f);
        if (sds_val_count < 3) {
          SDS_error_count++;
        }
      } else {
        SDS_error_count++;
      }
      sds_pm10_sum = 0;
      sds_pm25_sum = 0;
      sds_val_count = 0;
      sds_pm10_max = 0;
      sds_pm10_min = 20000;
      sds_pm25_max = 0;
      sds_pm25_min = 20000;
      SDS_read_OK = true;
      start_SDS = false;
      Serial.printf("Ready to send...\n");
      if (is_SDS_running) {
        Serial.printf("Stop SDS...\n");
        is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
      }
    }
  }
}


/*****************************************************************
   display values
 *****************************************************************/
static void display_values() {
  float t_value = -128.0;
  float h_value = -1.0;
  float p_value = -1.0;
  String t_sensor, h_sensor, p_sensor;
  float pm01_value = -1.0;
  float pm04_value = -1.0;
  float pm10_value = -1.0;
  float pm25_value = -1.0;
  String pm10_sensor;
  String pm25_sensor;
  float tps_value = -1.0;
  double lat_value = -200.0;
  double lon_value = -200.0;
  double alt_value = -1000.0;
  String display_header;
  String display_lines[3] = { "", "", ""};
  uint8_t screen_count = 0;
  uint8_t screens[8];
  int line_count = 0;
  Serial.printf("Output values to display...\n");
  pm10_sensor = pm25_sensor = FPSTR("SDS011");
  pm10_value = last_value_SDS_P1;
  pm25_value = last_value_SDS_P2;
  t_sensor = p_sensor = FPSTR("BMP280");
  t_value = last_value_BMX280_T;
  p_value = last_value_BMX280_P;
  if (bmx280.sensorID() == BME280_SENSOR_ID) {
    h_sensor = t_sensor = FPSTR("BME280");
    h_value = last_value_BME280_H;
  }

  screens[screen_count++] = 1;
  screens[screen_count++] = 3;

  Serial.printf("Screens: %u\n", screen_count++);

  display_header = pm25_sensor;
  if (pm25_sensor != pm10_sensor) {
    display_header += " / " + pm10_sensor;
  }
  display_lines[0] = std::move(tmpl(F("PM2.5: {v} µg/m³"), check_display_value(pm25_value, -1, 1, 6)));
  display_lines[1] = std::move(tmpl(F("PM10: {v} µg/m³"), check_display_value(pm10_value, -1, 1, 6)));
  display_lines[2] = emptyString;


  Display.setFont(ArialMT_Plain_10);
  Display.clear();
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 1, display_header);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 16, display_lines[0]);
  Display.drawString(0, 28, display_lines[1]);
  Display.drawString(0, 40, display_lines[2]);
  Display.display();

  delay(5000);

  display_header = t_sensor;
  if (h_sensor && t_sensor != h_sensor) {
    display_header += " / " + h_sensor;
  }
  if ((h_sensor && p_sensor && (h_sensor != p_sensor)) || (h_sensor == "" && p_sensor && (t_sensor != p_sensor))) {
    display_header += " / " + p_sensor;
  }
  if (t_sensor != "") {
    display_lines[line_count] = "Temp.: ";
    display_lines[line_count] += check_display_value(t_value, -128, 1, 6);
    display_lines[line_count++] += " °C";
  }
  if (h_sensor != "") {
    display_lines[line_count] = "Hum.:  ";
    display_lines[line_count] += check_display_value(h_value, -1, 1, 6);
    display_lines[line_count++] += " %";
  }
  if (p_sensor != "") {
    display_lines[line_count] = "Pres.: ";
    display_lines[line_count] += check_display_value(p_value / 100, (-1 / 100.0), 1, 6);
    display_lines[line_count++] += " hPa";
  }
  while (line_count < 3) {
    display_lines[line_count++] = emptyString;
  }

  Display.setFont(ArialMT_Plain_10);
  Display.clear();
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 1, display_header);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 16, display_lines[0]);
  Display.drawString(0, 28, display_lines[1]);
  Display.drawString(0, 40, display_lines[2]);
  Display.display();

  delay(5000);

  yield();
  // Éteindre l'écran OLED après le premier passage
  if (screen_count == 2) {
    Display.displayOff();
  }
}


/*****************************************************************
   Init BMP280/BME280
 *****************************************************************/
static bool initBMX280(char addr) {
  if (bmx280.begin(addr)) {
    Serial.printf("begin I2C");
    bmx280.setSampling(
      BMX280::MODE_FORCED,
      BMX280::SAMPLING_X1,
      BMX280::SAMPLING_X1,
      BMX280::SAMPLING_X1);
    return true;
  } else {
    return false;
  }
}

static void powerOnTestSensors() {
  Serial.printf("Test SDS...\n", SDS_version_date());
  Serial.printf("Start SDS011...\n");
  SDS_cmd(PmSensorCmd::ContinuousMode);
  delay(100);
  Serial.printf("Stop SDS011...\n");
  is_SDS_running = SDS_cmd(PmSensorCmd::Stop);

  Serial.printf("Test BMxE280...\n");
  if (!initBMX280(0x76)) { //&& !initBMX280(0x77)) {
    Serial.printf("Check BMx280 wiring...\n");
    bmx280_init_failed = true;
  }
}


static void logEnabledDisplays() {
  Serial.printf("Show on OLED...\n");
}




/* OTAA para*/
uint8_t devEui[] = { 0x60, 0x81, 0xF9, 0xD7, 0x5E, 0x6A, 0x22, 0x1A };
uint8_t appEui[] = { 0x60, 0x81, 0xF9, 0x02, 0x20, 0xE2, 0x6E, 0x18 };
uint8_t appKey[] = { 0x48, 0xC2, 0xAE, 0xDE, 0xBA, 0xF7, 0xEA, 0x69, 0x16, 0xAD, 0x95, 0x80, 0x42, 0xA7, 0x4F, 0xB4 };
/* ABP para*/
uint8_t nwkSKey[] = {  };
uint8_t appSKey[] = {  };
uint32_t devAddr =  ( uint32_t )0x01;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/

//uint32_t appTxDutyCycle = 15000;
uint32_t appTxDutyCycle = 100000; //deduire les delais des screens ?

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = false;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };


/* Application port */
uint8_t appPort = 2;

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
  None : print basic info.
  Freq : print Tx and Rx freq, DR info.
  Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
  Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt, MCU sleep and MCU wake info.
*/

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;


static void prepareTxFrame( uint8_t port )
{

  union float_2_byte {
    float temp_float;
    byte temp_byte[4] ;
  } u;

  appDataSize = 20;//AppDataSize max value is 64

  u.temp_float = last_value_SDS_P1;

  appData[0] = u.temp_byte[0];
  appData[1] = u.temp_byte[1];
  appData[2] = u.temp_byte[2];
  appData[3] = u.temp_byte[3];

  u.temp_float = last_value_SDS_P2;

  appData[4] = u.temp_byte[0];
  appData[5] = u.temp_byte[1];
  appData[6] = u.temp_byte[2];
  appData[7] = u.temp_byte[3];

  u.temp_float = last_value_BMX280_T;

  appData[8] = u.temp_byte[0];
  appData[9] = u.temp_byte[1];
  appData[10] = u.temp_byte[2];
  appData[11] = u.temp_byte[3];

  u.temp_float = last_value_BMX280_P;

  appData[12] = u.temp_byte[0];
  appData[13] = u.temp_byte[1];
  appData[14] = u.temp_byte[2];
  appData[15] = u.temp_byte[3];

  u.temp_float = last_value_BME280_H;

  appData[16] = u.temp_byte[0];
  appData[17] = u.temp_byte[1];
  appData[18] = u.temp_byte[2];
  appData[19] = u.temp_byte[3];

Serial.printf ("HEX values:\n");
  for (int i = 0; i < 20; i++) {
    Serial.printf (" %02x", appData[i]);
    if (i == 19) {
      Serial.printf ("\n");
    }
  }

}


void setup()
{
  Serial.begin(115200);
  Mcu.begin();
  LoRaWAN.displayMcuInit();
  serialSDS.begin(9600, SERIAL_8N1, PM_SERIAL_RX, PM_SERIAL_TX);
  serialSDS.setTimeout((4 * 12 * 1000) / 9600);

  Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);
  delay(100);
  Wire.setClock(100000);

  uint64_t chipid_num;
  chipid_num = ESP.getEfuseMac();
  esp_chipid = String((uint16_t)(chipid_num >> 32), HEX);
  esp_chipid += String((uint32_t)chipid_num, HEX);
  Serial.printf("ESP32ChipID=%u\n", chipid_num);
  Serial.printf("ESP32ChipID=%04X", (uint16_t)(chipid_num >> 32)); //print High 2 bytes
  Serial.printf("%08X\n", (uint32_t)chipid_num); //print Low 4bytes.
  Serial.printf("Starting...\n");
  Display.init();
  delay(100);
#ifdef Wireless_Stick
  Display.setFont(ArialMT_Plain_10);
#else
  Display.setFont(ArialMT_Plain_16);
#endif
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.clear();
#ifdef Wireless_Stick
  Display.drawString(32, 40, esp_chipid);
#else
  Display.drawString(58, 22, esp_chipid);
#endif
  Display.display();
  delay(1000);

  powerOnTestSensors();
  logEnabledDisplays();

  delay(50);

  starttime = millis();                 // store the start time
  time_point_device_start_ms = starttime;
  last_display_millis = starttime_SDS = starttime;
  start_SDS = true;

  delay(100);
  SPI.begin(SCK, MISO, MOSI, SS);
  deviceState = DEVICE_STATE_INIT;
  delay(1000);
}

void loop()
{
  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
        LoRaWAN.init(loraWanClass, loraWanRegion);
        Serial.printf("After init state: %u \n", deviceState);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.displayJoining();
        LoRaWAN.join();
        Serial.printf("After join state: %u \n", deviceState);
        break;
      }
    case DEVICE_STATE_SEND:
      {

        unsigned sum_send_time = 0;

        act_micro = micros();
        act_milli = millis();

        fetchSensorSDS();

        last_display_millis = act_milli;


        yield();

        if (SDS_read_OK == true ) {

          if (! bmx280_init_failed) {
            fetchSensorBMX280();
          }

          display_values();
          LoRaWAN.displaySending();
          prepareTxFrame( appPort );
          LoRaWAN.send();
          deviceState = DEVICE_STATE_CYCLE;

          yield();

          lowpulseoccupancyP1 = 0;
          lowpulseoccupancyP2 = 0;
          sample_count = 0;
          last_micro = 0;
          min_micro = 1000000000;
          max_micro = 0;
          sum_send_time = 0;
          starttime = millis();               // store the start time
          count_sends++;
          SDS_read_OK = false;
          start_SDS = true;

          // only do a restart after finishing sending
          if (msSince(time_point_device_start_ms) > DURATION_BEFORE_FORCED_RESTART_MS) {
            sensor_restart();
          }
          Serial.printf("After send state: %u \n", deviceState);
        }
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        Serial.printf("After cycle state: %u \n", deviceState);
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
  if (sample_count % 500 == 0) {
    //    Serial.println(ESP.getFreeHeap(),DEC);
  }
}
