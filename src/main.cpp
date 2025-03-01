#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Ticker.h> 
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <RTCMemory.h>
#include <SPI.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_ADS1X15.h>
#include <pcf8574_esp.h>
#include "DHTesp.h"
#include <M2M_LM75A.h>
#include "LittleFS.h"

#define DEVICE_NAME         "iot-monitor-1"
#define WIFI_SSID           "LAN_2.4G"
#define WIFI_PASSWORD       "**********"
#define PRIMARY_NTP_SERVER  "nl.pool.ntp.org"
#define BACKUP_NTP_SERVER   "igubu.saix.net"
#define MQTT_BROKER         "192.168.0.2"
#define TIMEZONE_OFFSET     7200
#define NTP_UPDATE          86400
#define BAT_VOLTAGE_CAL     0.96 

#define BMP280_CONNECTED    false
#define BMP180_CONNECTED    true
#define AHT10_CONNECTED     false
#define LM75_CONNECTED      true
#define ADC_CONNECTED       false
#define AM2320_CONNECTED    true
#define DHT_CONNECTED       false
#define PULSE_CONNECTED     false
#define SLEEP_MODE_ACTIVE   true
#define LEGACY_DEVICE       true

#define PCF8574_ADDRESS     0x3A
#define LM75_ADDRESS        0x49
#define BMP280_ADDRESS      0x77
#define ADC_ADDRESS         0x48
#define AHT20_ADDRESS       0x38
#define IO0_PIN             0
#define IO2_PIN             2
#define LED_PIN             4
#define IO6_PIN             5
#define IO15_PIN            15
#define SCL_PIN             12
#define SDA_PIN             13
#define INT_PIN             14

typedef struct{
    char hostname[20];
    char ssid[32];
    char password[32];
    char mqtt_broker[16];
    char topic[60];
    float voltage_cal;
    int ip[4];
    int gateway[4];
    int subnet[4];
    int dns[4];
    int mqtt_port;
    int use_dhcp;
    int collection_interval; //Time between transmit intervals where data is uploaded
    int discovery_interval; //Time between data collection intervals
    int persist_cumulative_interval;
    int timeout;
    int time; //Current time
    float pulse_cumulative_total_energy;
    float cumulative_total_energy1;
    float cumulative_total_energy2;
    float cumulative_total_energy3;
    bool mqtt; //Use MQTT or Rest.
    bool valid_wifi;
}settings_struct;

typedef struct{
    #if ADC_CONNECTED
        float grid_voltage;
        float ct1_current;
        float ct2_current;
        float ct3_current;
        float ct1_power;
        float ct2_power;
        float ct3_power;
        float ct1_pf;
        float ct2_pf;
        float ct3_pf;
        float grid_frequency;
        float cumulative_total_energy1;
        float cumulative_total_energy2;
        float cumulative_total_energy3;
    #endif

    #if BMP280_CONNECTED
        float bmp280_temperature;
        float bmp280_pressure;
    #endif

    #if BMP180_CONNECTED
        float bmp180_temperature;
        float bmp180_pressure;
    #endif

    #if AHT10_CONNECTED
        float aht10_temperature;
        float aht10_humidity;
    #endif

    #if PULSE_CONNECTED
        float pulse_detector_power;
        float pulse_cumulative_total_energy;
    #endif

    #if LM75_CONNECTED
         float lm75_temperature;
    #endif

    #if AM2320_CONNECTED
        float AM2320_temperature;
        float AM2320_humidity;
    #endif

    #if DHT_CONNECTED
        float dht_temperature;
        float dht_humidity;
    #endif

    float battery_voltage;
    float rssi;
    String ip_address;
    String ssid;
    int last_update_time;
    int startup_time;
}data_struct;

#if SLEEP_MODE_ACTIVE
typedef struct{
     #if ADC_CONNECTED
        float cumulative_total_energy1;
        float cumulative_total_energy2;
        float cumulative_total_energy3;
    #endif

    #if PULSE_CONNECTED
        float pulse_cumulative_total_energy;
    #endif

    int run_count;
}rtc_data_struct;
#endif

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, PRIMARY_NTP_SERVER, TIMEZONE_OFFSET, NTP_UPDATE);
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient pubsubClient(espClient);
PCF857x pcf8574(PCF8574_ADDRESS, &Wire);

#if SLEEP_MODE_ACTIVE
    RTCMemory<rtc_data_struct> rtcMemory;
    rtc_data_struct initial_struct = {
        #if ADC_CONNECTED
            0.0,
            0.0,
            0.0,
        #endif
        #if PULSE_CONNECTED
            0.0,
        #endif
        0
        };
    rtc_data_struct *rtc_data = &initial_struct;
#endif

#if DHT_CONNECTED
    DHTesp dht;
#endif
#if AHT10_CONNECTED
    Adafruit_AHTX0 aht;
#endif
#if AM2320_CONNECTED
    Adafruit_AM2320 am2320 = Adafruit_AM2320(&Wire);
#endif
#if BMP280_CONNECTED
    Adafruit_BMP280 bmp; // use I2C interface
    Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
    Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
#endif
#if BMP180_CONNECTED
    Adafruit_BMP085 bmp;
#endif
#if ADC_CONNECTED
    Adafruit_ADS1115 ads1115;
#endif
#if LM75_CONNECTED
    M2M_LM75A lm75a;
#endif

settings_struct settings = {
    DEVICE_NAME,
    WIFI_SSID,
    WIFI_PASSWORD,
    MQTT_BROKER,
    DEVICE_NAME,
    BAT_VOLTAGE_CAL,
    {192,168,192,101},
    {192,168,192,1},
    {255,255,255,0},
    {192,168,192,1},
    8888,
    0,
    300,
    600,
    43200,
    30,
    0,
    0.0,
    0.0,
    0.0,
    0.0,
    true,
    true
};


String error_txt = "";
data_struct data;
volatile bool test_mode = false;
bool am2320_connected = AM2320_CONNECTED;
bool bmp280_connected = BMP280_CONNECTED;
bool bmp180_connected = BMP180_CONNECTED;
bool dht22_connected = DHT_CONNECTED;
bool aht10_connected = AHT10_CONNECTED;
bool lm75_connected = LM75_CONNECTED;

bool debug_enabled = true;
int run_count = 0;
int pulse_cnt = 0;

void debug_print(String text){
    if(debug_enabled){
        char current_buf[] = "YY/MM/DD hh:mm:ss";
        DateTime current_time(timeClient.getEpochTime());
        
        Serial.println(String(current_time.toString(current_buf)) + " - " + text);
    }
}

void log_error(String text){
    char current_buf[] = "YY/MM/DD hh:mm:ss";
    DateTime current_time(timeClient.getEpochTime());

    error_txt = String(current_time.toString(current_buf)) + " - " + text;
}

void ICACHE_RAM_ATTR PCFInterrupt() {
}

void ICACHE_RAM_ATTR BTInterrupt() {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 300)
    {
        test_mode = !test_mode;

        if(test_mode){
            led.setBrightness(100);
            led.setPixelColor(0, 0XFF00FF);
            debug_print("Entering debug mode");
        }
        else{
            led.setBrightness(10);
            led.setPixelColor(0, 0X0000FF);
            debug_print("Leaving debug mode");
        }

        led.show();
    }
    last_interrupt_time = interrupt_time;
}

#if PULSE_CONNECTED
    void ICACHE_RAM_ATTR PulseInterrupt() {
        static unsigned long last_interrupt_time = 0;
        unsigned long interrupt_time = millis();
        if (interrupt_time - last_interrupt_time > 100)
        {
            pulse_cnt++;
            data.pulse_cumulative_total_energy += 0.001;
        }
        last_interrupt_time = interrupt_time;
    }
#endif

void setLed(int index, int brightness, int colour){
    if(brightness > 50)
        brightness = 50;
    led.setBrightness(brightness);
    led.setPixelColor(0, colour);
    led.show();
}

int checkWifi(){
    debug_print("Checking Wifi");
    int connection_time = 0;

    if(WiFi.status() != WL_CONNECTED)
    {
        debug_print("Wifi Disconnected");
        log_error("Wifi Disconnected");
    }

    while (WiFi.status() != WL_CONNECTED) {
        if(connection_time == settings.timeout){
            setLed(0, 10, 0XFF0000);
            settings.valid_wifi = false;
            EEPROM.put(0x0, settings);
            EEPROM.commit();
            ESP.restart();
            break;
        }
        delay(1000);
        connection_time++;
        if(connection_time%2 == 0)
        {
            setLed(0, 10, 0X00FF00);
        }
        else
        {
            setLed(0, 10, 0X000000);
        }
    }

    setLed(0, 10, 0X00FF00);
    if(settings.valid_wifi == false){
        settings.valid_wifi = true;
        EEPROM.put(0x0, settings);
        EEPROM.commit();
    }
    debug_print("Wifi Connected - " + WiFi.localIP().toString());
    return 1;
}

void checkMQTT()
{
    debug_print("Checking MQTT");
    checkWifi();
    int connection_time = 0;

    if(!pubsubClient.setBufferSize(512))
        setLed(0, 10, 0XFF0000);

    while (!pubsubClient.connected()) 
    {
        if(connection_time == settings.timeout){
            setLed(0, 10, 0XFF0000);
            debug_print("MQTT Disconnected");
            log_error("MQTT Disconnected");
            return;
        }
    
        if (!pubsubClient.connect(settings.hostname)) 
        {
            delay(1000);
            connection_time++;
        } 
    }

    debug_print("MQTT Connected");
}

void startOTA(){
    ArduinoOTA.onStart([]() 
    {
        setLed(0, 100, 0XFF00FF);
    });
    ArduinoOTA.onEnd([]() 
    {
        setLed(0, 100, 0X00FFFF);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
    {
        int colour = 255.0 * float(progress) / float(total);
        setLed(0, 100, ((255 - colour)) + (colour << 8));
    });
    ArduinoOTA.onError([](ota_error_t error) 
    {
        if (error == OTA_AUTH_ERROR) {
            log_error("OTA Auth Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_BEGIN_ERROR) {
            log_error("OTA Begin Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_CONNECT_ERROR) {
            log_error("OTA Connect Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_RECEIVE_ERROR) {
            log_error("OTA Receive Error");
            setLed(0, 100, 0XFF0000);
        } else if (error == OTA_END_ERROR) {
            log_error("OTA End Error");
            setLed(0, 100, 0XFF0000);
        }
    });
    ArduinoOTA.setRebootOnSuccess(true);
    ArduinoOTA.setHostname(settings.hostname);
    ArduinoOTA.begin();
}

float read_voltage(int samples){
    float cumul_voltage = 0.0;
    for(int i = 0 ; i < samples ; i++){
        cumul_voltage += analogRead(A0) / 1024.0 * 5.555 * settings.voltage_cal;
        delay(10);
    }
    return cumul_voltage/samples;
}

void getData(){
    #if ADC_CONNECTED
        //do adc stuff here
    #endif

    #if BMP280_CONNECTED
        if(bmp280_connected){
            sensors_event_t temp_event, pressure_event;
            bmp_temp->getEvent(&temp_event);
            bmp_pressure->getEvent(&pressure_event);
        
            data.bmp280_temperature = temp_event.temperature;
            data.bmp280_pressure = pressure_event.pressure;
        }
    #endif

    #if BMP180_CONNECTED
        if(bmp180_connected){
            data.bmp180_temperature = bmp.readTemperature();
            data.bmp180_pressure = bmp.readPressure()/100.0;
        }
    #endif

    #if AHT10_CONNECTED
        if(aht10_connected){
            sensors_event_t humidity, temp;
    
            aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
            data.aht10_temperature = temp.temperature;
            data.aht10_humidity = humidity.relative_humidity;
        }
    #endif

    #if LM75_CONNECTED
        if(lm75_connected){
            data. lm75_temperature = lm75a.getTemperature();
        }
    #endif

    #if AM2320_CONNECTED
        if(am2320_connected){
            data.AM2320_humidity = isnan(am2320.readHumidity()) ? 0 : am2320.readHumidity();
            data.AM2320_temperature = isnan(am2320.readTemperature()) ? 0 : am2320.readTemperature();
        }
    #endif

    #if DHT_CONNECTED
        if(dht22_connected){
            data.dht_humidity = dht.getHumidity();
            data.dht_temperature = dht.getTemperature();
        }
    #endif

    #if PULSE_CONNECTED
        static int prev_timestamp = timeClient.getEpochTime() - settings.collection_interval;
        int current_timestamp = timeClient.getEpochTime();

        data.pulse_detector_power = pulse_cnt * 3600.0/(current_timestamp - prev_timestamp);
        prev_timestamp = current_timestamp;
        pulse_cnt = 0;
    #endif

    data.battery_voltage = read_voltage(10);
    data.ip_address = WiFi.localIP().toString();
    data.rssi = WiFi.RSSI();
    data.ssid = WiFi.SSID();

    debug_print(String(data.rssi));
    debug_print(data.ssid);
    debug_print(data.ip_address);

    debug_print("Done Getting Data");
}

void sendData(){
    String device_name = String(settings.hostname);
    String topic = "";

    #if ADC_CONNECTED
        topic = device_name + "/grid_voltage"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.grid_voltage)).c_str(), String(data.grid_voltage).length());

        topic = device_name + "/grid_frequency"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.grid_frequency)).c_str(), String(data.grid_frequency).length());

        topic = device_name + "/ct1_current"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct1_current)).c_str(), String(data.ct1_current).length());

        topic = device_name + "/ct1_power"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct1_power)).c_str(), String(data.ct1_power).length());

        topic = device_name + "/ct1_pf"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct1_pf)).c_str(), String(data.ct1_pf).length());

        topic = device_name + "/ct2_current"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct2_current)).c_str(), String(data.ct2_current).length());

        topic = device_name + "/ct2_power"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct2_power)).c_str(), String(data.ct2_power).length());
        
        topic = device_name + "/ct2_pf"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct2_pf)).c_str(), String(data.ct2_pf).length());

        topic = device_name + "/ct3_current"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct3_current)).c_str(), String(data.ct3_current).length());

        topic = device_name + "/ct3_power"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct3_power)).c_str(), String(data.ct3_power).length());
        
        topic = device_name + "/ct3_pf"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.ct3_pf)).c_str(), String(data.ct3_pf).length());

        topic = device_name + "/cumulative_total_energy1"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.cumulative_total_energy1)).c_str(), String(data.cumulative_total_energy1).length());

        topic = device_name + "/cumulative_total_energy2"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.cumulative_total_energy2)).c_str(), String(data.cumulative_total_energy2).length());

        topic = device_name + "/cumulative_total_energy3"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.cumulative_total_energy3)).c_str(), String(data.cumulative_total_energy3).length());
    #endif
    
    #if BMP280_CONNECTED
        topic = device_name + "/bmp280_temperature"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.bmp280_temperature)).c_str(), String(data.bmp280_temperature).length());

        topic = device_name + "/bmp280_pressure"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.bmp280_pressure)).c_str(), String(data.bmp280_pressure).length());
    #endif

    #if BMP180_CONNECTED
        topic = device_name + "/bmp180_temperature"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.bmp180_temperature)).c_str(), String(data.bmp180_temperature).length());

        topic = device_name + "/bmp180_pressure"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.bmp180_pressure)).c_str(), String(data.bmp180_pressure).length());
    #endif

    #if AHT10_CONNECTED
        topic = device_name + "/aht10_temperature"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.aht10_temperature)).c_str(), String(data.aht10_temperature).length());

        topic = device_name + "/aht10_humidity"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.aht10_humidity)).c_str(), String(data.aht10_humidity).length());
    #endif

    #if PULSE_CONNECTED
        topic = device_name + "/pulse_detector_power"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.pulse_detector_power)).c_str(), String(data.pulse_detector_power).length());

        topic = device_name + "/pulse_cumulative_total_energy"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.pulse_cumulative_total_energy)).c_str(), String(data.pulse_cumulative_total_energy).length());
    #endif


    #if LM75_CONNECTED
        topic = device_name + "/lm75_temperature"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.lm75_temperature)).c_str(), String(data.lm75_temperature).length());
    #endif

    #if AM2320_CONNECTED
        if(!isnan(data.AM2320_temperature))
        {
            topic = device_name + "/am2320_temperature"; 
            checkMQTT();
            pubsubClient.publish(topic.c_str(), (String(data.AM2320_temperature)).c_str(), String(data.AM2320_temperature).length());
            debug_print(String(data.AM2320_temperature));
        }

        if(!isnan(data.AM2320_humidity))
        {
            topic = device_name + "/am2320_humidity"; 
            checkMQTT();
            pubsubClient.publish(topic.c_str(), (String(data.AM2320_humidity)).c_str(), String(data.AM2320_humidity).length());
        }
    #endif

    #if DHT_CONNECTED
        topic = device_name + "/dht_temperature"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.dht_temperature)).c_str(), String(data.dht_temperature).length());

        topic = device_name + "/dht_humidity"; 
        checkMQTT();
        pubsubClient.publish(topic.c_str(), (String(data.dht_humidity)).c_str(), String(data.dht_humidity).length());
    #endif

    topic = device_name + "/battery_voltage"; 
    checkMQTT();
    pubsubClient.publish(topic.c_str(), (String(data.battery_voltage)).c_str(), String(data.battery_voltage).length());

    topic = device_name + "/ip_address"; 
    checkMQTT();
    pubsubClient.publish(topic.c_str(), (String(data.ip_address)).c_str(), String(data.ip_address).length());

    topic = device_name + "/rssi"; 
    checkMQTT();
    pubsubClient.publish(topic.c_str(), (String(data.rssi)).c_str(), String(data.rssi).length());

    topic = device_name + "/ssid"; 
    checkMQTT();
    pubsubClient.publish(topic.c_str(), (String(data.ssid)).c_str(), String(data.ssid).length());

    debug_print("Done Sending Data");
}

void sendMQTTDiscovery(){
    debug_print("Sending Discovery");
    String base[] = {
        #if ADC_CONNECTED
            "grid_voltage",
            "ct1_current",
            "ct2_current",
            "ct3_current",
            "ct1_power",
            "ct2_power",
            "ct3_power",
            "ct1_pf",
            "ct2_pf",
            "ct3_pf",
            "grid_frequency",
            "cumulative_total_energy1",
            "cumulative_total_energy2",
            "cumulative_total_energy2",
        #endif

        #if BMP280_CONNECTED
            "bmp280_temperature",
            "bmp280_pressure",
        #endif

        #if BMP180_CONNECTED
            "bmp180_temperature",
            "bmp180_pressure",
        #endif

        #if AHT10_CONNECTED
            "aht10_temperature",
            "aht10_humidity",
        #endif
        
        #if PULSE_CONNECTED
            "pulse_detector_power",
            "pulse_cumulative_total_energy",
        #endif

        #if LM75_CONNECTED
            "lm75_temperature",
        #endif

        #if AM2320_CONNECTED
            "AM2320_temperature",
            "AM2320_humidity",
        #endif

        #if DHT_CONNECTED
            "dht_temperature",
            "dht_humidity",
        #endif
        
        "battery_voltage",
        "ip_address",
        "rssi",
        "ssid"
    };

    String device_name = String(settings.hostname);

    for(unsigned int i = 0 ; i < ((sizeof(base))/(sizeof(String))) ; i++)
    {
        const size_t capacity = (21*JSON_ARRAY_SIZE(1) + 2*JSON_OBJECT_SIZE(1) + 2*20*JSON_OBJECT_SIZE(2) + 
                            JSON_OBJECT_SIZE(3) + 5*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5))*2;
        DynamicJsonDocument dataObject(capacity);

        String underscored = base[i];
        underscored.toLowerCase();
        underscored.replace(" ", "_");

        String topic = "homeassistant/sensor/" + device_name + "/" + underscored + "/config";

        dataObject["unique_id"] = device_name + "_" + underscored;
        dataObject["name"] = device_name + " " + base[i];

        if(underscored.lastIndexOf("voltage") != -1)
        {
            dataObject["device_class"] =  "voltage";
            dataObject["unit_of_measurement"] = "V";
        }

        if(underscored.lastIndexOf("current") != -1)
        {
            dataObject["device_class"] =  "current";
            dataObject["unit_of_measurement"] = "A";
        }

        if(underscored.lastIndexOf("power") != -1)
        {
            dataObject["device_class"] =  "power";
            dataObject["unit_of_measurement"] = "W";
        }

        if(underscored.lastIndexOf("frequency") != -1)
        {
            dataObject["device_class"] =  "frequency";
            dataObject["unit_of_measurement"] = "Hz";
        }

        if(underscored.lastIndexOf("temp") != -1)
        {
            dataObject["device_class"] =  "temperature";
            dataObject["unit_of_measurement"] = "°C";
        }

        if(underscored.lastIndexOf("humid") != -1)
        {
            dataObject["device_class"] =  "humidity";
            dataObject["unit_of_measurement"] = "%";
        }

        if(underscored.lastIndexOf("press") != -1)
        {
            dataObject["device_class"] =  "pressure";
            dataObject["unit_of_measurement"] = "hPa";
        }

        if(underscored.lastIndexOf("total") != -1)
        {
            dataObject["state_class"] = "total";
            dataObject["device_class"] =  "energy";
            dataObject["unit_of_measurement"] = "kWh";
        }

        if(underscored.lastIndexOf("load") != -1)
        {
            dataObject["device_class"] =  "battery";
            dataObject["unit_of_measurement"] = "%";
        }

        if(underscored.lastIndexOf("rssi") != -1)
        {
            dataObject["device_class"] =  "signal_strength";
            dataObject["unit_of_measurement"] = "dBm";
        }
        
        dataObject["json_attributes_topic"] = device_name + "/" + underscored + "/attr";
        dataObject["state_topic"] = device_name + "/" + underscored;
        
        JsonObject device = dataObject.createNestedObject("device");

        device["manufacturer"] = "Orion Electronics";
        device["model"] = "ESP8266-IOT-1";
        device["name"] = "ESP IoT Monitor";
        device["sw_version"] = "1.0";

        JsonArray identifiers = device.createNestedArray("identifiers");
        identifiers.add(device_name);

        String json_data = "";
        serializeJson(dataObject, json_data);
        checkMQTT();
        pubsubClient.publish(topic.c_str(), json_data.c_str(), json_data.length());
        debug_print(json_data);
    }
    debug_print("Done Sending Discovery");
}

void power_down(int seconds){
    debug_print(String("Execution completed in: ") + String(millis()/1000.0) + String(" Seconds"));
    debug_print("Going to sleep for " + String(seconds) + " seconds");
    
    pcf8574.write8(0xEF);

    digitalWrite(SCL_PIN, HIGH);
    digitalWrite(SDA_PIN, HIGH);
    digitalWrite(IO6_PIN, HIGH);
    digitalWrite(IO0_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(IO15_PIN, LOW);

    #if LEGACY_DEVICE
        digitalWrite(IO2_PIN, HIGH);
    #endif

    delay(10);
    ESP.deepSleep(seconds * 1e6);
}

void mqtt_receive(char *topic, byte *payload, unsigned int length) {
}

String processor(const String& var){
    char current_buf[] = "YY/MM/DD hh:mm:ss";
    char startup_buf[] = "YY/MM/DD hh:mm:ss";
    char update_buf[] = "YY/MM/DD hh:mm:ss";
    int cur_time = timeClient.getEpochTime();
    int uptime = cur_time - data.startup_time;
    int uptime_days = uptime/(86400);
    int uptime_hours = (uptime%86400)/3600;
    int uptime_min = ((uptime%86400)%3600)/60;
    int uptime_sec = ((uptime%86400)%3600)%60;

    DateTime current_time(cur_time);
    DateTime startup_time(data.startup_time);
    DateTime last_update_time(data.last_update_time);

    if(var == "host"){
        return settings.hostname;
    }
    if(var == "ssid"){
        return settings.ssid;
    }
    if(var == "password"){
        return settings.password;
    }
    if(var == "mqtt_host"){
        return settings.mqtt_broker;
    }
    if(var == "mqtt_port"){
        return String(settings.mqtt_port);
    }
    if(var == "mqtt_topic"){
        return settings.topic;
    }
    if(var == "interval"){
        return String(settings.collection_interval);
    }
    if(var == "discovery"){
        return String(settings.discovery_interval);
    }
    if(var == "wifi_timeout"){
        return String(settings.timeout);
    }
    if(var == "persist_cumulative_interval"){
        return String(settings.persist_cumulative_interval);
    }
    if(var == "voltage_cal"){
        return String(settings.voltage_cal);
    }

    #if ADC_CONNECTED
        if(var == "grid_voltage"){
            return String(data.grid_voltage);
        }
        if(var == "ct1_current"){
            return String(data.ct1_current);
        }
        if(var == "ct2_current"){
            return String(data.ct2_current);
        }
        if(var == "ct3_current"){
            return String(data.ct3_current);
        }
        if(var == "ct1_power"){
            return String(data.ct1_power);
        }
        if(var == "ct2_power"){
            return String(data.ct2_power);
        }
        if(var == "ct3_power"){
            return String(data.ct3_power);
        }
        if(var == "ct1_pf"){
            return String(data.ct1_pf);
        }
        if(var == "ct2_pf"){
            return String(data.ct2_pf);
        }
        if(var == "ct3_pf"){
            return String(data.ct3_pf);
        }
        if(var == "grid_frequency"){
            return String(data.grid_frequency);
        }
        if(var == "cumulative_total_energy1"){
            return String(data.cumulative_total_energy1);
        }
        if(var == "cumulative_total_energy2"){
            return String(data.cumulative_total_energy2);
        }
        if(var == "cumulative_total_energy3"){
            return String(data.cumulative_total_energy3);
        }
    #endif

    #if BMP280_CONNECTED
        if(var == "bmp280_pressure"){
            return String(data.bmp280_pressure);
        }
        if(var == "bmp280_temperature"){
            return String(data.bmp280_temperature);
        }
    #endif

    #if BMP180_CONNECTED
        if(var == "bmp180_temperature"){
            return String(data.bmp180_temperature);
        }
        if(var == "bmp180_pressure"){
            return String(data.bmp180_pressure);
        }
    #endif

    #if AHT10_CONNECTED
        if(var == "aht10_temperature"){
            return String(data.aht10_temperature);
        }
        if(var == "aht10_humidity"){
            return String(data.aht10_humidity);
        }
    #endif

    #if PULSE_CONNECTED
        if(var == "pulse_detector_power"){
            return String(data.pulse_detector_power);
        }
        if(var == "pulse_cumulative_total_energy"){
            return String(data.pulse_cumulative_total_energy);
        }
    #endif

    #if LM75_CONNECTED
        if(var == "lm75_temperature"){
            return String(data.lm75_temperature);
        }
    #endif

    #if AM2320_CONNECTED
        if(var == "AM2320_temperature"){
            return String(data.AM2320_temperature);
        }
        if(var == "AM2320_humidity"){
            return String(data.AM2320_humidity);
        }
    #endif

    #if DHT_CONNECTED
        if(var == "dht_temperature"){
            return String(data.dht_temperature);
        }
        if(var == "dht_humidity"){
            return String(data.dht_humidity);
        }
    #endif

    if(var == "battery_voltage"){
        return String(data.battery_voltage);
    }

    if(var == "current_time"){
        return String(current_time.toString(current_buf));
    }
    if(var == "Last_update_time"){
        return String(last_update_time.toString(update_buf));
    }
    if(var == "startup_time"){
        return String(startup_time.toString(startup_buf));
    }
    if(var == "uptime"){
        return String(uptime_days) + "d " + String(uptime_hours) + "h " + String(uptime_min) + "m " + String(uptime_sec) + "s";
    }
    return String();
}

String api_data() {
    char current_buf[] = "YY/MM/DD hh:mm:ss";
    char startup_buf[] = "YY/MM/DD hh:mm:ss";
    char update_buf[] = "YY/MM/DD hh:mm:ss";
    int cur_time = timeClient.getEpochTime();
    int uptime = cur_time - data.startup_time;
    int uptime_days = uptime/(86400);
    int uptime_hours = (uptime%86400)/3600;
    int uptime_min = ((uptime%86400)%3600)/60;
    int uptime_sec = ((uptime%86400)%3600)%60;

    DateTime current_time(cur_time);
    DateTime startup_time(data.startup_time);
    DateTime last_update_time(data.last_update_time);

    String message = "{";

    #if ADC_CONNECTED
        message += "\"grid_voltage\":\"" + String(data.grid_voltage, 1) + "\",";
        message += "\"ct1_current\":\"" + String(data.ct1_current, 1) + "\",";
        message += "\"ct2_current\":\"" + String(data.ct2_current, 1) + "\",";
        message += "\"ct3_current\":\"" + String(data.ct3_current, 1) + "\",";
        message += "\"ct1_power\":\"" + String(data.ct1_power, 1) + "\",";
        message += "\"ct2_power\":\"" + String(data.ct2_power, 1) + "\",";
        message += "\"ct3_power\":\"" + String(data.ct3_power, 1) + "\",";
        message += "\"ct1_pf\":\"" + String(data.ct1_pf, 1) + "\",";
        message += "\"ct2_pf\":\"" + String(data.ct2_pf, 1) + "\",";
        message += "\"ct3_pf\":\"" + String(data.ct3_pf, 1) + "\",";
        message += "\"grid_frequency\":\"" + String(data.grid_frequency, 1) + "\",";
        message += "\"cumulative_total_energy1\":\"" + String(data.cumulative_total_energy1, 1) + "\",";
        message += "\"cumulative_total_energy2\":\"" + String(data.cumulative_total_energy2, 1) + "\",";
        message += "\"cumulative_total_energy3\":\"" + String(data.cumulative_total_energy3, 1) + "\",";
    #endif

    #if BMP280_CONNECTED
        message += "\"bmp280_temperature\":\"" + String(data.bmp280_temperature, 1) + "\",";
        message += "\"bmp280_pressure\":\"" + String(data.bmp280_pressure, 1) + "\",";
    #endif

    #if BMP180_CONNECTED
        message += "\"bmp180_temperature\":\"" + String(data.bmp180_temperature, 1) + "\",";
        message += "\"bmp180_pressure\":\"" + String(data.bmp180_pressure, 1) + "\",";
    #endif

    #if AHT10_CONNECTED
        message += "\"aht10_temperature\":\"" + String(data.aht10_temperature, 1) + "\",";
        message += "\"aht10_humidity\":\"" + String(data.aht10_humidity, 1) + "\",";
    #endif

    #if PULSE_CONNECTED
        message += "\"pulse_detector_power\":\"" + String(data.pulse_detector_power, 1) + "\",";
        message += "\"pulse_cumulative_total_energy\":\"" + String(data.pulse_cumulative_total_energy, 1) + "\",";
    #endif

    #if LM75_CONNECTED
        message += "\"lm75_temperature\":\"" + String(data.lm75_temperature, 1) + "\",";
    #endif

    #if AM2320_CONNECTED
        message += "\"AM2320_temperature\":\"" + String(data.AM2320_temperature, 1) + "\",";
        message += "\"AM2320_humidity\":\"" + String(data.AM2320_humidity, 1) + "\",";
    #endif

    #if DHT_CONNECTED
        message += "\"dht_temperature\":\"" + String(data.dht_temperature, 1) + "\",";
        message += "\"dht_humidity\":\"" + String(data.dht_humidity, 1) + "\",";
    #endif

    message += "\"battery_voltage\":\"" + String(data.battery_voltage, 2) + "\",";

    message += "\"current_time\":\"" + String(current_time.toString(current_buf)) + "\",";
    message += "\"last_update_time\":\"" + String(last_update_time.toString(update_buf)) + "\",";
    message += "\"startup_time\":\"" + String(startup_time.toString(startup_buf)) + "\",\"uptime\": \"" + String(uptime_days) + "d " + String(uptime_hours) + "h " + String(uptime_min) + "m " + String(uptime_sec) + "s\"";

    message += "}";
    
    return message;
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    EEPROM.begin(512);
    EEPROM.get(0x0, settings); 
    
    if(isnan(settings.timeout)){
        debug_print("Error reading settings from EEPROM");
        EEPROM.put(0x0, settings);
        EEPROM.commit();
    }

    debug_print("");
    debug_print("Device boot.");
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(IO6_PIN, OUTPUT);
    pinMode(IO15_PIN, INPUT);
    
    pinMode(IO0_PIN, INPUT);
    pinMode(INT_PIN, INPUT_PULLUP);

    #if LEGACY_DEVICE
        pinMode(IO2_PIN, OUTPUT);
        digitalWrite(IO2_PIN, LOW);
    #else
        pinMode(IO2_PIN, INPUT);
    #endif

    digitalWrite(IO0_PIN, HIGH);
    digitalWrite(IO6_PIN, HIGH);

    pcf8574.begin();
    pcf8574.resetInterruptPin();
    pcf8574.write8(0x00);

    led.begin();
    led.clear();
    led.setBrightness(10);
    led.setPixelColor(0, 0X0000FF);
    led.show();

    attachInterrupt(digitalPinToInterrupt(INT_PIN), PCFInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(IO0_PIN), BTInterrupt, FALLING);  

    #if PULSE_CONNECTED
        attachInterrupt(digitalPinToInterrupt(IO2_PIN), PulseInterrupt, FALLING); 
        attachInterrupt(digitalPinToInterrupt(IO15_PIN), PulseInterrupt, FALLING);
    #endif

    if(!LittleFS.begin()){
        debug_print("An Error has occurred while mounting SPIFFS");
    }

    if(settings.valid_wifi){
        debug_print("Trying old wifi credentials");
        debug_print("Connecting to: " + String(settings.ssid));
        WiFi.mode(WIFI_STA);
        WiFi.setPhyMode(WIFI_PHY_MODE_11G);
        WiFi.hostname(settings.hostname);
        WiFi.begin(settings.ssid, settings.password);

        checkWifi();
        timeClient.forceUpdate();
    }
    else{
        debug_print("No valid wifi credentials");
        IPAddress local_IP(192,168,0,1);
        IPAddress gateway(192,168,0,1);
        IPAddress subnet(255,255,255,0);

        WiFi.softAPConfig(local_IP, gateway, subnet);
        WiFi.softAP(settings.hostname, "12345678");
        debug_print(WiFi.softAPIP().toString());
    }

    timeClient.update();
    int retry_count[] = {3, 1};

    while(!timeClient.isTimeSet())
    {
        if(retry_count[0] == 0)
        {
            if(retry_count[1] == 0)
            {
                ESP.restart();
            }
            checkWifi();
            log_error("NTP Error: Using backup server.");
            timeClient.setPoolServerName(BACKUP_NTP_SERVER);
            retry_count[0] = 3;
            retry_count[1]--;
        }

        delay(3000);
        timeClient.forceUpdate();
        retry_count[0]--;
    }

    #if ADC_CONNECTED
        ads1115.begin(ADC_ADDRESS);
        ads1115.setGain(GAIN_ONE);
    #endif

    #if AM2320_CONNECTED
        am2320.begin();
        if(am2320.readHumidity() == NAN)
        {
            am2320_connected = false;
            Serial.println("Could not find a valid AM2320 sensor.");
        }
    #endif

    #if DHT_CONNECTED
        dht.setup(IO6_PIN, DHTesp::DHT22);

        delay(dht.getMinimumSamplingPeriod());
        dht.getTemperature();
        if(!String(dht.getStatusString()).equals("OK")){
            dht22_connected = false;
            Serial.println("Could not find a valid DHT sensor.");
        }
    #endif

    #if BMP280_CONNECTED
        if (!bmp.begin()) {
            Serial.println(F("Could not find a valid BMP280 sensor."));
            bmp280_connected = false;
        }
        else{
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        }
    #endif

    #if BMP180_CONNECTED
        if (!bmp.begin()) {
            Serial.println(F("Could not find a valid BMP180 sensor."));
            bmp180_connected = false;
        }
    #endif
    
    #if AHT10_CONNECTED
        int aht_status = aht.begin(&Wire, 0, AHT20_ADDRESS);

        if (!aht_status) {
            Serial.println(F("Could not find a valid AHT20 sensor."));
            aht10_connected = false;
        }
    #endif

    #if LM75_CONNECTED
        lm75a.begin();

        if(lm75a.isShutdown())
            lm75a.wakeup();
    #endif

    int current_time =  timeClient.getEpochTime();
    data = {

        #if ADC_CONNECTED
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            settings.cumulative_total_energy1,
            settings.cumulative_total_energy2,
            settings.cumulative_total_energy3,
        #endif

        #if BMP280_CONNECTED
            0.0,
            0.0,
        #endif

        #if BMP180_CONNECTED
            0.0,
            0.0,
        #endif

        #if AHT10_CONNECTED
            0.0,
            0.0,
        #endif

        #if PULSE_CONNECTED
            0.0,
            settings.pulse_cumulative_total_energy,
        #endif

        #if LM75_CONNECTED
            0.0,
        #endif

        #if AM2320_CONNECTED
            0.0,
            0.0,
        #endif

        #if DHT_CONNECTED
            0.0,
            0.0,
        #endif

        0.0,
        0.0,
        "",
        "",
        current_time,
        current_time
    };

    #if SLEEP_MODE_ACTIVE
        bool result = rtcMemory.begin();
        if(result){

            rtc_data = rtcMemory.getData();
            if(result){
                debug_print("Previous Data Found");
                #if ADC_CONNECTED
                    data.cumulative_total_energy1 = rtc_data->cumulative_total_energy1;
                    data.cumulative_total_energy2 = rtc_data->cumulative_total_energy2;
                    data.cumulative_total_energy3 = rtc_data->cumulative_total_energy3;
                #endif

                #if PULSE_CONNECTED
                    data.pulse_cumulative_total_energy = rtc_data->pulse_cumulative_total_energy;
                #endif

                run_count = rtc_data->run_count;
                debug_print(String("Staring run from: ") + String(run_count));
            }
        }
        
    #endif

    data.battery_voltage = read_voltage(10);

    pubsubClient.setServer(settings.mqtt_broker, settings.mqtt_port);
    pubsubClient.setCallback(mqtt_receive);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", String(), false, processor);
    });

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request){
        int args = request->args();
        for(int i=0;i<args;i++){
            if(request->arg(i) == ""){
                continue;
            }
            
            if(request->argName(i) == "host"){
                strcpy(settings.hostname, request->arg(i).c_str());
            }

            if(request->argName(i) == "ssid"){
                strcpy(settings.ssid, request->arg(i).c_str());
                settings.valid_wifi = true;
            }

            if(request->argName(i) == "password"){
                strcpy(settings.password, request->arg(i).c_str());
                settings.valid_wifi = true;
            }

            if(request->argName(i) == "mqtt_host"){
                strcpy(settings.mqtt_broker, request->arg(i).c_str());
            }

            if(request->argName(i) == "mqtt_topic"){
                strcpy(settings.topic, request->arg(i).c_str());
            }

            if(request->argName(i) == "voltage_cal"){
                settings.voltage_cal = atof(request->arg(i).c_str());
            }

            if(request->argName(i) == "mqtt_port"){
                settings.mqtt_port = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "interval"){
                settings.collection_interval = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "discovery"){
                settings.discovery_interval = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "persist_cumulative_interval"){
                settings.persist_cumulative_interval = atoi(request->arg(i).c_str());
            }

            if(request->argName(i) == "wifi_timeout"){
                settings.timeout = atoi(request->arg(i).c_str());
            }
        }

        #if PULSE_CONNECTED
            settings.pulse_cumulative_total_energy = data.pulse_cumulative_total_energy;
        #endif

        #if ADC_CONNECTED
            settings.cumulative_total_energy1 = data.cumulative_total_energy1;
            settings.cumulative_total_energy2 = data.cumulative_total_energy2;
            settings.cumulative_total_energy3 = data.cumulative_total_energy3;
        #endif

        #if SLEEP_MODE_ACTIVE
            rtc_data->run_count = 0;
            rtcMemory.save();
        #endif

        EEPROM.put(0x0, settings);
        EEPROM.commit();

        request->send(LittleFS, "/index.html", String(), false, processor);
        ESP.restart();
    });

    server.on("/get_data", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/plain", api_data().c_str());
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/style.css", "text/css");
    });

    server.onNotFound(notFound);
    server.begin();
    
    startOTA();
    //sendMQTTDiscovery();
    delay(1000);
}

void loop() {
    if(!test_mode){
        if(run_count % settings.collection_interval == 0) 
        {
            getData();
            sendData();
        }

        if(run_count % settings.discovery_interval == 0)
        {
            //sendMQTTDiscovery();
        }

        if(run_count >= settings.persist_cumulative_interval)
        {
            #if ADC_CONNECTED
                settings.cumulative_total_energy1 = data.cumulative_total_energy1;
                settings.cumulative_total_energy2 = data.cumulative_total_energy2;
                settings.cumulative_total_energy3 = data.cumulative_total_energy3;
            #endif

            #if PULSE_CONNECTED
                settings.pulse_cumulative_total_energy = data.pulse_cumulative_total_energy;
            #endif

            #if ADC_CONNECTED || PULSE_CONNECTED
                EEPROM.put(0x0, settings);
                EEPROM.commit();
            #endif
            run_count = 0;
        }

        #if SLEEP_MODE_ACTIVE
            run_count += settings.collection_interval;

            #if ADC_CONNECTED
                rtc_data->cumulative_total_energy1 = data.cumulative_total_energy1;
                rtc_data->cumulative_total_energy2 = data.cumulative_total_energy2;
                rtc_data->cumulative_total_energy3 = data.cumulative_total_energy3;
            #endif

            #if PULSE_CONNECTED
                rtc_data->pulse_cumulative_total_energy = data.pulse_cumulative_total_energy;
            #endif

            rtc_data->run_count = run_count;
            rtcMemory.save();
            power_down(settings.collection_interval);
        #else
            run_count++;
        #endif
    }
    else{
        ArduinoOTA.handle();
        getData();

        if(checkWifi()){
            int rssi = WiFi.RSSI();

            if(rssi >= -50){
                led.setPixelColor(0, 0X00FF00);
            }
            if((rssi < -50) && (rssi > -100)){
                int colour = 255 * (-1 * (rssi + 50))/50;
                led.setPixelColor(0, ((colour << 8) + (255 - colour)) << 8);
            }
            if(rssi < -100){
                led.setPixelColor(0, 0XFF0000);
            }

            led.show();
            debug_print("RSSI: " + String(WiFi.RSSI()));
        }   
    }
    
    delay(1000);
}