#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdbool.h>

#include "WiFi.h"
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_22
#define THINGNAME "vehicle_health"

#define I2C_SDA 18
#define I2C_SCL 19

const char WIFI_SSID[] = "Ajay";
const char WIFI_PASSWORD[] = "123456789";

const char AWS_IOT_ENDPOINT[] = "my_aws_iot_point";

uint8_t temperature;
// uint16_t average;
uint8_t Touch_Result;
uint16_t distance;
uint8_t x, y, z;
uint8_t flag;
uint8_t steering;
uint16_t vibration;

bool can_alive = false;
unsigned long lastCanRxTime = 0;
const unsigned long CAN_TIMEOUT = 3000;

LiquidCrystal_I2C lcd(0x27, 16, 2);

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();   // must match STM32
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // receive all IDs

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----


)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----

-----END RSA PRIVATE KEY-----


)KEY";
#define AWS_IOT_PUBLISH_TOPIC "vehicle_health_pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "vehicle_health_sub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void publishMessage();
// void messageHandler();
void connectAWS();
void setup()
{
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    lcd.init(); // initialize the lcd
    lcd.init();
    // Print a message to the LCD.
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.setCursor(0, 1);
    connectAWS();
    lcd.print("AWS CONNECTED");
    lcd.clear();

    // Install CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        Serial.println("Failed to install TWAI driver");
        while (1)
            ;
    }
    // Start CAN driver
    if (twai_start() != ESP_OK)
    {
        Serial.println("Failed to start TWAI driver");
        while (1)
            ;
    }
    Serial.println("ESP32 CAN Receiver Ready");
}

void loop()
{
    twai_message_t rx_frame;
    if (twai_receive(&rx_frame, pdMS_TO_TICKS(100)) == ESP_OK && rx_frame.identifier == 0x123)
    {

        can_alive = true;
        lastCanRxTime = millis();
        temperature = rx_frame.data[0];
        distance = rx_frame.data[2];
        x = rx_frame.data[3];
        y = rx_frame.data[4];
        z = rx_frame.data[5];
        vibration = abs(sqrt(x * x + y * y + z * z) - 350);

        Touch_Result = rx_frame.data[6];

        Serial.print("temp: ");
        Serial.println(temperature);
        Serial.print("distance: ");
        Serial.println(distance);

        Serial.print("vibration: ");
        Serial.println(vibration);
        Serial.print("Touch_Result: ");
        Serial.println(Touch_Result);
        Serial.println("data recieved: ");

        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(temperature);
        lcd.print("  D:");
        lcd.print(distance);
        lcd.print("   "); // clear leftovers

        lcd.setCursor(0, 1);
        lcd.print("V:");
        lcd.print(vibration);
        lcd.print("  Touch:");
        lcd.print(Touch_Result);
        lcd.print(" ");

        if (!client.connected())
        {
            connectAWS();
        }
        client.loop();
        static unsigned long lastMillis = 0;
        if (millis() - lastMillis > 5000)
        {
            lastMillis = millis();
            publishMessage();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Sending to AWS");
        }
    }

    if (millis() - lastCanRxTime > CAN_TIMEOUT)
    {
        can_alive = false;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("CAN DATA LOST");
        lcd.setCursor(0, 1);
        lcd.print("Check STM32");

        Serial.println("CAN DATA LOST");
    }
    delay(1000);
}

void publishMessage()
{
    StaticJsonDocument<200> doc;

    doc["temperature"] = temperature;
    doc["oil"] = distance;
    doc["vibration"] = vibration;

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void connectAWS()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        lcd.setCursor(0, 0);
        lcd.print("Connecting WiFi");
        delay(100);
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi connected!");
    // Configure WiFiClientSecure with AWS IoT certificates
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    client.setServer(AWS_IOT_ENDPOINT, 8883);
    // client.setCallback(messageHandler);

    Serial.println("Connecting to AWS IOT");
    while (!client.connect(THINGNAME))
    {
        Serial.print(".");
        delay(100);
    }

    lcd.setCursor(0, 0);
    lcd.print("Connected to AWS");

    if (!client.connected())
    {
        Serial.println("AWS IoT Timeout!");
        lcd.setCursor(0, 0);
        lcd.print("TIMEOUT AWS");
        return;
    }

    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.println("AWS IoT Connected!");
}

void messageHandler(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}
