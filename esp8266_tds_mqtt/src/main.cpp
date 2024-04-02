#include "Arduino.h"

// WATCHDOG
#include <Esp.h>

#define TIMER_INTERVAL_MS 500
// WIFI
#include <ESP8266WiFi.h>
// OTA
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// MQTT
#include <PubSubClient.h>

#define MAYOR 1
#define MINOR 0
#define PATCH 3
#define WIFI_SSID "JAVI"
#define WIFI_PASS "xavier1234"

// MQTT
#define MSG_BUFFER_SIZE (50)

volatile uint32_t lastMillis = 0;

String version = String(MAYOR) + "." + String(MINOR) + "." + String(PATCH);
bool state = 1;
// OTA
AsyncWebServer server(8080);

// MQTT(char*)
std::string device = "TDS";
const char *mqtt_server = "192.168.1.251";
const char *TOPIC = "Tanque1/canal/tds/sensor1";
std::string TOPIC_IP = "Tanque1/canal/ip/" + device;
const char *TOPIC_TEMP = "Tanque1/canal/temperature/sensor1";
WiFiClient espClient;
PubSubClient client(espClient);

// Update these with values suitable for your network.
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];

void callback_mqtt(char *topic, byte *payload, unsigned int length);
void reconnect_mqtt();

// APP
int getMedianNum(int bArray[], int iFilterLen);

#define TdsSensorPin A0
#define VREF 5.0          // analog reference voltage(Volt) of the ADC
#define SCOUNT 30         // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  Serial.begin(9600);
  Serial.printf("\n");
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // change it to your ussid and password
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
  }
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        uint32_t seconds = (uint32_t)(millis() / 1000);
                    char reply[100];
                    Serial.println(seconds);
                    sprintf(reply, "%ld %s  TDS mg/L temp: %0.2f", seconds,version.c_str(),temperature);

    request->send(200, "text/plain", reply); });
  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  // SETUP APP
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback_mqtt);

  Serial.println("Delta ms = " + String(millis() - lastMillis) + " " + version);
}
void loop()
{
  if (!client.connected())
  {
    reconnect_mqtt();
  }
  else
  {
    client.loop();

    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) // every 40 milliseconds,read the analog value from the ADC
    {
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
      analogBufferIndex++;
      if (analogBufferIndex == SCOUNT)
        analogBufferIndex = 0;
    }
    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
      printTimepoint = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                       // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                    // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                                 // temperature compensation
      tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value
      snprintf(msg, MSG_BUFFER_SIZE, "%.2f", tdsValue);
      client.publish(TOPIC, msg);
    }
    if (millis() - lastMillis > 2000)
    {
      lastMillis = millis();

      digitalWrite(LED_BUILTIN, state);
      state = !state;

      Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      snprintf(msg, MSG_BUFFER_SIZE, "%s:%s", device.c_str(), WiFi.localIP().toString().c_str());
      client.publish(TOPIC_IP.c_str(), msg);
    }
  }
}

void callback_mqtt(char *topic, byte *payload, unsigned int length)
{

  if (strcmp(topic, TOPIC_TEMP) == 0)
  {
    //temperature = atof((char *)payload);
    Serial.println("TDS GET TEMP:");
    std::string temp;
    for (size_t i = 0; i < 5; i++)
    {
      temp+=(char)payload[i];
    }
    
    Serial.printf("%s\n", temp.c_str());
    temperature=strtof(temp.c_str(),NULL);
  
  }
}

void reconnect_mqtt()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(TOPIC, "Teperature sensor");
      // ... and resubscribe
      client.subscribe(TOPIC_TEMP);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
