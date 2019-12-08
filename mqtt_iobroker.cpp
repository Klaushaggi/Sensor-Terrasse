// BME280-Werte per MQTT zum iobroker
// und deep_sleep

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1018.00)
Adafruit_BME280 bme; // I2C, also Pin D1 und D2 des NodeMCU

float temperature = 0;
float humidity = 0;
float rssi = 0;
float druck = 0;
double spg  = 0;

const char* ssid = "****";
const char* password = "****";
const char* mqtt_server = "****";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msgtemp[50];
char msgluft[50];
char msgrssi[50];
char msgspg[50];
char msgdruck[50];
int value = 0;

void setup_wifi() 
{
  int versuch=1;
  delay(10);
  // We start by connecting to a WiFi network
 // Serial.println();
 // Serial.print("Try to connecting to ");
 // Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    if(WiFi.status() != WL_CONNECTED)
    { 
      //delay(250); 
      //digitalWrite(D4, LOW); 
      //delay(250);
      //digitalWrite(D4, HIGH);
        delay(500);
      //Serial.print(".");
      if(++versuch==30) 
      {
        //Serial.println("Keine WiFi-Verbindung möglich");
        return;
      }
    }
  }

}

void reconnect()
{
  int versuch=1;

  // Loop until we're reconnected
  while (!client.connected()) {
    //Serial.print("Versuch Nr. ");Serial.printf("%i: ",versuch);
    //Serial.println("Attempting MQTT connection...");  
    // Attempt to connect
    if (client.connect("TerrasseESP")) {
      //Serial.println("connected");
    } else {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 2 seconds"); 
      // Wait 2 seconds before retrying
      delay(2000);

      if(++versuch==4)
         return;
    }
  }
}

void setup()
{
  //pinMode(D4, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
 // digitalWrite(D4, HIGH);  // LED ist aus
  //Serial.begin(115200);
                               // Division mit float beachten: 242.0 statt 242
  spg = analogRead (A0)/242.0; // Analog Values 0 to 1024
  //Serial.println (spg,2);      // 2 Nachkommenstellen 
  setup_wifi();
  bme.begin();
  if(WiFi.status() == WL_CONNECTED) // nur falls Wifi verbunden
  {
    //Serial.println("");
    //Serial.println("WiFi connected");
    //Serial.println("IP address: ");
    //Serial.println(WiFi.localIP());
      //Received Signal Strength Indicator
    rssi=WiFi.RSSI();
    //Serial.print("RSSI= ");Serial.print(rssi);
     //  Serial.println("dBm"); 
    client.setServer(mqtt_server, 1883);

    //digitalWrite(D4, LOW);  // blaue LED
    if (!client.connected()) 
    {
      reconnect();
    }

    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    druck = bme.readPressure() / 100.0F;
  
    
   
   //Serial.print((float)temperature,1); Serial.print(" *C, ");  // serial.print(wert,Nachkommastellen)
   //Serial.print((float)humidity,1); Serial.println(" RH%");
   
    if(client.connected())
    { 
    dtostrf(temperature, 0, 1, msgtemp);
    dtostrf(humidity, 0, 1, msgluft);
    dtostrf(rssi, 0, 1, msgrssi);
    dtostrf(spg, 0, 2, msgspg);
    dtostrf(druck+6.00, 0, 1, msgdruck); // Korrektur um 6 mbar

    
    //Serial.println("Publish message: ");
    //Serial.print("Temp.: "); Serial.println(msgtemp);    
    //Serial.print("Luft: ");Serial.println(msgluft);
    //Serial.print("RSSI: ");Serial.println(msgrssi);
    //
    //Serial.print("Spg: ");Serial.println(msgspg);
    client.publish("Sensoren/Terrasse/RSSI", msgrssi); 
    delay(50); 
    client.publish("Sensoren/Terrasse/Luft", msgluft);
    delay(50);
    client.publish("Sensoren/Terrasse/Spg", msgspg);
    delay(50);
    client.publish("Sensoren/Terrasse/Druck", msgdruck);
    delay(50);
    client.publish("Sensoren/Terrasse/Temp", msgtemp);
    delay(50);
    
    } /*
    else
    {
      Serial.println("Nichts gesendet, keine Verbindung zum Broker"); 
    } */
    
  }
    ESP.deepSleep(3600e6); //20.000.000us oder 20s
    delay(100);
}


void loop() {  
}
