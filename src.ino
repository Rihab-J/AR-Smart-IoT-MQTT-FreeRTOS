
//------------------------------------------------- A&R Smart ----------------------------------------------//
// Released by : Aroua SAIDAOUI & Rihab JOUINI

//-------------------------------------------------------------------------------------------------------------//
//----------------------------------------------- Includes ----------------------------------------------------//
#include <STM32FreeRTOS.h>
#include <SPI.h>
#include <WiFiST.h>
#include <PubSubClient.h>
#include <stdio.h> 
#include <HTS221Sensor.h>
//----------------------------------------- Netword & MQTT variables -------------------------------------------//
char ssid[] = "rihab";
const char* password = "*****";
const char* mqtt_server = "tailor.cloudmqtt.com"; 
const int  mqttPort = 11289 ;
const char* mqttUser = "*****";
const char* mqttPassword ="****";
//------------------------------------------ All Program Variables ----------------------------------------------//
long lastMsg = 0;
long lastMsg1 = 0;
long lastMsg2 = 0;
char msg[50];
long value = 0;
char buffer_T[25];
char buffer_H[25];
char buffer_HS[25];
char buffer_G[25];
float pressure;
float humidity;
float temperature;
float GazValue;
float soil_moisture; 
float soil_moisture_perc; 
//--------------------------------------- WiFi Protocol Definition ----------------------------------------------//
SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);
WiFiClient STClient;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
PubSubClient client(STClient);
//------------------------------------------I2C Sensors Protocol ----------------------------------------------//
#define I2C2_SCL    PB10
#define I2C2_SDA    PB11

// Components.
TwoWire *dev_i2c;
#if defined(ARDUINO_SAM_DUE)
#define DEV_I2C Wire1   //Define which I2C bus is used. Wire1 for the Arduino Due
#define SerialPort Serial
#else
#define DEV_I2C Wire    //Or Wire
#define SerialPort Serial
#endif
#define SerialPort Serial
#define I2C2_SCL    PB10
#define I2C2_SDA    PB11

HTS221Sensor  *Humidity_Temper;
SemaphoreHandle_t gas_sem;
SemaphoreHandle_t MS_sem;
SemaphoreHandle_t dht_sem;




void TaskTemperature( void *pvParameters );
void TaskSoilMoisture( void *pvParameters );
void TaskGaz( void *pvParameters );
//------------------------------------------------- WiFi Config ------------------------------------------------//
void setup_wifi() {
  delay(10);
  if (WiFi.status() == WL_NO_SHIELD) 
  {
    Serial.println("WiFi module not detected");
    while (true);
  }
  String fv = WiFi.firmwareVersion(); // print firmware version:
  Serial.print("Firmware version: ");
  Serial.println(fv);

  if (fv != "C3.5.2.3.BETA9") 
  {
    Serial.println("Please upgrade the firmware");
  }
  Serial.println("Start network connection using : "); // Connect to WPA (TKIP) network:
  Serial.print("SSID : ");
  Serial.println(ssid);
  Serial.print("Password : "); 
  Serial.println(password);
  while (status != WL_CONNECTED)
  {
    Serial.println("..Loading..");
    status = WiFi.begin(ssid, password); // Connect to WPA2 network:
    if (status != WL_CONNECTED) 
    {
      status = WiFi.begin(ssid, password, ES_WIFI_SEC_WPA); // Connect to WPA(WiFi Protected Access,TKIP) network:
    }
    delay(1000);
    }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
//---------------------------------------------- MQTT config --------------------------------------------------//
void setup_mqtt(){
  client.setServer(mqtt_server, 11289);
  if(client.connect("STClient", mqttUser, mqttPassword ))
  {Serial.println("Successfully connected to MQTT");
  }
  else
  {Serial.println("Cannot connect to MQTT");
    }
  client.setCallback(callback);
  }
void setup() {
  Serial.begin(9600);  
  setup_wifi(); 
  setup_mqtt();
  dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL); // Initialize I2C bus (Communication between the sensors and CPU)
  dev_i2c->begin();
  
  Humidity_Temper = new HTS221Sensor (dev_i2c); // Initlialize temperature & humidity sensor 
  Humidity_Temper->Enable();
  while (!Serial) {
  }
   if ( gas_sem == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    gas_sem = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( gas_sem ) != NULL )
      xSemaphoreGive( ( gas_sem ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
   if ( dht_sem == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    dht_sem = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( dht_sem ) != NULL )
      xSemaphoreGive( ( dht_sem ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  if ( MS_sem == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    MS_sem = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( MS_sem ) != NULL )
      xSemaphoreGive( ( MS_sem ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  xTaskCreate(
    TaskTemperature
    ,  (const portCHAR *)"Tempreture"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

    
    xTaskCreate(
    TaskSoilMoisture
    ,  (const portCHAR *)"Task_Soil_Moisture"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    
     xTaskCreate(
    TaskGaz
    ,  (const portCHAR *)"DigitalRead"  // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2 //
    ,  NULL );
  vTaskStartScheduler();  // start scheduler
  Serial.println("Insufficient RAM");
  while(1);
}

//----------------------------------------------- Tasks Definition ----------------------------------------------//


void TaskTemperature(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit
  {
    if ( xSemaphoreTake( dht_sem, ( TickType_t ) 5 ) == pdTRUE )
    {
  Humidity_Temper->GetHumidity(&humidity);
  Humidity_Temper->GetTemperature(&temperature);
  Serial.print("Temperature [°C] = ");
  Serial.println(temperature);
  Serial.print("Humidity [%] = ");
  Serial.println(humidity);
  
  }
  xSemaphoreGive( dht_sem );
   long now1 = millis();
    if (now1 - lastMsg1 > 2000) {
    lastMsg1 = now1;
    ++value;
    snprintf (msg, 50, "Send Temperature and Humidity #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    dtostrf(temperature,5, 2, buffer_T);
    dtostrf(humidity,5, 2, buffer_H);
    client.publish("temperature",buffer_T);
    client.publish("humidity",buffer_H);

  }
  vTaskDelay( 100 );
  }
}



void TaskSoilMoisture(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
 
  for (;;) // A Task shall never return or exit.
  {
    if ( xSemaphoreTake( MS_sem, ( TickType_t ) 5 ) == pdTRUE )
    { 
     soil_moisture = analogRead(A1);
     soil_moisture_perc = map(soil_moisture,0,1023,0,100);
    
     Serial.print("soil moisture [%] = ");
     Serial.println(soil_moisture_perc);
      
  }
  xSemaphoreGive( MS_sem );
  long now1 = millis();
    if (now1 - lastMsg1 > 2000) {
    lastMsg1 = now1;
    ++value;
    snprintf (msg, 50, "Send soil moisture #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    dtostrf(soil_moisture_perc,5, 2,buffer_HS);
    client.publish("soil_moisture",buffer_HS);
 
  }
  vTaskDelay( 100 );
  
  }
}
void TaskGaz( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    if ( xSemaphoreTake( gas_sem, ( TickType_t ) 5 ) == pdTRUE )
    {
    // read the input on analog pin 0:
     float sensorGaz = analogRead(A0);
    GazValue=map(sensorGaz,150,1023,0,100);
    Serial.print("Gaz = ");
    Serial.println(GazValue);
    
  }
  xSemaphoreGive( gas_sem );
   long now1 = millis();
    if (now1 - lastMsg1 > 2000) {
    lastMsg1 = now1;
    ++value;
    snprintf (msg, 50, "Send Gaz #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    dtostrf(GazValue,5, 2, buffer_G);
    client.publish("Gaz",buffer_G);

  }
  vTaskDelay( 100 );
}
}

void loop(){}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("STClient", mqttUser, mqttPassword )) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("3AGE2", "{\"temperature\":10,\"humidity\":11,\"position\":12}");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
