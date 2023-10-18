#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "AiEsp32RotaryEncoder.h"



// GPIO Pins fÃ¼r den Bewegungsmelder und den Ultraschallsensor
#define PIN_MOTION_SENSOR 33
#define PIN_ULTRASOUND_TRIGGER 5
#define PIN_ULTRASOUND_ECHO 4

#define CALIB_PIN 13


// Timeout fÃ¼r den Ultraschallsensor
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

// Rotary Encoder
#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 22
#define ROTARY_ENCODER_BUTTON_PIN 21
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4


// MAC of Receiver Address
uint8_t receiverAddress[] = {0xEC, 0xFA, 0xBC, 0x7A, 0x7E, 0x88};
esp_now_peer_info_t peerInfo;

// Globale Variablen
int distance,duration;
RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR long SchwellenDistanz = 0;
RTC_DATA_ATTR bool motionDetected = false;
RTC_DATA_ATTR bool occupied = false;

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

// Data for ESP-Now connection
typedef struct messageToBeSent {
  bool statusWC;
  int distanzWC;
} messageToBeSent;

typedef struct receivedMessage {
  bool changeDistance;
  int setDistance;
} receivedMessage;

messageToBeSent myMessageToBeSent;
receivedMessage myReceivedMessage;


// Funktion zum Messen des Abstands mit dem Ultraschallsensor
int measureDistance() {
  int median = 0;
  digitalWrite(PIN_ULTRASOUND_TRIGGER, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(PIN_ULTRASOUND_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASOUND_TRIGGER, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(PIN_ULTRASOUND_ECHO, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;

  return distance;
}

// callback when data is sent
void messageSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback when data is received
void messageReceived(const uint8_t* macAddr, const uint8_t* incomingData, int len){
    memcpy(&myReceivedMessage, incomingData, sizeof(myReceivedMessage));
}

void IRAM_ATTR detectsMovement() {
  myMessageToBeSent.statusWC = true;
}

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}

// Change the set distance through the other ESP32
void changeDistance(){
  Serial.println("Pruefen ob neue Distanz eingestellt wird");
  Serial.println(myReceivedMessage.changeDistance);
  while(myReceivedMessage.changeDistance){
    SchwellenDistanz = myReceivedMessage.setDistance;
    Serial.println("Empfangene Distanz");
    Serial.println(myReceivedMessage.changeDistance);
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
    Serial.println(result == ESP_OK ? "Delivery Success" : "Delivery Fail");
    delay(500);
  }
}

void InitESPNow(){
  // WIFI Einstellungen
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(messageSent);  
  esp_now_register_recv_cb(messageReceived); 

  // Register peer
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}

void Ausgabe(){

  Serial.print("Distanz: ");
  Serial.println(distance);
  Serial.print("Normaldistanz: ");
  Serial.println(SchwellenDistanz);
  Serial.print("Counter ");
  Serial.println(counter);
  Serial.print("motionDetected ");
  Serial.println(motionDetected);

}

bool MotionDetection(){
  bool motion = false;
  if (digitalRead(PIN_MOTION_SENSOR) == HIGH) {
      motion = true;
      Serial.println("Motion detected");
      digitalWrite(BUILTIN_LED, HIGH);
      delay(500);
      digitalWrite(BUILTIN_LED,LOW); 
    }

  return motion;
}

void setup() {
  Serial.begin(115200);

  // Bewegungsmelder initialisieren
  pinMode(PIN_MOTION_SENSOR, INPUT);
  
  // Ultraschallsensor initialisieren
  pinMode(PIN_ULTRASOUND_TRIGGER, OUTPUT);
  pinMode(PIN_ULTRASOUND_ECHO, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(CALIB_PIN, INPUT_PULLDOWN);
  //Rotary Encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  bool circleValues = false;
  rotaryEncoder.setBoundaries(0, 1000, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setAcceleration(250);

  if (counter > 0){
    esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * TIME_TO_SLEEP);
  }else{
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  }
}

void loop() {
  // Wenn der Bewegungsmelder eine Bewegung detektiert hat
  myMessageToBeSent.statusWC = occupied;
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
  Serial.println(result == ESP_OK ? "Delivery Success" : "Delivery Fail");

  // Checks if changes to referenc distance habe been made
  changeDistance();

  

/*  if (digitalRead(PIN_MOTION_SENSOR) == LOW && counter) {
    esp_deep_sleep_start();
  }*/

  // Wenn sich jemand in der NÃ¤he befindet, den ESP32 wieder in den Tiefschlaf versetzen
  if (MotionDetection || motionDetected) {
    motionDetected = true;
    distance = measureDistance();
    delay(100);
    distance = measureDistance();
    
      // Ultraschallsensor noch einmal messen, um den Wert zu bestaetigen
    if (distance >= SchwellenDistanz && counter >= 3) {
      occupied = false;
      motionDetected = false;
      counter = 0;

      Serial.println("External Wakeup");
      Serial.println("myData.d = false");

      myMessageToBeSent.distanzWC = distance;
      myMessageToBeSent.statusWC = occupied;

      esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }

      // Niemand in der NÃ¤he, den ESP32 wieder in den Tiefschlaf versetzen
      esp_deep_sleep_start();

    }else{

      if(distance < SchwellenDistanz && counter < 3){
        counter = 0;
        Serial.println("myData.d = true");
        occupied = true;
        
        myMessageToBeSent.distanzWC = distance;
        myMessageToBeSent.statusWC = occupied;
        esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
        if (result == ESP_OK) {
          Serial.println("Sent with success");
        }
        else {
          Serial.println("Error sending the data");
        }
      }else{
        counter += 1;
      }
      delay(100);
      Serial.println("Timer Wakeup");
      esp_deep_sleep_start();
    }
  }else{
    //esp_deep_sleep_start();
    delay(1000);
  }

      
}


