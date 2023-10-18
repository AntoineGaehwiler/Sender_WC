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


// Globale Variablen
uint8_t broadcastAddress[] = {0xEC, 0xFA, 0xBC, 0x7A, 0x7E, 0x88};
int distance,duration;
RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR long NORMALDISTANZ = 0;
RTC_DATA_ATTR bool motionDetected = false;
RTC_DATA_ATTR bool occupied = false;

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

esp_now_peer_info_t peerInfo;
// Variablen fÃ¼r WLAN
typedef struct message {
  bool status_WC;
  int distanz_WC;
} message;
struct message myMessage;

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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void IRAM_ATTR detectsMovement() {
  myMessage.status_WC = true;
}

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}

void rotary_loop()
{
  Serial.println("Rotary Abfrage");

  while(digitalRead(CALIB_PIN)){

    if (rotaryEncoder.encoderChanged())
      {
        Serial.print("Value: ");
        Serial.println(rotaryEncoder.readEncoder());
        NORMALDISTANZ = rotaryEncoder.readEncoder();
        delay(100);
      }
    delay(100);
  }
}

void Ausgabe(){

  Serial.print("Distanz: ");
  Serial.println(distance);
  Serial.print("Normaldistanz: ");
  Serial.println(NORMALDISTANZ);
  Serial.print("Counter ");
  Serial.println(counter);
  Serial.print("motionDetected ");
  Serial.println(motionDetected);

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

  // WIFI Einstellungen
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

    // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  if (counter > 0){
    esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * TIME_TO_SLEEP);
  }else{
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  }
}

void loop() {
  // In den Tiefschlaf versetzen
  rotary_loop();

  // Wenn der Bewegungsmelder eine Bewegung detektiert hat
  myMessage.status_WC = occupied;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myMessage, sizeof(myMessage));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }


  if (digitalRead(PIN_MOTION_SENSOR) == HIGH) {
  
    motionDetected = true;
    digitalWrite(BUILTIN_LED, HIGH);
    delay(500);
    digitalWrite(BUILTIN_LED,LOW); 
  }


/*  if (digitalRead(PIN_MOTION_SENSOR) == LOW && counter) {
    esp_deep_sleep_start();
  }*/
  
  Serial.print("motionDetected in general ");
  Serial.println(motionDetected);

  // Wenn sich jemand in der NÃ¤he befindet, den ESP32 wieder in den Tiefschlaf versetzen
  if (motionDetected) {
    distance = measureDistance();
    delay(100);
    distance = measureDistance();


    
      // Ultraschallsensor noch einmal messen, um den Wert zu bestÃ¤tigen
    if (distance >= NORMALDISTANZ && counter >= 3) {
      occupied = false;
      motionDetected = false;
      counter = 0;
      Serial.println("External Wakeup");
      Serial.println("myData.d = false");

      myMessage.distanz_WC = distance;
      myMessage.status_WC = occupied;

      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myMessage, sizeof(myMessage));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }

      // Niemand in der NÃ¤he, den ESP32 wieder in den Tiefschlaf versetzen
      esp_deep_sleep_start();

    }else{
      if(distance < NORMALDISTANZ && counter < 3){
        counter = 0;
        Serial.println("myData.d = true");
        occupied = true;
        
        myMessage.distanz_WC = distance;
        myMessage.status_WC = occupied;
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myMessage, sizeof(myMessage));
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


