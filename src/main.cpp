#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// GPIO Pins fÃ¼r den Bewegungsmelder und den Ultraschallsensor
#define PIN_MOTION_SENSOR 33
#define PIN_ULTRASOUND_TRIGGER 5
#define PIN_ULTRASOUND_ECHO 4

// Timeout fÃ¼r den Ultraschallsensor
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

// MAC of Receiver Address
uint8_t receiverAddress[] = {0x94, 0xE6, 0x86, 0x2D, 0xA2, 0xB4};
esp_now_peer_info_t peerInfo;

// Globale Variablen
int distance,duration;
bool debugMode = true;
bool distanceChangeMode;

RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR long SchwellenDistanz = 0;
RTC_DATA_ATTR bool motionDetected = false;
RTC_DATA_ATTR bool occupied = false;

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
  //digitalWrite(PIN_ULTRASOUND_TRIGGER, LOW);
  //delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(PIN_ULTRASOUND_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASOUND_TRIGGER, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(PIN_ULTRASOUND_ECHO, HIGH);
  // Calculating the distance
  distance = duration * 0.017;

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
  Serial.print("Message Received\t");
  distanceChangeMode = myReceivedMessage.changeDistance;
  Serial.println(myReceivedMessage.changeDistance,distanceChangeMode);
  SchwellenDistanz = myReceivedMessage.setDistance;
}

void IRAM_ATTR detectsMovement() {
  myMessageToBeSent.statusWC = true;
}

// Change the set distance through the other ESP32
void changeDistance(){
  if(debugMode){Serial.print("Pruefen ob neue Distanz eingestellt wird\t");  Serial.println(distanceChangeMode);}

  while(distanceChangeMode){
    if(debugMode){ Serial.print("Empfangene Distanz\t"); Serial.println(myReceivedMessage.setDistance);}

    // Send again to receive newest data from ESP32
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
    Serial.println(result == ESP_OK ? "Delivery Success" : "Delivery Fail");
    delay(500);
  }
}

void InitESPNow(){
  // WIFI Einstellungen
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK){
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
  Serial.print("Distanz [cm]: ");
  Serial.println(distance);
  Serial.print("Normaldistanz: ");
  Serial.println(SchwellenDistanz);
  Serial.print("Counter ");
  Serial.println(counter);
  Serial.print("motionDetected ");
  Serial.println(motionDetected);
}

void PrepForMotiondetectionSleep(){
  occupied = false;
  motionDetected = false;
  counter = 0;
  myMessageToBeSent.distanzWC = distance;
  myMessageToBeSent.statusWC = occupied;

  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
  delay(100);
  Serial.println("Sending before MotionSleep:");
  Serial.println(result == ESP_OK ? "Sent with success" : "Error sending the data");
}

void resetCounterAndResendData(){
  counter = 1;
  occupied = true;
  myMessageToBeSent.distanzWC = distance;
  myMessageToBeSent.statusWC = occupied;

  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
  delay(100);
  Serial.println("Sending before TimerSleep:");
  Serial.println(result == ESP_OK ? "Sent with success" : "Error sending the data");
}

bool MotionDetection(){
  bool motion = false;
  if (digitalRead(PIN_MOTION_SENSOR) == HIGH) {
      digitalWrite(LED_BUILTIN,HIGH);
      motion = true;
      Serial.println("Motion detected");
      digitalWrite(BUILTIN_LED, HIGH);
      delay(500);
      digitalWrite(BUILTIN_LED,LOW); 
    }
    else{
    digitalWrite(LED_BUILTIN,LOW);}

  return motion;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);

  // Bewegungsmelder initialisieren
  pinMode(PIN_MOTION_SENSOR, INPUT);
  
  // Ultraschallsensor initialisieren
  pinMode(PIN_ULTRASOUND_TRIGGER, OUTPUT);
  pinMode(PIN_ULTRASOUND_ECHO, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);

  InitESPNow();

  // Sleep timer definition
  if (counter > 0){
    Serial.println("Timer Wakeup");
    esp_sleep_enable_timer_wakeup(uS_TO_S_FACTOR * TIME_TO_SLEEP);
  }else{
    Serial.println("External Wakeup");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  }

}

void loop() {
  // Wenn der Bewegungsmelder eine Bewegung detektiert hat


  /*distance = measureDistance();
  Serial.println(distance);
  delay(300);
  */

  //if(occupied){myMessageToBeSent.statusWC = false; occupied = false;}else{myMessageToBeSent.statusWC = true; occupied = true;} 

  myMessageToBeSent.statusWC = occupied;
  myMessageToBeSent.distanzWC = distance;
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
  Serial.println(result == ESP_OK ? "Delivery Success" : "Delivery Fail");
  delay(500);

  changeDistance();
  delay(50);

  // Wenn sich jemand in der NÃ¤he befindet, den ESP32 wieder in den Tiefschlaf versetzen
  if (MotionDetection || motionDetected) {
    motionDetected = true;
    distance = measureDistance();
    delay(300);
    distance = measureDistance();
    if(debugMode){Ausgabe();}
    delay(100);
    // Wurde Schwellenwert ueberschritten (Sitz Leer) und dreimal nachgemesssen (counter)
    if (distance >= SchwellenDistanz && counter > 3) {

      PrepForMotiondetectionSleep();
      esp_deep_sleep_start();
    }else{

      // Wir innerhalb von drei Messungen Sitz wieder besetzt wird counter zurueckgesetzt
      if((distance < SchwellenDistanz || distance > 900) && counter < 4){
        resetCounterAndResendData();

      }else{
        counter += 1;
      }

      delay(100);
      esp_deep_sleep_start();
    }
  }else{
    //esp_deep_sleep_start();
    delay(1000);
  }  
  
}