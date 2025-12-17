//%Added Log funktion
///////////////*versions Nr*/////////////////
#define __MYVRS__ "v15"
/////////////////////////////////////////////
#include "WiFi.h"
#include "DHT.h"
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
#include "esp_now.h"
#include "esp_wifi.h"

/////////////////////////////////////////////
uint8_t peerMac[] = { 0xD4, 0xD4, 0xDA, 0x5A, 0xA7, 0x3C };

typedef struct MessegeData {
  float afstand;   //distance variable
  float Safstand;  //distance variable
  float dt;
  float Height;
  unsigned long sendStart;
  float humidity;
  float temperature;
  int failsafeAvg;
  unsigned int packets;
} MessegeData;

bool sensorState = false;

MessegeData myData;

esp_now_peer_info_t peerInfo;
unsigned long lastSend = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


//Send data
unsigned long lastrecivet = 0;
unsigned long recivet = 0;
unsigned long timestamp = 0;


void ESPNOW() {
  if (sensorState == true) {
    myData.packets++;
    lastSend = micros();
    esp_err_t result = esp_now_send(peerMac, (uint8_t *)&myData, sizeof(myData));
    lastSend = micros() - lastSend;

    // Serial.println("__________________________________________________________________________________________________________________");
    // Serial.print("sendt time: ");
    // Serial.print("\t");
    // Serial.print(lastSend);
    // Serial.println(" ms");
    Serial.println("_______________");
    Serial.print("Måling nr: \t");
    Serial.println(myData.packets);

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
    Serial.println("_______________");
    Serial.println();


    if (recivet == 0) {
    } else {
      lastrecivet = millis() - recivet;
      timestamp += lastrecivet;
    }

    recivet = millis();


    // Serial.print("Sidst sendt for:");
    // Serial.print("\t");
    // Serial.print(lastrecivet);
    // Serial.println(" ms");

    // Serial.print("timestamp:");
    // Serial.print("\t");
    // Serial.print(timestamp);
    // Serial.println(" ms");

    // Serial.print("recivet:");
    // Serial.print("\t");
    // Serial.print(recivet);
    // Serial.println(" ms");


    size_t myDatasize = sizeof(myData);
    // Serial.println("_______________");
    // Serial.print("Data size: ");
    // Serial.print(myDatasize);
    // Serial.println(" bytes");
    // Serial.println("__________________________________________________________________________________________________________________");
    // Serial.println();
    // Serial.println();
    // Serial.println();
    sensorState = false;
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int trigPin = 44;  //Trig
const int echoPin = 43;  //Echo

//udregning af afstand med lyd
//d = c*delta(t)*100/2
//d afstanden
//c er lydens hastighed
//Delta(t) er den tiden det tager for lyden, at komme frem og tilbage
//1 sekund = 1.000.000 mikrosekunder - Dividere med 1.000.000
//1m = 100cm - vi gange med 100 for at få det i cm
//const int Sound = 340;  //Lyd 340m/s
float sound;
void Startultralyd() {


  digitalWrite(trigPin, LOW);  // sikre at der ikke sendes sinaler
  delayMicroseconds(5);        //giver optimale mål

  digitalWrite(trigPin, HIGH);  // transmit
  delayMicroseconds(10);        //det nødvendige tid, ifølge datasheet
  digitalWrite(trigPin, LOW);   // stop transmission

  myData.dt = pulseIn(echoPin, HIGH, 30000);  // listen for pulses

  sound = 331.4 + (myData.temperature * 0.6);

  //d = c*delta(t)*100/2*1.000.000
  myData.Safstand = (myData.dt * sound * 100) / (2 * 1000000);  // calculate distance


  //   Serial.print("Safstand: ");
  //   Serial.print(myData.Safstand);
  //   Serial.println(" cm");
  //   Serial.println();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ultralyd() {


  digitalWrite(trigPin, LOW);  // sikre at der ikke sendes sinaler
  delayMicroseconds(5);        //giver optimale mål

  digitalWrite(trigPin, HIGH);  // transmit
  delayMicroseconds(10);        //det nødvendige tid, ifølge datasheet
  digitalWrite(trigPin, LOW);   // stop transmission

  myData.dt = pulseIn(echoPin, HIGH, 30000);  // listen for pulses

  sound = 331.4 + (myData.temperature * 0.6);

  //d = c*delta(t)*100/2*1.000.000
  myData.afstand = (myData.dt * sound * 100) / (2 * 1000000);  // calculate distance


  // myData.Height = myData.Safstand - myData.afstand;

  Serial.println("_______________");
  Serial.print("Afstand: ");
  Serial.print(myData.afstand);
  Serial.println(" cm");
  // Serial.print("\t");
  // Serial.print("Temperatur",myData.temperature);
  // Serial.println(" C");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int readPin = 17, vccPin = 21;
const int maxLimit = 300;


int readingSum, readsPrLoop = 6;
int arr[6];


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int AlarmPin = 18;  //Hvis der ikke læses en afstand
const int relayPin = 16;





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void failsafe() {
  myData.failsafeAvg = 0;
  digitalWrite(vccPin, HIGH);

  Serial.println("_______________");
  for (int readCount = 0; readCount < readsPrLoop; readCount++) {
    int sensorValue = analogRead(readPin);
    //Serial.println("Måling: ");
    arr[readCount] = sensorValue;
    //Serial.println(arr[readCount]);

    readingSum += arr[readCount];
  }

  myData.failsafeAvg = readingSum / readsPrLoop;

  if (myData.failsafeAvg > maxLimit) {
    if (myData.failsafeAvg > 800) {
      Serial.println();
      Serial.println();
      Serial.println("Bug Vandsensor");
      Serial.println();
      Serial.println();
    }
    Serial.println("Vandstand for høj!");
  }

  Serial.println("Failsafe gennemsnit: ");
  Serial.println(myData.failsafeAvg);
  readingSum = 0;
  digitalWrite(vccPin, LOW);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DHTPIN 13      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11  // DHT 11

DHT dht(DHTPIN, DHTTYPE);

void DHLsensor() {
  // Wait a few seconds between measurements.

  myData.humidity = dht.readHumidity();

  myData.temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(myData.humidity) || isnan(myData.temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.println("_______________");
  Serial.print("Humidity: ");
  Serial.print(myData.humidity);
  Serial.print("% ");
  Serial.print("|  Temperature: ");
  Serial.print(myData.temperature);
  Serial.println("°C ");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int knap = 12;  //til knappen, der kan tænde og slukke for alarm

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////
void printHeader() {
  Serial.println();
  Serial.println(__FILE__);
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  Serial.println(__MYVRS__);
  Serial.println("____________");
}

void Start() {
  Serial.begin(115200);
  delay(50);
  printHeader();
  delay(50);
}
/////////////////////////////////////////////
void macAddress() {
  // Serial.print("macAddress: ");
  // Serial.println(WiFi.macAddress());
  // Serial.println();
  delay(100);
}
unsigned long timeToRead_ultralyd, timeToRead_DHL, timeToRead_failsafe;

//interval mellem måling
unsigned long delay_ultralyd = 1000, delay_DHL = 5000, delay_failsafe = 10000;



void sensorstyring(String x, unsigned long &timeToRead, unsigned long &delayToNext, void (*sensor)()) {
  if (timeToRead < millis()) {
    timeToRead = millis() + delayToNext;
    sensor();
    // Serial.print(x);
    // Serial.print("\t");
    // Serial.print(millis());
    // Serial.print("\t");
    // Serial.print("næste måling: ");
    // Serial.println(delayToNext);
    // Serial.println("_______________");
    sensorState = true;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int maxafstand = 80;
int procentvand;
int state;
unsigned long knaptrykalarm = 0;
unsigned long alarmtid = 0;
unsigned long alarmstart = 0;
bool lock = false;
bool alarmred = false;
bool alarmyellow = false;
bool alarmfailsafe = false;

bool relay = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // 50 ms
int lastButtonState = HIGH;              // Aktiv-LOW knap
int buttonState = HIGH;

unsigned long knaptid;

int reading;

void kanp() {

  reading = digitalRead(knap);

  // Debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    buttonState = reading;
  }

  lastButtonState = reading;


  if ((alarmred == true || alarmfailsafe==true) && relay == true) {
    if (buttonState == LOW && knaptrykalarm == 0) {
      knaptrykalarm = millis();
      Serial.println("Starter 3 sek knap");
    }


    if (buttonState == LOW) {
      alarmtid = millis();
      if (alarmtid - knaptrykalarm >= 3000) {
        if (lock == false) {
          digitalWrite(relayPin, LOW);
          Serial.println("Knap trykket efter 3 sek - slukker pumpen");

          lock = true;
          relay = false;
          log('b');

          knaptrykalarm = 0;
          knaptid = millis() + 3000;
        }
      }
    } else if (buttonState == HIGH && knaptrykalarm != 0) {
      knaptrykalarm = 0;
      Serial.println("DBG: TIMER RESET AF HIGH");
    }

  } else if (lock == false) {
    if (buttonState == LOW && (millis() >= knaptid)) {
      bool reverseknap = !digitalRead(relayPin);
      digitalWrite(relayPin, reverseknap);
      Serial.println("Knap trykket!");
      knaptid = millis() + 500;
      lock = true;
      relay = !relay;
      log('b');
    }
  }

  if (buttonState == HIGH && lock == true) {
    lock = false;
    knaptrykalarm = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void logData() {
  Serial.print("Vandstand: ");
  Serial.print(procentvand);
  Serial.print("%");
  Serial.print('\t');
  Serial.print("Hum: ");
  Serial.print(myData.humidity);
  Serial.print("%");
  Serial.print('\t');
  Serial.print("Temp: ");
  Serial.print(myData.temperature);
  Serial.println("C");
}

void logButton() {
  Serial.println("Button Override");
  Serial.print("relayState: ");
  Serial.print('\t');
  if (relay == true) {
    Serial.print("ON");
  } else {
    Serial.print("OFF");
  }
  Serial.println();
}

void logAlarm() {
  Serial.print("alarmRed: ");
  if (alarmred == true) {
    Serial.print("ON");
  } else {
    Serial.print("OFF");
  }
  Serial.print('\t');
  Serial.print("alarmYellow: ");
  if (alarmyellow == true) {
    Serial.print("ON");
  } else {
    Serial.print("OFF");
  }
  Serial.print('\t');
  Serial.print("alarmFailsafe: ");
  if (alarmfailsafe == true) {
    Serial.print("ON");
  } else {
    Serial.print("OFF");
  }
  Serial.println();
}

void log(char x) {
  Serial.println("__________");
  Serial.print("LOG");
  Serial.print('\t');
  Serial.print("time: ");
  Serial.println(millis());
  switch (x) {
    case 'r':
      Serial.println("Red Alarm Activated");
      logAlarm();
      break;
    case 'y':
      Serial.println("Yellow Alarm Activated");
      logAlarm();
      break;
    case 'f':
      Serial.println("Failsafe Alarm Activated");
      logAlarm();
      break;
    case 'b':
      logButton();
      logAlarm();
      break;
    case 'n':
      Serial.println("Alarm Off");
      logAlarm();
      break;
    default:
      break;
  }
  logData();
  Serial.println("__________");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Start();  //Laver start kriterier
  WiFi.mode(WIFI_STA);
  macAddress();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);  // Sæt kanal 1
  // esp_now_start();

  // ACTIVATE LONG RANGE PROTOCOL:
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  //WiFi.begin();
  Serial.println();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW-connections initialiseret.");
    Serial.println();
  } else {
    Serial.println("ESP-NOW-connections er ikke initialiseret.");
    Serial.println();

    ESP.restart();
  }
  esp_wifi_set_max_tx_power(80);

  esp_now_register_send_cb(OnDataSent);

  //Register peer

  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(trigPin, OUTPUT);  //transmit is ouput
  pinMode(echoPin, INPUT);   //receive is input

  pinMode(readPin, INPUT);
  pinMode(vccPin, OUTPUT);
  digitalWrite(vccPin, LOW);

  pinMode(AlarmPin, OUTPUT);
  digitalWrite(AlarmPin, LOW);
  pinMode(relayPin, OUTPUT);

  Startultralyd();

  delay(1000);

  dht.begin();
  pinMode(knap, INPUT_PULLUP);

  timeToRead_ultralyd = millis();
  timeToRead_DHL = millis();
  timeToRead_failsafe = millis();
  sound = 340;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int tmp;
bool bugUltralyd = false;

void loop() {

  ESPNOW();

  kanp();

  sensorstyring("DHL sensor", timeToRead_DHL, delay_DHL, DHLsensor);
  sensorstyring("Ultralyd sensor", timeToRead_ultralyd, delay_ultralyd, ultralyd);

  if (sensorState == true) {
    tmp = myData.afstand;

    procentvand = map(tmp, 0, maxafstand, 100, 0);
    if (procentvand >= 100) {
      procentvand = 404;
    } else if (procentvand < 0) {
      procentvand = 0;
    }
  }
  sensorstyring("vandstands sensor", timeToRead_failsafe, delay_failsafe, failsafe);



  if (myData.failsafeAvg >= 300) {
    if (false == alarmfailsafe) {
      digitalWrite(relayPin, HIGH);
      relay = true;
      // lock = false;
      Serial.println("Failsafe alarm");
      alarmfailsafe = true;
      alarmred = false;
      alarmyellow = false;
      log('f');
    }
  }

  if (myData.failsafeAvg < 300) {
    if (procentvand == 404) {
      if (bugUltralyd == false) {
        Serial.println();
        Serial.println();
        Serial.print('\t');
        Serial.print('\t');
        Serial.println("BUG ULTRALYD");
        Serial.println();
        Serial.println();
        bugUltralyd = true;
      }

    } else if (procentvand > 80) {
      if (false == alarmred) {
        digitalWrite(relayPin, HIGH);
        relay = true;
        // lock = false;
        // Serial.println("Alarm red");
        alarmred = true;
        alarmyellow = false;
        alarmfailsafe = false;
        bugUltralyd = false;
        log('r');
      }
    } else if (80 > procentvand && procentvand > 50) {
      if (alarmyellow == false) {
        digitalWrite(relayPin, HIGH);
        // Serial.println("Alarm yellow");
        relay = true;
        alarmyellow = true;
        alarmred = false;
        alarmfailsafe = false;
        bugUltralyd = false;
        log('y');
      }

    } else if (procentvand < 50) {
      if (alarmyellow == true || alarmred == true || alarmfailsafe == true) {
        digitalWrite(relayPin, LOW);
        // Serial.println("Ingen alarm");
        relay = false;
        alarmyellow = false;
        alarmred = false;
        alarmfailsafe = false;
        bugUltralyd = false;
        log('n');
      }
    }
  }


  if (alarmfailsafe == true) {
    alarmtid = millis();
    if (1000 <= alarmtid - alarmstart) {
      tone(AlarmPin, 1000, 150);
      alarmstart = alarmtid;
    }

  } else if (alarmred == true || alarmyellow == true) {
    alarmtid = millis();
    if (1000 <= alarmtid - alarmstart) {
      if (alarmred == true) {
        tone(AlarmPin, 1000, 150);
      } else if (alarmyellow == true) {
        tone(AlarmPin, 2000, 150);
      }
      alarmstart = alarmtid;
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////