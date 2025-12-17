//biblioteker
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <SPI.h>

const int BUZZER = 13;

bool alarmGreen;
bool alarmYellow;
bool alarmRed;
bool alarmFS;
bool noAlarm;
bool datareciv;

//til alarmerne nå de siger bib bib bib
unsigned long timer = 0;
unsigned long alarmtime = 0;

TFT_eSPI tft = TFT_eSPI();  // TFT display objekt

//indeholder de værdier der sendes med ESP-NOW
typedef struct MessegeData {
  float afstand;   //distance variable
  float sAfstand;  //distance variable
  float dt;
  float height;
  unsigned long sendStart;
  float humidity;
  float temperature;
  int failsafeAvg;
  unsigned int packets;

} MessegeData;


MessegeData myData;

int hoejdePct = 0;

unsigned long recivet = 0;
unsigned int countreeding;
unsigned long timestamp = 0;
unsigned long lastrecivet = 0;

unsigned long recieveTime = 0;
unsigned long transmissionTime = 0;
int datasize;
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  recieveTime = micros();
  memcpy(&myData, incomingData, sizeof(myData));
  transmissionTime = recieveTime - myData.sendStart;
  datasize = len;
  // timestamp = lastrecivet ;
  if (recivet == 0)
    ;
  else {
    lastrecivet = millis() - recivet;
    timestamp += lastrecivet;
  }
  countreeding++;
  recivet = millis();
  datareciv = true;
}



void printDataToDisplay(int x) {
  //tft.fillScreen(TFT_BLACK);


  // Vandstandshøjde
  printLevel(x);

  //tft.println("ESP-NOW Receiver");
  // tft.setTextColor(TFT_CYAN);
  // tft.print("MAC: ");
  //tft.println(macStr);
  tft.setCursor(120, 60);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.print(myData.afstand);
  tft.print(" cm ");
  tft.setCursor(120, 75);
  tft.print(myData.humidity);
  //tft.print(lastrecivet);
  tft.print(" % ");
  tft.setCursor(120, 90);
  //tft.print(myData.packets - countreeding);
  tft.print(myData.temperature);
  tft.print(" C ");

  //  tft.print(myData.sAfstand);
  //  tft.print(myData.height);

  if (myData.failsafeAvg > 300) {
    alarmFS = true;
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(0, 115);
    tft.print("FAILSAFE ALARM");
  } else {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(0, 115);
    tft.print("               ");
    alarmFS = false;
  }
}


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(BUZZER, OUTPUT);


  tft.init();
  tft.setRotation(1);  // Tilpas hvis nødvendigt
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  delay(1000);

  printDisplayInit();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("macAddress: ");
  Serial.println(WiFi.macAddress());

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);  // Sæt kanal 1
  // ACTIVATE LONG RANGE PROTOCOL:
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);


  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_wifi_set_max_tx_power(80);
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}



void loop() {
  if (datareciv == true) {

    size_t myDatasize = sizeof(myData);

    int maxReading = 80;  //kan måle en max afstand på 80cm her
    int tmp = myData.afstand;
    hoejdePct = map(tmp, 0, maxReading, 100, 0);  // omvendt skalering
    if (hoejdePct >= 100) {
      hoejdePct = 404;
    } else if (hoejdePct < 0) {
      hoejdePct = 0;
    }




      alarmState(hoejdePct);
      printDataToDisplay(hoejdePct);

      Serial.println();
      //data vises på serial monitor
      Serial.println("_______________");
      Serial.print("Data size: ");
      Serial.print(myDatasize);
      Serial.println(" bytes");


      Serial.println("_______________");

      Serial.print("Timestamp: ");
      Serial.print(timestamp);
      Serial.print(" ms");
      Serial.print("\t");
      Serial.print(recivet);
      Serial.println(" ms");

      Serial.print("Modtaget nr: ");
      Serial.println(countreeding);
      Serial.print("Pakke: ");
      Serial.print(myData.packets);
      Serial.print("\t");
      Serial.print("Pakker ikke modtaget: ");
      Serial.println(myData.packets - countreeding);

      Serial.print("Sidst modtaget for: ");
      Serial.print(lastrecivet);
      Serial.println(" ms");

      Serial.print("Modtager tid: ");
      Serial.print(transmissionTime);
      Serial.println(" ms");

      Serial.print("Bytes received: ");
      Serial.println(datasize);
      Serial.println("_______________");
      Serial.print("Afstand: ");
      Serial.println(myData.afstand);

      Serial.print("StartAfstand: ");
      Serial.println(myData.sAfstand);

      Serial.print("deltaTid: ");
      Serial.println(myData.dt);

      Serial.print("Height: ");
      Serial.println(myData.height);

      Serial.print("humidity: ");
      Serial.println(myData.humidity);

      Serial.print("failsafeAvg: ");
      Serial.println(myData.failsafeAvg);

      Serial.print("temperatur: ");
      Serial.println(myData.temperature);



      datareciv = false;
    }

    alarmTurnOn();
  }