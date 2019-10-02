
#include <ESP8266WiFi.h>
#define BAUD_RATE 115200

const char *ssid = "SR";
const char *password = "98765432";
const char* host = "13.125.248.87";

char buff[20];
int count = 0;
int send_flag = 0;
int fall_flag = 0;
const long interval = 1000; 
unsigned long previousMillis = 0;
String heartRate;

void setup() {
  
  Serial.begin(115200);
  delay(10);
  
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}


void loop() {

  unsigned long currentMillis = millis();
  
  WiFiClient client;
  const int httpPort = 80;
  if(!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  
  String read_url = "/Hrespond.php?devNumber=testNumber";
  String send_url = "/send.php?devNumber=testNumber&heartRate=";
  String fall_url = "/emergancy.php?devNumber=testNumber";
  
  //보드에서 읽어오는 부분
  while(Serial.available() > 0) {
    if('<' == Serial.read()) {
      String HR = Serial.readStringUntil('>');
      send_url += HR;
      send_flag = 1;
      
      Serial.println(HR);
      Serial.println(send_url);
    }
    else if('@' == Serial.read()) {
      fall_flag = 1;
    }
  }

  //심박수 값이 들어왔을 때만 서버로 업로드
  if(send_flag) {
    client.print(String("GET ") + send_url + " HTTP/1.1\r\n" +
        "Host: " + host + "\r\n" + 
        "Connection: close\r\n\r\n");
    send_flag = 0;
    Serial.println("heartrate upload");
  }
  
  //넘어졌을 때만 서버로 업로드
  if(fall_flag){
    client.print(String("GET ") + fall_url + " HTTP/1.1\r\n" +
    "Host: " + host + "\r\n" + 
    "Connection: close\r\n\r\n");
    fall_flag = 0;
    Serial.println("fall upload");
  }
  
  //1초마다 서버에서 명령어를 받아오고 보드로 보내는 부분
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    client.print(String("GET ") + read_url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
         
    int timeout = millis() + 5000;
    while (client.available() == 0) {
      if (timeout - millis() < 0) {
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }     
    }
    
    String str = "";
    
    while(client.available()){
      String line = client.readStringUntil('\r');
      str = line;
      if(str.indexOf("testNumber,") != -1) {
        int start_index = str.indexOf("testNumber,")+11;
        Serial.print('[');
        Serial.print(str.charAt(start_index));
        Serial.print(']');
      }
    }
    Serial.println();
    Serial.println("closing connection");
  
  }
  
}
