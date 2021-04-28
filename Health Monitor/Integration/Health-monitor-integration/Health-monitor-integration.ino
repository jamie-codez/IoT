#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <TinyGPS.h>
#include <string.h>

#define DEBUG true
#define bpPin A5
#define WAITING_TIME 15
#define bodyTempPin A6
bool debug = false;
int PWR_KEY = 1,RST_KEY = 2,LOW_PWR_KEY = 3;
int rxPin = 13,txPin=14;
int bloodPressure;
float lat=0.0f,lon=0.0f,speed=0.0f,alt=0;
int buzzerPin = 5,greenLedPin = 6,blueLedPin = 7,redLedPin = 8;
TinyGPS gps;
bool ModuleState =false;
unsigned long timeCount;
float internalTemp,bodyTemperature;
String internalTemperature;
struct SigFoxMessage{
  int8_t bodyTemp;
  int8_t latitude;
  int8_t longitude;
  int8_t altitude;
  int8_t speed;
  int8_t bp;
  };
void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  SerialUSB.begin(115200);
  Serial.println("Booting......");
  Serial.println("Welcome");
  pinMode(greenLedPin,OUTPUT);
  pinMode(blueLedPin,OUTPUT);
  pinMode(redLedPin,OUTPUT);
  pinMode(buzzerPin,OUTPUT);
  pinMode(PWR_KEY,OUTPUT);
  pinMode(RST_KEY,OUTPUT);
  pinMode(LOW_PWR_KEY,OUTPUT);
  digitalWrite(RST_KEY,LOW);
  digitalWrite(LOW_PWR_KEY,HIGH);
  digitalWrite(PWR_KEY,HIGH);
  testSystem("test");
  while(!SerialUSB){
    Serial.print("Waiting for serial usb to connect...");
    }
    digitalWrite(PWR_KEY,LOW);
    delay(3000);
    digitalWrite(PWR_KEY,HIGH);
    delay(5000);
    ModuleState = moduleStateCheck();
    sendCommandToA9G("AT+GPS=1",1000,DEBUG);//1.turns on gps 0.turns off gps
    sendCommandToA9G("AT+GPSRD=10", 1000, DEBUG);//Read nmea information after 10 seconds
    if(!SigFox.begin()){
      Serial.println("Shelid error or not present...");
      return;
      }
      SigFox.debug();//Remove this line when going to production
      String version = SigFox.SigVersion();
      String ID = SigFox.ID();
      String PAC = SigFox.PAC();
      //Display module information
      Serial.println("First configuration");
      Serial.println("SigFox FW version "+version);
      Serial.println("ID = "+ID);
      Serial.println("PAC = "+PAC);
      Serial.println();
      delay(100);
      SigFox.end();
      String mess = "Configuration message";
      mess.trim();
      if(mess.length()>12){
        Serial.println("Message too long only first 12 bytes will be sent...");
        }
      Serial.println("Sending configuration message"+mess);
      Serial.println("Getting the response will take up to 50 seconds");
      Serial.println("The LED will blink while the operation is ongoing");
      sendConfigMessage(mess);
}
void testSystem(String test){
  if(test=="test"){
    digitalWrite(greenLedPin,HIGH);
    digitalWrite(blueLedPin,HIGH);
    digitalWrite(redLedPin,HIGH);
    digitalWrite(buzzerPin,HIGH);
    delay(5000);
    digitalWrite(greenLedPin,LOW);
    digitalWrite(blueLedPin,LOW);
    digitalWrite(redLedPin,LOW);
    digitalWrite(buzzerPin,LOW);
  }
}
void sendConfigMessage(String msg){
  SigFox.begin();
  delay(100);
  SigFox.status();
  delay(1);
  SigFox.beginPacket();
  SigFox.print(msg);
}

bool moduleStateCheck(){
  int i = 0;
  bool state = false;
  for(i=0;i<10;i++){
    String msg = String("");
    msg = sendCommandToA9G("AT",1000,DEBUG);
    if(msg.indexOf("OK")){
      SerialUSB.println("A9G module turned on.");
      state = true;
      return state;
      }
      delay(500);
    }
    return state;
  }

void Wait(int m,bool s){
  if(debug){
    Serial.print("Waiting: ");Serial.print(m);Serial.println("mins.");
    }
    digitalWrite(LED_BUILTIN,LOW);
    if(s){
      int seg = m*30;
      for(int i=0;i<seg;i++){
        digitalWrite(LED_BUILTIN,HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN,LOW);
        delay(1000);
        }
      }else{
        int seg = m*15;
        for(int i=0;i<seg;i++){
          digitalWrite(LED_BUILTIN,HIGH);
          delay(1000);
          digitalWrite(LED_BUILTIN,LOW);
          delay(1000);
          }
        }
  }

  String sendCommandToA9G(String command,const int timeout,boolean debug){
    String response  = "";
    Serial1.println(command);
    long int time = millis();
    while((time + timeout)>millis()){
      while(Serial1.available()){
        char c = Serial1.read();
        response +=c;
        }
      }
      if(debug){
        SerialUSB.println(response);
        }
        return response;
}

void readCoordinates(){
  if(millis()-timeCount>5000){
    
    String nmeaData = sendCommandToA9G("AT+LOCATION=2",1000,DEBUG);
    char conv;
    for(int i=0;i<=nmeaData.length();i++){
      conv+=nmeaData[i];
      }
    if(gps.encode(conv)){
      gps.f_get_position(&lat,&lon);
      alt = gps.altitude()/100;
      }
    timeCount=millis();//refresh
    }
    while(Serial1.available()>0){
      SerialUSB.write(Serial1.read());
      yield();
     }
      while(SerialUSB.available()>0){
       Serial1.write(SerialUSB.read());
       yield();
     }
}
 

void readInternalTemperature(){
  internalTemp = SigFox.internalTemperature();
  Serial.println(internalTemp);
}
void readBodyTemp(){
  bodyTemperature = analogRead(bodyTempPin);
  bodyTemperature = (500 * bodyTemperature) /1024;
  Serial.print("Body temperature: ");Serial.print(bodyTemperature);
}
void readBloodPressure(){
 bloodPressure = analogRead(bpPin);
 Serial.println("Blood pressure: "+bloodPressure);
}
void sendDataToSigFox(SigFoxMessage msg){
  if(debug){
    Serial.print("Sending: ");
    Serial.print("Body temp: ");Serial.println(msg.bodyTemp);
    Serial.print("latitude: ");Serial.println(msg.latitude);
    Serial.print("longotude: ");Serial.println(msg.longitude);
    Serial.print("altitude: ");Serial.println(msg.altitude);
    Serial.print("speed: ");Serial.println(msg.speed);
    Serial.print("Blood Pressure: ");Serial.println(msg.bp);
    if(sizeof(msg)>12){
      Serial.println("Message too long only first 12 bytes will be sent");
      }
    }
    //remove white spaces
//    msg.trim();
    SigFox.begin();
    delay(100);//after first configuration
    SigFox.status();//clea all pending interrupts
    delay(1);
    if(debug) SigFox.debug();
    delay(100);

    //beginpacket
    SigFox.beginPacket();
    SigFox.print(msg.bodyTemp);

    if(debug){
      int retPack = SigFox.endPacket(true);//send buffer and wait for response
      if(retPack>0){
        Serial.println("No transmission");
        }else{
          Serial.println("Transmission ok");
          }
          Serial.println(SigFox.status(SIGFOX));
          Serial.println(SigFox.status(ATMEL));

          if(SigFox.parsePacket()){
            Serial.println("Response from server: ");
            while(SigFox.available()){
              Serial.print("0x");
              Serial.println(SigFox.read(),HEX);
              }
            }else{
              Serial.println("Could not get any response from the server");
              Serial.println("Check the SigFox coverage in your area");
              Serial.println("If you are indoor, check the 20dB coverage or move near a window");
            }
            Serial.println();
      }else{
        SigFox.endPacket();
      }
    SigFox.end();
}
void sendTextMessage(SigFoxMessage message,String condition){
  bool isStillOn = moduleStateCheck();
  if(isStillOn){
    sendCommandToA9G("AT+CMGF=1",1000,DEBUG);
    sendCommandToA9G("AT+CGMS=\"0759459363\"",1000,DEBUG);
                      Serial1.println(condition);
                      Serial1.print("Temperature: ");Serial1.println(message.bodyTemp);
                      Serial1.print("Blood pressure: ");Serial1.println(message.bp);
                      Serial1.print("Latitude: ");Serial1.println(message.latitude);
                      Serial1.print("Longitude: ");Serial1.println(message.longitude);
                      Serial1.print("Speed: ");Serial1.println(message.speed);
                      Serial1.print("https://www.google.com/maps/@");
                      Serial1.print(message.latitude);Serial1.print(",");Serial.print(message.longitude);
                      Serial1.write(26);
    
  }
}
void lightingNotification(int led){
  digitalWrite(led,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
readInternalTemperature();
readBodyTemp();
readBloodPressure();
readCoordinates();
SigFoxMessage message;
message.bodyTemp = bodyTemperature;
message.latitude = lat;
message.longitude = lon;
message.altitude = alt;
message.speed = speed;
message.bp = bloodPressure;
sendDataToSigFox(message);
if(message.bodyTemp>30 && message.bodyTemp<35){
  if(message.bp<100){
    sendTextMessage(message,"Cold:Abn-cold");
    lightingNotification(blueLedPin);
  }else{
    digitalWrite(blueLedPin,LOW);
  }
}else if(message.bodyTemp>35 && message.bodyTemp<38){
  if(message.bp>120 && message.bp<160){
    sendTextMessage(message,"Optimal:Fine-cond");
    lightingNotification(greenLedPin);
  }else{
    digitalWrite(greenLedPin,LOW);
  }
}else if(message.bodyTemp>39){
  if(message.bp>160){
    sendTextMessage(message,"Critical:Critical");
    lightingNotification(redLedPin);
    digitalWrite(buzzerPin,HIGH);
  }else{
    digitalWrite(buzzerPin,LOW);
  }
}
Wait(WAITING_TIME,false);
}
