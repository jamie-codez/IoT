#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <TinyGPS.h>
//#include <string.h>

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
struct SigFoxTemp{
  float bodyTemp;};
  struct SigFoxLat{
  float latitude;};
  struct SigFoxLong{
  float longitude;};
  struct SigFoxAlt{
  float altitude;};
  struct SigFoxSpeed{
  float speed;};
  struct SigFoxBP{
  float bp;
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
      String mess = "Config text";
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
  SigFox.endPacket();
  SigFox.end();
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
  sendCommandToA9G("AT+GPS=1",1000,DEBUG);
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
  bodyTemperature = bodyTemperature/2.048;
  Serial.print("Body temperature: ");Serial.print(bodyTemperature);
}
void readBloodPressure(){
 bloodPressure = analogRead(bpPin);
 Serial.println("Blood pressure: "+bloodPressure);
}
void sendTempToSigFox(SigFoxTemp temp){
  if(debug){
    Serial.print("Sending: ");
    Serial.print("Body temp: ");Serial.println(temp.bodyTemp);
//    Serial.print("latitude: ");Serial.println(msg.latitude);
//    Serial.print("longotude: ");Serial.println(msg.longitude);
//    Serial.print("altitude: ");Serial.println(msg.altitude);
//    Serial.print("speed: ");Serial.println(msg.speed);
//    Serial.print("Blood Pressure: ");Serial.println(msg.bp);
    if(sizeof(temp.bodyTemp)>12){
      Serial.println("Message too long only first 12 bytes will be sent");
      }
    }
    //remove white spaces
//    msg.trim();
    SigFox.begin();
    delay(100);//after first configuration
    SigFox.status();//clear all pending interrupts
    delay(1);
    if(debug) SigFox.debug();
    delay(100);

    //beginpacket
    SigFox.beginPacket();
    SigFox.print(temp.bodyTemp);

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
void sendLatToSigFox(SigFoxLat lat){
  if(debug){
    Serial.print("Sending: ");
    Serial.print("Body temp: ");Serial.println(lat.latitude);
//    Serial.print("latitude: ");Serial.println(msg.latitude);
//    Serial.print("longotude: ");Serial.println(msg.longitude);
//    Serial.print("altitude: ");Serial.println(msg.altitude);
//    Serial.print("speed: ");Serial.println(msg.speed);
//    Serial.print("Blood Pressure: ");Serial.println(msg.bp);
    if(sizeof(lat.latitude)>12){
      Serial.println("Message too long only first 12 bytes will be sent");
      }
    }
    //remove white spaces
//    msg.trim();
    SigFox.begin();
    delay(100);//after first configuration
    SigFox.status();//clear all pending interrupts
    delay(1);
    if(debug) SigFox.debug();
    delay(100);

    //beginpacket
    SigFox.beginPacket();
    SigFox.print(lat.latitude);

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
void sendLongToSigFox(SigFoxLong longi){
  if(debug){
    Serial.print("Sending: ");
    Serial.print("Body temp: ");Serial.println(longi.longitude);
//    Serial.print("latitude: ");Serial.println(msg.latitude);
//    Serial.print("longotude: ");Serial.println(msg.longitude);
//    Serial.print("altitude: ");Serial.println(msg.altitude);
//    Serial.print("speed: ");Serial.println(msg.speed);
//    Serial.print("Blood Pressure: ");Serial.println(msg.bp);
    if(sizeof(longi.longitude)>12){
      Serial.println("Message too long only first 12 bytes will be sent");
      }
    }
    //remove white spaces
//    msg.trim();
    SigFox.begin();
    delay(100);//after first configuration
    SigFox.status();//clear all pending interrupts
    delay(1);
    if(debug) SigFox.debug();
    delay(100);

    //beginpacket
    SigFox.beginPacket();
    SigFox.print(longi.longitude);

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
void sendAltToSigFox(SigFoxAlt alt){
  if(debug){
    Serial.print("Sending: ");
    Serial.print("Body temp: ");Serial.println(alt.altitude);
//    Serial.print("latitude: ");Serial.println(msg.latitude);
//    Serial.print("longotude: ");Serial.println(msg.longitude);
//    Serial.print("altitude: ");Serial.println(msg.altitude);
//    Serial.print("speed: ");Serial.println(msg.speed);
//    Serial.print("Blood Pressure: ");Serial.println(msg.bp);
    if(sizeof(alt)>12){
      Serial.println("Message too long only first 12 bytes will be sent");
      }
    }
    //remove white spaces
//    msg.trim();
    SigFox.begin();
    delay(100);//after first configuration
    SigFox.status();//clear all pending interrupts
    delay(1);
    if(debug) SigFox.debug();
    delay(100);

    //beginpacket
    SigFox.beginPacket();
    SigFox.print(alt.altitude);

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
void sendSpeedToSigFox(SigFoxSpeed mSpeed){
  if(debug){
    Serial.print("Sending: ");
    Serial.print("Body temp: ");Serial.println(mSpeed.speed);
//    Serial.print("latitude: ");Serial.println(msg.latitude);
//    Serial.print("longotude: ");Serial.println(msg.longitude);
//    Serial.print("altitude: ");Serial.println(msg.altitude);
//    Serial.print("speed: ");Serial.println(msg.speed);
//    Serial.print("Blood Pressure: ");Serial.println(msg.bp);
    if(sizeof(mSpeed)>12){
      Serial.println("Message too long only first 12 bytes will be sent");
      }
    }
    //remove white spaces
//    msg.trim();
    SigFox.begin();
    delay(100);//after first configuration
    SigFox.status();//clear all pending interrupts
    delay(1);
    if(debug) SigFox.debug();
    delay(100);

    //beginpacket
    SigFox.beginPacket();
    SigFox.print(mSpeed.speed);

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
void sendBPToSigFox(SigFoxBP bp){
  if(debug){
    Serial.print("Sending: ");
    Serial.print("Body temp: ");Serial.println(bp.bp);
//    Serial.print("latitude: ");Serial.println(msg.latitude);
//    Serial.print("longotude: ");Serial.println(msg.longitude);
//    Serial.print("altitude: ");Serial.println(msg.altitude);
//    Serial.print("speed: ");Serial.println(msg.speed);
//    Serial.print("Blood Pressure: ");Serial.println(msg.bp);
    if(sizeof(bp)>12){
      Serial.println("Message too long only first 12 bytes will be sent");
      }
    }
    //remove white spaces
//    msg.trim();
    SigFox.begin();
    delay(100);//after first configuration
    SigFox.status();//clear all pending interrupts
    delay(1);
    if(debug) SigFox.debug();
    delay(100);

    //beginpacket
    SigFox.beginPacket();
    SigFox.print(bp.bp);

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
void sendTextMessage(String condition,SigFoxTemp temp,SigFoxLat lati,SigFoxLong longi,SigFoxAlt alti,SigFoxSpeed mSpeed,SigFoxBP bp){
  bool isStillOn = moduleStateCheck();
  if(isStillOn){
    sendCommandToA9G("AT+CMGF=1",1000,DEBUG);
    sendCommandToA9G("AT+CGMS=\"0759459363\"",1000,DEBUG);
                      Serial1.println(condition);
                      Serial1.print("Temperature: ");Serial1.println(temp.bodyTemp);
                      Serial1.print("Blood pressure: ");Serial1.println(bp.bp);
                      Serial1.print("Latitude: ");Serial1.println(lati.latitude);
                      Serial1.print("Longitude: ");Serial1.println(longi.longitude);
                      Serial1.print("Speed: ");Serial1.println(mSpeed.speed);
                      Serial1.print("Condition: ");Serial1.println(condition);
                      Serial1.print("https://www.google.com/maps/@");
                      Serial1.print(lati.latitude);Serial1.print(",");Serial.print(longi.longitude);
    
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
SigFoxTemp temp;
SigFoxLat lati;
SigFoxLong longi;
SigFoxAlt alti;
SigFoxSpeed mSpeed;
SigFoxBP bp;
temp.bodyTemp = bodyTemperature;
lati.latitude = lat;
longi.longitude = lon;
alti.altitude = alt;
mSpeed.speed = speed;
bp.bp = bloodPressure;
sendTempToSigFox(temp);
sendLatToSigFox(lati);
sendLongToSigFox(longi);
sendAltToSigFox(alti);
sendSpeedToSigFox(mSpeed);
sendBPToSigFox(bp);

if(temp.bodyTemp>80 && temp.bodyTemp<100){
  if(bp.bp<100){
    sendTextMessage("Cold:Abn-cold",temp,lati,longi,alti,mSpeed,bp);
    lightingNotification(blueLedPin);
  }else{
    digitalWrite(blueLedPin,LOW);
  }
}else if(temp.bodyTemp>100 && temp.bodyTemp<110){
  if(bp.bp>120 && bp.bp<160){
    sendTextMessage("Optimal:Fine-cond",temp,lati,longi,alti,mSpeed,bp);
    lightingNotification(greenLedPin);
  }else{
    digitalWrite(greenLedPin,LOW);
  }
}else if(temp.bodyTemp>110){
  if(bp.bp>160){
    sendTextMessage("Critical:Critical",temp,lati,longi,alti,mSpeed,bp);
    lightingNotification(redLedPin);
    digitalWrite(buzzerPin,HIGH);
  }else{
    digitalWrite(buzzerPin,LOW);
  }
}
Wait(WAITING_TIME,false);
}
