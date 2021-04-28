#include <Arduino_MKRGPS.h>
#define greenLed 6
#define blueLed 7
#define redLed 8
#define buzzer 5
#define tempPin A6
float temp,gpslat,gpslon;

void setup() {
  // put your setup code here, to run once:
  pinMode(greenLed,OUTPUT);
  pinMode(blueLed,OUTPUT);
  pinMode(redLed,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(tempPin,INPUT);
  Serial.begin(9600);
//  while(!Serial){
//    Serial.println("Serial Port not yet connected");
//    }
//    if(!GPS.begin()){
//      Serial.println("Failed to initialize GPS!!");
//      }
    
}

void loop() {
  // put your main code here, to run repeatedly:
// if(GPS.available()){
//  float latitude = GPS.latitude();
//  float longitude = GPS.longitude();
//  float altitude = GPS.altitude();
//  float speed = GPS.speed();
//  int satellites = GPS.satellites();
//
//  Serial.print("Location: ");
//  Serial.print(latitude,7);
//  Serial.print(",");
//  Serial.print(longitude,7);
//
//  Serial.print("Altitude: ");
//  Serial.print(altitude);
//  Serial.print("m");
//
//  Serial.print("Ground speed: ");
//  Serial.print(speed);
//  Serial.print(" km/h");
//  Serial.println();
//  }
  temp = analogRead(tempPin);
  temp = temp*0.48828125;
  Serial.print("Temperature=");
  Serial.print(temp);
  Serial.println("*C");
  digitalWrite(buzzer,HIGH);
  digitalWrite(greenLed,HIGH);
  delay(1000);
  digitalWrite(greenLed,LOW);
  delay(1000);
  digitalWrite(blueLed,HIGH);
  delay(1000);
  digitalWrite(blueLed,LOW);
  delay(1000);
  digitalWrite(redLed,HIGH);
  buzzerStatus(digitalRead(redLed));
  delay(1000);
  digitalWrite(redLed,LOW);
  buzzerStatus(digitalRead(redLed));
  delay(1000);

}

void buzzerStatus(int s){
  if(s==1){
    digitalWrite(buzzer,HIGH);
    Serial.println(s);
    }else{
    digitalWrite(buzzer,LOW);
    Serial.println(s);
      }
  }
