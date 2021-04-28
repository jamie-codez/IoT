#include <SoftwareSerial.h>
int pin10=10,led11=11,led12=12,led13=13,tempPin=0;
float temp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pin10,OUTPUT);
  pinMode(led11,OUTPUT);
  pinMode(led12,OUTPUT);
  pinMode(led13,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
temp = analogRead(tempPin);
temp = temp *0.48828125;
if(temp<30){
  digitalWrite(led11,HIGH);
  }else{
    digitalWrite(led13,HIGH);
    digitalWrite(pin10,HIGH);
    if(led11==HIGH){
      digitalWrite(led11,LOW);
      }
    }
Serial.print("Temperature=");
Serial.print(temp);
Serial.println("*C");
delay(1000);
//digitalWrite(led11,HIGH);
//Serial.println("LED 11 on");
//delay(500);
//digitalWrite(led11,LOW);
//Serial.println("LED 11 off");
//delay(500);
digitalWrite(led12,HIGH);
Serial.println("LED 12 on");
delay(500);
digitalWrite(led12,LOW);
Serial.println("LED 12 off");
delay(500);
//digitalWrite(led13,HIGH);
//digitalWrite(pin10,HIGH);
//Serial.println("LED 13 on");
//delay(500);
//digitalWrite(led13,LOW);
//digitalWrite(pin10,LOW);
//Serial.println("LED 13 off");
//delay(500);
}
