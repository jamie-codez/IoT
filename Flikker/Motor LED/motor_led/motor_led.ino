int redLED=13,greenLED=12, motorPin=6;

void setup() {
  // put your setup code here, to run once:
  pinMode(redLED,OUTPUT);
  pinMode(greenLED,OUTPUT);
  pinMode(motorPin,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(redLED,HIGH);
  delay(1000);
  if(redLED==HIGH){
    digitalWrite(greenLED,LOW);
    digitalWrite(motorPin,LOW);
    }else{
    digitalWrite(redLED,HIGH);
    digitalWrite(greenLED,HIGH);
    digitalWrite(motorPin,HIGH);
    }
    delay(5000);

}
