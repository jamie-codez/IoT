#define pin6 6
void setup() {
  // put your setup code here, to run once:
  pinMode(pin6,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(pin6,HIGH);
delay(1000);
digitalWrite(pin6,LOW);
delay(1000);
}
