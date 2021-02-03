int led = 10;
#define led 10

void setup() {

  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(true){
  digitalWrite(led,HIGH);
  Serial.println("Led on");
  delay(1000);
  digitalWrite(led,LOW);
  Serial.println("Led off");
  delay(1000);
    }
}
