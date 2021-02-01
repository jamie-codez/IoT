float sinVal;
int toneVal;


void setup() {
  // put your setup code here, to run once:
pinMode(8,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0;i<=180;i++){
    sinVal = (sin(i*(3.1412/180)));
    toneVal = 2000+(int(sinVal*1000));
    tone(8,toneVal);
    delay(2);
    }

}
