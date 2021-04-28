float sinVal;
int toneVal;
int led = 10;
int val = 0;

unsigned long tepTimer ;
void setup(){
pinMode(8, OUTPUT);
pinMode(led,OUTPUT);
Serial.begin(9600);

 // configure pin of buzzer
Serial.begin(9600);
 // configure baud rate to 9600 bps
}
void loop(){
int val;
 //save the value of LM35
double data;
val = analogRead(1);
 // save the converted value of temperature
val=analogRead(1);
 //Connect LM35 to analog pin and read
//value from it // Read voltage value ranging from 0 -1023
Serial.println(val);
 // read voltage value from serial monitor
data = (double) val * (5/10.24); //Convert the voltage value to
//temperature value
if(data>27){ //If temperature is higher than 27, the buzzer starts to
  //digitalWrite(led,HIGH);
  delay(1000);
  //digitalWrite(led,LOW);
  delay(1000);
//make sound.
for(int x=0; x<180; x++){
//Convert sin function to radian
sinVal = (sin(x*(3.1412/180)));
//Use sin function to generate frequency of sound
toneVal = 2000+(int(sinVal*1000));
//Configure the buzzer pin 8
//tone(8, toneVal);
delay(1);
}
} else {
 // If the temperature is lower than
//27, turn off the buzzer
//noTone(8);
 // Turn off the buzzer
}
if(millis() - tepTimer > 50){ // Every 500 ms, serial port outputs
  if(val<1000 && data < 27){
 // If lower than 1000, turn off LED
digitalWrite(led,LOW);
noTone(8);
}else{
 // Otherwise turn on LED
digitalWrite(led,HIGH);
tone(8,toneVal);
}
delay(10);
//temperature value.
tepTimer = millis();
Serial.print("temperature: "); // Serial port outputs temperature
Serial.print(data);
 // Serial port outputs temperature
//value
Serial.println("C");
 // Serial port output temperature
//unit
}
}
