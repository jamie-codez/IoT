void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.println("AT");
  delay(1000);
  Serial.println("AT+CPOWD=1");
  delay(7000);
}
