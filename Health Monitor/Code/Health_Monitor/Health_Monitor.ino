#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <TinyGPS.h>

#define WAITING_TIME 15
#define GPS_PIN 2
#define GPS_INFO_BUFFER_SIZE 128

bool debug = false;
TinyGPS gps;

/***
 * GPS data variables
 */
 int year;
 byte month,day,hour,minute,second,hundredths;
 unsigned long chars;
 unsigned short sentences, failed_checksum;
 char GPS_info_char;
 char GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
 unsigned int received_char;
 bool message_started = false;
 int i = 0;
 /***
  * GPS coordinate structure, 12 byte size on 32 bits platform
  */
  struct gpscoordinates{
    float a_latitude;
    float a_longitude;
    float a_altitude;
    };

    float latitude = 0.0f;
    float longitude = 0.0f;
    float altitude = 0;

    void Wait(int m,bool s){
    /***
     * m - minutes to wait
     * s - slow led pulse
     */
     if(debug){
      Serial.print("Waiting:");Serial.print(m);Serial.print(" minuz.");
      }
      digitalWrite(LED_BUILTIN,LOW);
     if(s){
      int seg = m*30;
      for(int i = 0;i<seg;i++){
        digitalWrite(LED_BUILTIN,HIGH);//Turn led on
        delay(1000);
        digitalWrite(LED_BUILTIN,LOW);
        delay(1000);
        }
      }else{
        int seg = m*15;
        for(int i = 0;i<seg;i++){
          digitalWrite(LED_BUILTIN,HIGH);
          delay(1000);
          digitalWrite(LED_BUILTIN,LOW);
          delay(3000);
          }
        }
      }
      /***
       * Send data to sigfox
       */
       void SendSigFox(String data){
        if(debug){
          Serial.print("Sending: ");Serial.println(data);
          if(data.length()>12){
            Serial.println("Message too long,only first 12 bytes will be sent");
            }
          }
          //Remove whitespace using data.trim()
          //start module
          SigFox.begin();
          //wait for atleast 30ms after first configuration (100ms before)
          delay(100);
          //clear all pending interrupts
          SigFox.status();
          delay(1);
          if(debug){
            SigFox.debug();
            delay(100);
            }
            SigFox.beginPacket();
            SigFox.print(data);

            if(debug){ 
              int ret = SigFox.endPacket(true);//Send buffer to sigfox network and wait
              if(ret>0){
                Serial.println("No transmission");
                }else{
                  Serial.println("Transmission ok");
                  }
                  Serial.println(SigFox.status(SIGFOX));
                  Serial.println(SigFox.status(ATMEL));

                  if(SigFox.parsePacket()){
                    Serial.println("Response from server");
                    while(SigFox.available()){
                      Serial.print("0x");
                      Serial.println(SigFox.read(),HEX);
                      }
                    }else{
                    Serial.println("Could not get any response fro server");
                    Serial.println("Check SigFox coverage in your area");
                    Serial.println("If you are outside check the 20dB coverage or move near a window");
                    }
                  Serial.println();
              }else{
                SigFox.endPacket();
                }
                SigFox.end();
        }
        /***
         * Convert gps function
         * Convert gps info from float to char data 
         */
         String ConvertGPSdata(const void* data,uint8_t len){
          uint8_t* bytes = (uint8_t*)data;
          String cadena;
          if(debug){
            Serial.print("Length: ");Serial.println(len);
            }
            for(uint8_t i=len-1;i<len;--i){
              if(bytes[i]<12){
                cadena.concat(byte(0));//not tested
                }
                cadena.concat(char (bytes[i]));
                if(debug) Serial.print(bytes[i],HEX);
              }
              if(debug){
                Serial.println("");
                Serial.println("Strint to send: ");Serial.println(cadena);
                }
                return cadena;  
          }
          String GetGPSposition(){
            int message_count = 0;
            String pos;
            if(debug) Serial.println("GPS ON");
            digitalWrite(GPS_PIN,HIGH);
            Wait(1,false);
            while(message_count<5000){
              while(Serial1.available()){
                int GPS_info_char = Serial1.read();
                if(GPS_info_char=='$')message_count ++;//Start of message counting message
                if(debug){
                  if(GPS_info_char=='$'){//start of message
                    message_started = true;
                    received_char = 0;
                    }else if(GPS_info_char=='*'){//end of message
                      for(i=0;i<received_char;i++){
                        Serial.write(GPS_info_buffer[i]);
                        }
                        Serial.println();
                        message_started = false;
                      }else if(message_started ==true){
                        if(received_char <=GPS_INFO_BUFFER_SIZE){
                          GPS_info_buffer[received_char] = GPS_info_char;
                          received_char++;
                          }else{
                            message_started = false;
                            received_char = 0;
                            }
                        }
                  }
                  if(gps.encode(GPS_info_char)){
                    gps.f_get_position(&latitude,&longitude);
                    altitude = gps.altitude()/100;
                    //Store coordinates in dedicated structure
                    gpscoordinates coordinates = {altitude,longitude,latitude};
                    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);

                    if (debug) {
                      Serial.println();
                      Serial.println();
                      Serial.print("Latitud/Longitud: ");
                      Serial.print(latitude, 5);
                      Serial.print(", ");
                      Serial.println(longitude, 5);
                      Serial.println();
                      Serial.print("Fecha: "); Serial.print(day, DEC); Serial.print("/");
                      Serial.print(month, DEC); Serial.print("/"); Serial.print(year);
                      Serial.print(" Hora: "); Serial.print(hour, DEC); Serial.print(":");
                      Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC);
                      Serial.print("."); Serial.println(hundredths, DEC);
                      Serial.print("Altitud (metros): "); Serial.println(gps.f_altitude());
                      Serial.print("Rumbo (grados): "); Serial.println(gps.f_course());
                      Serial.print("Velocidad(kmph): "); Serial.println(gps.f_speed_kmph());
                      Serial.print("Satelites: "); Serial.println(gps.satellites());
                      Serial.println();
                    }
                    gps.stats(&chars,&sentences,&failed_checksum);
                    if(debug)Serial.println("GPS turned off");
                    digitalWrite(GPS_PIN,LOW);//GPS turned off
                    pos = ConvertGPSdata(&coordinates,sizeof(gpscoordinates));//send data
                    return pos;
                   }
                }
              }
              pos = "No Signal";
            }

void setup() {
  // put your setup code here, to run once:
  if(debug){
    Serial.begin(9600);
    while(!Serial1){
      Serial.println("Waiting for serial to connect");
      }
      Serial.println("Serial connected");
    }
    //Serial pins 13 -14 for 3.3V connection to GPS
    Serial1.begin(9600);
    while(!Serial1){
      Serial.println("Waiting for GPS to connect");
      }
    if(debug){
      Serial.println("GPS connected");
      }
      pinMode(GPS_PIN,OUTPUT);//interupt pin to gps
      if(!SigFox.begin()){
        Serial.println("Sheild error or not present");
        return;
        }
        //Enable debug led and disable automatic deep sleep
        if(debug){
          SigFox.debug();
          }else{
            SigFox.end();
            }

}

void loop() {
  // put your main code here, to run repeatedly:

  String position_data;
  position_data = GetGPSposition();
  SendSigFox(position_data);
  Wait(WAITING_TIME,false);

}
