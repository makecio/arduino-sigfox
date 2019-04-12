// Esse código faz o arduino acordar a cada minuto (aproximadamente) e grava a tensão na porta A0 no cartão SD
//
//  Ideia: Ao acordar, poderia passar 1 minuto ou 30 segundos gravando informações e depois voltaria a dormir. Que tal?
//  Dessa maneira eu conseguiria pegar um valor médio e mais confiável

#include <Wire.h>    
#include <RTClibExtended.h>
#include <LowPower.h>
#include <Nanofox.h>   //Nanofox Library
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include "DHT.h"

#define wakePin 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define ledPin 13    //use arduino on-board led for indicating sleep or wakeup status

#define DHTPIN A1 // pino que estamos conectado
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

/*------Global Variables---------------------------------------------------------------------------*/
uint16_t Counter_Sig = 0;        //Counter for testing 
unsigned long timeref_ms;        //Reference time for delay calculations

uint8_t Downlink_Buffer[8];   //Buffer for Downlink Payload

/*------Objects -----------------------------------------------------------------------------------*/
Nanofox MyNanofox;    //Nanofox Object Instance
SoftwareSerial serial_connection(10, 11); //TX=pin 10, RX=pin 11
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

RTC_DS3231 RTC;      //we are using the DS3231 RTC
DateTime a;
DateTime dt;

void printDigits(int digits);
void digitalClockDisplay();

byte AlarmFlag = 0;
byte ledStatus = 1;
int cont, segundo =0;
int minuto= 0;
int hora= 0;

float _lat = 0;
float _lng = 0;
float _temp = 0; 
float h ;
float t;
  
uint32_t *Pointer_32; //32 bits - Pointer
uint8_t Uplink_Buffer[12]; 
//-------------------------------------------------

void wakeUp()        // here the interrupt is handled after wakeup
{
}

//------------------------------------------------------------

void setup() {

   
  Serial.begin(9600);   //Initi Debug serial port

  MyNanofox.Init_ArduinoNano_IO();   //Setup arduino Nano IO
  MyNanofox.Init_Modem_WISOL(RC2);  //Initialize WISOL Sigfox Modem
  
  Serial.println("Bem-vindo ao dispositivo IOT - ANGULUS!");
  
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231/////////
  pinMode(wakePin, INPUT);

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);

  delay(1000);

  //Initialize communication with the clock
  Wire.begin();
  RTC.begin();
  dt = DateTime(__DATE__, __TIME__);
  RTC.adjust(dt);

  //MyNanofox.Print_PAC_WISOL();  //Prints Modem PAC Number in serial Debug
  //MyNanofox.Print_DEVICE_ID_WISOL();  //Prints Modem Device ID in serial Debug
  
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started
  
  //clear any pending alarms
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  
  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
  //The output of the DS3231 INT pin is connected to this pin
  //It must be connected to arduino D2 pin for wake-up
  RTC.writeSqwPinMode(DS3231_OFF);

  //Set alarm1 every day at (hora:minuto:segundo)
  RTC.setAlarm(ALM1_MATCH_HOURS, dt.minute()+2, dt.hour(), dt.second());   //set your wake-up time here
  RTC.alarmInterrupt(1, true);

  dht.begin();
  
    Serial.println("Aguardando GPS...");
}

void loop() 
{
  // Wait a few seconds between measurements.
  delay(10);

   if (AlarmFlag < 99) {
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
       h = dht.readHumidity();
      // Read temperature as Celsius (the default)
       t = dht.readTemperature();
        
      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t)) {
        AlarmFlag = AlarmFlag +1;
        Serial.println(F("Failed to read from DHT sensor!"));
      }else{
        Serial.print(F("Humidity: "));
        Serial.print(h);
        Serial.print(F("%  Temperature: "));
        Serial.print(t);
        Serial.println(F("°C "));
        AlarmFlag = 99;
      }
   }
  
  //On first loop we enter the sleep mode
  if (AlarmFlag >= 99) {

   
    while(serial_connection.available())//While there are characters to come from the GPS
    {
      gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
    }
    if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
    {

      
  
      //Get the latest info from the gps object which it derived from the data sent by the GPS unit
      Serial.println("Satellite Count:");
      Serial.println(gps.satellites.value());
      Serial.println("Altitude:");
      Serial.println(gps.altitude.meters(), 6);
      Serial.println("Latitude:");
      Serial.println(gps.location.lat(), 6 );
      Serial.println("Longitude:");
      Serial.println(gps.location.lng(), 6);


      const float scalingFactor = 1000000.0;
      long int LatInt = 0;
      _lat = gps.location.lat();
      
      if(_lat < 0){
        LatInt = (long int) ((_lat*-1) * scalingFactor);
      }else{
        LatInt = (long int) (_lat * scalingFactor);
      }

      long int LngInt = 0;
      _lng = gps.location.lng();
      
      if(_lng < 0){
        LngInt = (long int) ((_lng*-1) * scalingFactor);
      }else{
        LngInt = (long int) (_lng * scalingFactor);
      }
           
      String latString = "";
      int indexLat = String(_lat).indexOf(".");
       if(indexLat == -1){
        indexLat = 0;
      }
      
      if(_lat < 0){
        latString = "1"+String(indexLat)+String(LatInt);
      }else{
        latString = "2"+String(indexLat)+String(LatInt);
      }
      Serial.println("latString: "+latString);
       
      String lngString = "";
      int indexLng = String(_lng).indexOf(".");
      if(indexLng == -1){
        indexLng = 0;
      }
      
      if(_lng < 0){
        lngString = "1"+String(indexLng)+String(LngInt);
      }else{
        lngString = "2"+String(indexLng)+String(LngInt);
      }
      Serial.println("lngString: "+lngString);
      
      int i=0;
      int j=0;
      while ( i < latString.length()){
        Uplink_Buffer[j] = latString.substring(i, i+2).toInt();
        
        Serial.println("BUFFER "+latString.substring(i, i+2));
        i=i+2;
        j=j+1;
      }

      i=0;
      while ( i < lngString.length()){
        Uplink_Buffer[j] = lngString.substring(i, i+2).toInt();

        Serial.println("BUFFER "+lngString.substring(i, i+2));
        i=i+2;
        j=j+1;
      }

      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
      }else{
       
        const float scalingFactorTemp = 100.0;
        long int TempInt = 0;
        _temp = t;
      
        if(_temp < 0){
          TempInt = (long int) ((_temp*-1) * scalingFactorTemp);
        }else{
          TempInt = (long int) (_temp * scalingFactorTemp);
        }

         String tempString = "";
        
        if(_temp < 0){
          tempString = String(TempInt);
        }else{
          tempString = String(TempInt);
        }
        Serial.println("tempString: "+tempString);

         i=0;
        while ( i < tempString.length()){
          Uplink_Buffer[j] = tempString.substring(i, i+2).toInt();
  
          Serial.println("BUFFER "+tempString.substring(i, i+2));
          i=i+2;
          j=j+1;
        }
      }
      
      MyNanofox.Send_Payload_Sigfox(&Uplink_Buffer[0], sizeof(Uplink_Buffer), &Downlink_Buffer[0],0);
      
      Serial.println("DORMINDO...");
      
      serial_connection.end();//This close up communications to the GPS
      
      MyNanofox.Set_Powermode_Modem_WISOL(2);
      
      digitalWrite(ledPin, LOW);
      delay(1000);
      attachInterrupt(0, wakeUp, LOW);                       //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW 
      digitalWrite(ledPin, LOW);                             //switch-off the led for indicating that we enter the sleep mode
      ledStatus = 0;                                         //set the led status accordingly
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
      detachInterrupt(0);                                    //execution resumes from here after wake-up
  
      Serial.println("ACORDANDO....");
      MyNanofox.Wakeup_From_Deep_Sleep_Modem_WISOL();

      serial_connection.begin(9600);//This opens up communications to the GPS
      Serial.println("GPS Start");//Just show to the monitor that the sketch has started
  
      //When exiting the sleep mode we clear the alarm
      RTC.armAlarm(1, false);
      RTC.clearAlarm(1);
      RTC.alarmInterrupt(1, false);

      RTC.writeSqwPinMode(DS3231_OFF);
      dt = DateTime(__DATE__, __TIME__);
      RTC.adjust(dt);
      RTC.setAlarm(ALM1_MATCH_HOURS, dt.minute()+2, dt.hour(), dt.second());   //set your wake-up time here
      RTC.alarmInterrupt(1, true);
      AlarmFlag=0;
      delay(1000);
      
    }
  
    
  }


 
}
