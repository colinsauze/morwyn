/*
 This file is part of Morwyn's control system.  Morwyn's control system is free software: you can
 redistribute it and/or modify it under the terms of the GNU General Public
 License as published by the Free Software Foundation, version 2.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 details.

 You should have received a copy of the GNU General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 51
 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

 Copyright Colin Sauze
*/

#include <Wire.h>
#include <OneWire.h>
#include <stdint.h>
#include <avr/sleep.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <IridiumSBD.h>

byte ledState=0;
volatile unsigned long counter;
volatile int msgCount=0;
volatile int motorCurrent, motorVoltage, controlCurrent, controlVoltage;
SoftwareSerial myDebug(7, 8);
SoftwareSerial openLog(6, 5);
long good = 0;
int SBDIstatus[6];

const unsigned long WAIT_TIME = 500;

#define IRIDIUM_PWR 12

IridiumSBD isbd(Serial1, IRIDIUM_PWR);
static const int ledPin = 7;

#define HEADING 0
#define WIND_DIR 1
#define ROLL 2
#define PITCH 3
#define RUDDER 4
#define SAIL 5
#define LAT 6
#define LON 7
#define TIME 8

#define DEBUG_CRITICAL 1 //really important messages that we don't want to ignore and are prepared to sacrifice execution speed to see
#define DEBUG_IMPORTANT 2 //fairly important messages that we probably want to see, but might cause issues with execution speed
#define DEBUG_MINOR 3 //less important messages that we can safely turn off to improve execution speed

#define DEBUG_THRESHOLD DEBUG_IMPORTANT //set to 0 to show no debugging messages


union BucketData {
  struct Data{ 
    byte heading_bucket[10]; //10 degree intervals, 0-359, 10 bytes 
    byte wind_bucket[10]; //10 degree intervals, 0-359, 10bytes , 20 total
    byte roll_bucket[14]; //10 degree intervals, <-60, -60 to +60 and >60, 14 bytes, 34 total
    byte pitch_bucket[14]; //10 degree intervals, <-60, -60 to +60 and >60, 14 bytes, 48 total
    byte motor_batt_current[16]; //0.2 amp intervals, 0-3 amps and over 3 amps, multiply by 10. 16 bytes, 64 total
    byte motor_batt_voltage[17]; //0.2 volt intervals, <6, 6-9 and >9 volts, multiply by 10. 17 bytes, 81 total
    byte comp_batt_current[6]; //0.2 amp intervals, 0-1 amps and over 1 amps, multiply by 10. 6 bytes, 87 total
    byte comp_batt_voltage[10]; //0.2 volt intervals, <3, 3-4.6 and >4.6 volts, multiply by 10. 10 bytes, 97 total
    byte rudder_position[11]; //-5 to +5 ,  11 bytes, 108 total
    byte sail_position[6]; //0 to 5, 6 bytes, 114 total
    float lat,lon; //current lat/lon, 8 bytes, 122 total
    int temp1,temp2; //temperature readings, 4 bytes, 126 total
    unsigned long unixtime;
    byte checksum;
  }
  data;

  byte buffer[sizeof(struct Data)];

}
transmit;


static FILE uartout = {
  0} 
;

//makes printf work
static int uart_putchar (char c, FILE *stream)
{
  myDebug.write(c) ;
  return 0 ;
}

//debugging code to print messages at different debug levels with different prefixes
static void say(byte level, char* msg)
{
  if(level<=DEBUG_THRESHOLD)
  {
    myDebug.print("Debug");
    myDebug.print(level);
    myDebug.print(": [Comms] ");
    myDebug.println(msg);
  }
}

//debugging printf that prepends "Debug:" to everything and can be easily turned off
void dprintf(byte level, const char *fmt, ...)
{      
  
  if(level<=DEBUG_THRESHOLD)
  {
    printf("Debug%d: [Comms] ",level);
    va_list ap;
    va_start(ap, fmt);
    
    vprintf(fmt, ap);
    va_end(ap);
  }
}

//flashing led callback for Iridium modem
bool ISBDCallback()
{
   digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
   return true;
}

//addresses of the temperature sensors
byte addr1[8] = { 
  0x10, 0xA, 0x3E, 0x82, 0x2, 0x8, 0x0, 0xF5};
byte addr2[8] = { 
  0x10, 0xFD, 0x4F, 0x82, 0x2, 0x8, 0x0, 0xA2 };

//computer NMEA style checksums
byte compute_checksum(byte *data,byte length)
{                
  byte computed_checksum=0;

  for (byte i = 0; i < length; i++)
  {
    computed_checksum = (byte)computed_checksum ^ data[i];
  }

  return computed_checksum;
}

/* computes the checksum of length-1 bytes in data and compares this with the last byte 
 * returns 0 if checksums aren't the same, 1 if they are
 */
byte verify_checksum(byte *data,byte length)
{
  byte computed_checksum = compute_checksum(data,length-1);
  if(computed_checksum==data[length-1])
  {
    return 1;
  }
  ////printf("c = %d r = %d\n",computed_checksum,data[length-1]);
  return 0;   
}

void setup()
{ 

  Serial.begin(115200);          //USB serial, debugging only
  Serial1.begin(19200);          //modem serial

  myDebug.begin(115200);        //debug serial
  openLog.begin(9600);         

  say(DEBUG_CRITICAL,"Starting up, waiting 30 seconds to allow reprogramming time");
  openLog.println("Starting up");

  delay(30000); // wait long enough to allow reprogramming without sleeping interfering with bootload process

  //let us have //printf
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout ;
  dprintf(DEBUG_CRITICAL,"Printf Setup\r\n");


  pinMode(7, OUTPUT);  //LED pins
  pinMode(9, OUTPUT);  
  pinMode(IRIDIUM_PWR, OUTPUT); //Iridium power
  digitalWrite (7, HIGH);
  digitalWrite (7, LOW);
  digitalWrite (IRIDIUM_PWR, LOW);

  Wire.begin(4);                // join i2c bus with address #4

  Wire.onReceive(receiveEvent); // register event

  #if DEBUG_THRESHOLD >= DEBUG_MINOR
  isbd.attachConsole(myDebug);
  #endif
  isbd.setPowerProfile(1);
  
}

ISR( WDT_vect ) {
  cli();
  wdt_reset();
  sei();
  say(DEBUG_IMPORTANT,"Watchdog reset");
}

void loop()
{

 byte old_ADCSRA = ADCSRA;
  //disable ADC to save power
  goto DUMMY;
  
  ADCSRA = 0;  

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);             // select the watchdog timer mode
  MCUSR &= ~(1 << WDRF);                           // reset status flag
  WDTCSR |= (1 << WDCE) | (1 << WDE);              // enable configuration changes
  WDTCSR = (1<< WDP0) | (1 << WDP1) | (1 << WDP2) | (1 << WDP3); // set the prescalar = 7
  WDTCSR |= (1 << WDIE);                           // enable interrupt mode
  cli();
  sleep_enable();
  //    sleepx_bod_disable();
  sei();
  sleep_cpu();/* ...time passes ... */
  sleep_disable();                                 // prevent further sleeps

DUMMY:

  delay (16000);
  say(DEBUG_IMPORTANT,"Woke");
  dprintf(DEBUG_IMPORTANT,"msgCount=%d\r\n",msgCount);
  delay(100);
  ADCSRA = old_ADCSRA; //enable ADC for use

  if(msgCount>=10) //once we've got 10 messages ready send the data to the modem
  {
    //digitalWrite (12, HIGH); // Turn Iridium module on
    transmit.data.checksum = compute_checksum(transmit.buffer,sizeof(transmit.buffer)-1);
    
    dprintf(DEBUG_IMPORTANT,"about to read temperature\r\n");
    transmit.data.temp1 = getTemperature(addr1);
    //transmit.data.temp2 = getTemperature(addr2);

    dprintf(DEBUG_IMPORTANT,"temp1 = %d temp2 = %d\n",transmit.data.temp1,transmit.data.temp2);
    openLog.print("Temp1 = ");
    openLog.println(transmit.data.temp1);

    dprintf(DEBUG_IMPORTANT,"About to transmit millis=%ld, bucket summary:\n",millis());
    decodeBuckets();
    msgCount=0;

    dprintf(DEBUG_MINOR,"Calling sendIridiumMessage()\r\n");
    openLog.print("Sending Iridium message at time");
    openLog.println(transmit.data.unixtime);
    
    sendIridiumMessage();
    
    resetBuckets(); 
   }

}

void sendIridiumMessage()
{
      /*say(DEBUG_MINOR,"Raw message data:");
    
    for(int i=0;i<sizeof(transmit.buffer);i++)
    {
      dprintf(DEBUG_MINOR,"%c",transmit.buffer[i]);
    }*/
    //printf("\nEnd Raw\n");

    /*composeBinaryMessage(transmit.buffer,sizeof(transmit.buffer));     

    say(DEBUG_IMPORTANT,"Composed binary message");

     radioOn();
     noCIER();
     
     say(DEBUG_IMPORTANT,"Radio on...");
     
     delay(10000);
     
     while(Serial1.available())
       Serial1.read();
     
     sendReceiveMessage();
     
     say(DEBUG_IMPORTANT,"SBDI status returned:");
     
     for (int i = 0; i < 6; i++)
     {
      myDebug.println(SBDIstatus[i]);
     }
     
     radioOff();
     digitalWrite (12, LOW); // Turn Iridium module off
     */
      int signalQuality;
      isbd.begin();

      int err = isbd.getSignalQuality(signalQuality);
      if (err != 0)
      {
        dprintf(DEBUG_IMPORTANT,"SignalQuality failed: error %d\r\n",err);
        return;
      }

      dprintf(DEBUG_IMPORTANT,"Signal quality is %d",signalQuality);

      err = isbd.sendSBDBinary(transmit.buffer,sizeof(transmit.buffer));
      if (err != 0)
      {
        dprintf(DEBUG_IMPORTANT,"sendSBDText failed: error %d\r\n",err);
        return;
      }
      isbd.sleep();
     

  
}

void resetBuckets()
{
  memset(transmit.buffer,0,sizeof(transmit));
}

//reads currents and voltages
void readAttoPilot()
{
  motorCurrent = analogRead(A1)*44;  //0-1024, 1024 = 45A, expected range 0-3 amps, 3A = 68.2666, 68*44 = 2992, so multiply by 44 to get integer milliamps 
  motorVoltage = analogRead(A0)*13; //0-1024, 1024 = 13.6V, multiply by 13 to get integer millivolts 
  controlCurrent = analogRead(A3)*44; //expected range 0-1 amps, multiply by 44 to get integer milliamps
  controlVoltage = analogRead(A2)*13; //expected range of 3 to 4.6 V

  dprintf(DEBUG_MINOR,"attopilot read, motor = %d mV, %d mA computer = %d mV, %d mA\r\n",motorVoltage,motorCurrent,controlVoltage,controlCurrent);

  if(motorCurrent>3000)
  {
    transmit.data.motor_batt_current[15]++;
  }
  else
  {
    transmit.data.motor_batt_current[motorCurrent/200]++; //should give 15 buckets of 200 milliamps each, 2999/200 = 14.995
  }


  if(motorVoltage<6000)
  {
    transmit.data.motor_batt_voltage[0]++;
  }
  else if(motorVoltage>=9000)
  {
    transmit.data.motor_batt_voltage[16]++; 
  }
  else 
  {
    transmit.data.motor_batt_voltage[(motorVoltage/200)-29]++; //cover buckets 1-15 for 6000 to 8999 mV
  }




  if(controlCurrent>1000)
  {
    transmit.data.comp_batt_current[5]++;
  }
  else
  {
    transmit.data.comp_batt_current[controlCurrent/200]++; //should give 5 buckets of 200 milliamps each, 999/200 = 4.995
  }


  if(controlVoltage<3000)
  {
    transmit.data.comp_batt_voltage[0]++;
  }
  else if(controlVoltage>=4600)
  {
    transmit.data.comp_batt_voltage[9]++; 
  }
  else 
  {
    transmit.data.comp_batt_voltage[(controlVoltage/200)-14]++; //cover buckets 1 to 8 for  3000 to 4599 mV
  }



}

//reads a DS1820 temp sensor
int getTemperature(byte *addr)
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];

  float celcius=0;

  OneWire  ds(13); 

  switch (addr[0]) {
  case 0x10:
    //  Serial.println("  Chip = DS18S20");  // or old DS1820
    type_s = 1;
    break;
  case 0x28:
    // Serial.println("  Chip = DS18B20");
    type_s = 0;
    break;
  case 0x22:
    // Serial.println("  Chip = DS1822");
    type_s = 0;
    break;
  default:
    say(DEBUG_CRITICAL,"Device is not a DS18x20 family device.");
    return -1;
  }


  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();   
  }


  if(OneWire::crc8(data, 8) != data[8])
  {
    dprintf(DEBUG_CRITICAL,"CRC Failed, got %X expected %X\r\n",OneWire::crc8(data, 8),data[8]);
    return -9999;
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } 
  else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celcius = (float)raw / 16.0;
  //fahrenheit = celsius * 1.8 + 32.0;

  return (int)(celcius*100);
}

//show us what's in the buckets data structure
void decodeBuckets()
{
  int i;

  if(DEBUG_THRESHOLD>=DEBUG_IMPORTANT)
  {
    dprintf(DEBUG_IMPORTANT,"\r\nBucket summary:\r\n");

    printf("Heading:\r\n");
    for(int i=0;i<10;i++)
    {
      printf("%d-%d: %d\r\n",i*36,((i+1)*36)-1,transmit.data.heading_bucket[i]);
    }
    delay(50);

    printf("Wind:\r\n");
    for(int i=0;i<10;i++)
    {
      printf("%d-%d: %d\r\n",i*36,((i+1)*36)-1,transmit.data.wind_bucket[i]);
    }
    delay(50);

    printf("Roll:\r\n");
    printf("<-60: %d\r\n",transmit.data.roll_bucket[0]);

    for(int i=1;i<13;i++)
    {
      printf("%d to %d: %d\r\n",((i-1)*10)-60,((i-1)*10)-51,transmit.data.roll_bucket[i]);
    }
    printf(">=60: %d\r\n",transmit.data.roll_bucket[13]);
    delay(50);


    printf("Pitch:\r\n");
    printf("<-60: %d\r\n",transmit.data.pitch_bucket[0]);

    for(int i=1;i<13;i++)
    {
      printf("%d to %d: %d\r\n",((i-1)*10)-60,((i-1)*10)-51,transmit.data.pitch_bucket[i]);
    }
    printf(">=60: %d\r\n",transmit.data.pitch_bucket[13]);


    delay(50);


    printf("Sail Position\r\n");
    for(int i=0;i<6;i++)
    {
      printf("%d: %d\r\n",i,transmit.data.sail_position[i]);
    }

    delay(50);

    printf("Rudder Position\r\n");
    for(int i=0;i<11;i++)
    {
      printf("%d: %d\r\n",i-5,transmit.data.rudder_position[i]);
    }

    delay(50);


    /*  byte motor_batt_current[16]; //0.2 amp intervals, 0-3 amps and over 3 amps, multiply by 10. 64
    byte motor_batt_voltage[17]; //0.2 volt intervals, <6, 6-9 and >9 volts, multiply by 10. 81
    byte comp_batt_current[6]; //0.2 amp intervals, 0-1 amps and over 1 amps, multiply by 10. 87
    byte comp_batt_voltage[10]; //0.2 volt intervals, <3, 3-4.6 and >4.6 volts, multiply by 10. 97*/


    printf("Motor Battery Voltage\r\n");

    printf("Under 6V: %d\r\n", transmit.data.motor_batt_voltage[0]);

    for(int i=1;i<16;i++)
    {
      printf("%d mV to %d mV: %d\r\n",(i*200)+6000,((i+1)*200)+5999,transmit.data.motor_batt_voltage[i]);
    }

    printf("Over 9V: %d\r\n",transmit.data.motor_batt_voltage[16]);

    delay(50);


    printf("Motor Battery Current\r\n");


    for(int i=0;i<15;i++)
    {
      printf("%d mA to %d mA: %d\r\n",(i*200),((i+1)*200)-1,transmit.data.motor_batt_current[i]);
    }

    printf("Over 3000 mA: %d\r\n",transmit.data.motor_batt_current[15]);

    delay(50);   

    printf("Computer Battery Voltage\r\n");

    printf("Under 3V: %d\r\n", transmit.data.comp_batt_voltage[0]);

    for(int i=1;i<9;i++)
    {
      printf("%d mV to %d mV: %d\r\n",(i*200)+3000,((i+1)*200)+2999,transmit.data.comp_batt_voltage[i]);
    }

    printf("Over 4.6V: %d\r\n",transmit.data.comp_batt_voltage[9]);

    delay(50);


    printf("Computer Battery Current\r\n");


    for(int i=0;i<6;i++)
    {
      printf("%d mA to %d mA: %d\r\n",(i*200),((i+1)*200)-1,transmit.data.motor_batt_current[i]);
    }

    printf("Over 3000 mA: %d\r\n",transmit.data.motor_batt_current[6]);

    dprintf(DEBUG_IMPORTANT,"time=%ld lat=%d lon=%d temp1=%d temp2=%d\r\n",transmit.data.unixtime,(int)(transmit.data.lat*100),(int)(transmit.data.lon*100),transmit.data.temp1,transmit.data.temp2);

    printf("Checksum = %d\r\n",transmit.data.checksum);
  }
}





// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  //just here for building up the debug string
  static uint16_t heading;
  static uint16_t wind_dir;
  static int8_t roll;
  static int8_t pitch;
  static int8_t rudder;
  static uint8_t sail;

  byte buf[10];
  #define PRINTBUF_SIZE 80
  char printbuf[PRINTBUF_SIZE];

  byte command;

  sleep_disable(); // in case about to sleep

  good++;

  if(Wire.available()>0)
  {

    if(ledState==1)
    {
      digitalWrite(9,HIGH);  
      ledState=0;
    }
    else
    {
      digitalWrite(9,LOW);  
      ledState=1;
    }

    command=Wire.read();

    dprintf(DEBUG_MINOR,"Got i2c message %d\r\n",command);

    switch(command){
    case HEADING:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();
        buf[2]=Wire.read();

        if(verify_checksum(buf,3))
        {
          heading=(uint16_t)buf[0];
          heading=heading+(((uint16_t)buf[1])*255);
          if(heading>=0&&heading<=359)
          {    
            transmit.data.heading_bucket[heading/36]++;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on heading");
          good = 0;
        }
      }
      break;

    case WIND_DIR:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();
        buf[2]=Wire.read();

        if(verify_checksum(buf,3))
        {
          wind_dir=(uint16_t)buf[0];
          wind_dir=wind_dir+(((uint16_t)buf[1])*255);
          if(wind_dir>=0&&wind_dir<=359)
          {    
            transmit.data.heading_bucket[wind_dir/36]++;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on wind");
          good = 0;        
        }
      }
      break;

    case ROLL:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();  

        if(verify_checksum(buf,2))
        {
          roll=(int8_t)buf[0]; //this limits us to -127 to +127 range, so its self limiting within safe ranges
          if(roll<-60)
          {
            transmit.data.roll_bucket[0]++;
          }
          else if(roll>=60)
          {
            transmit.data.roll_bucket[13]++;
          }
          else
          {
            transmit.data.roll_bucket[((roll+60)/10)+1]++;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on roll");
          good = 0;        
        }
      }
      break;

    case PITCH:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();  

        if(verify_checksum(buf,2))
        {
          pitch=(int8_t)buf[0]; //this limits us to -127 to +127 range, so its self limiting within safe ranges
          if(pitch<-60)
          {
            transmit.data.pitch_bucket[0]++;
          }
          else if(pitch>=60)
          {
            transmit.data.pitch_bucket[13]++;
          }
          else
          {
            transmit.data.pitch_bucket[((pitch+60)/10)+1]++;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on pitch");
          good = 0;      
        }
      }
      break;          

    case RUDDER:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();    

        if(verify_checksum(buf,2))
        {
          rudder = (int8_t)buf[0];    
          if(rudder>=-5&&rudder<=5)
          {
            transmit.data.rudder_position[rudder+5]++;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on rudder");
          good = 0;      
        }   
      }
      break;

    case SAIL:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();      

        if(verify_checksum(buf,2))
        {
          sail = buf[0];  
          if(sail>=0&&sail<=5)
          {
            transmit.data.sail_position[sail]++;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on sail");
          good = 0;      
        }
      }
      break;

    case LAT:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();
        buf[2]=Wire.read();      
        buf[3]=Wire.read();      
        buf[4]=Wire.read();      

        if(verify_checksum(buf,5))
        {

          union {
            byte buf[4];
            float lat;
          } 
          latU;

          latU.buf[0]=buf[0];
          latU.buf[1]=buf[1];
          latU.buf[2]=buf[2];
          latU.buf[3]=buf[3];
          
          //only store valid readings
          if(latU.lat>=-90&&latU.lat<=90)
          {
            transmit.data.lat = latU.lat;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on lat");
	  good = 0;      
        }

      }
      break;

    case LON:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();
        buf[2]=Wire.read();      
        buf[3]=Wire.read();      
        buf[4]=Wire.read();      

        if(verify_checksum(buf,5))
        { 
          union {
            byte buf[4];
            float lon;
          } 
          lonU;

          lonU.buf[0]=buf[0];
          lonU.buf[1]=buf[1];
          lonU.buf[2]=buf[2];
          lonU.buf[3]=buf[3];
          
          
          //only store valid readings
          if(lonU.lon>=-180&&lonU.lon<=180)
          {
            transmit.data.lon = lonU.lon;
          }
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on lon");
          good = 0;      
        }
      }
      break;

    case TIME:
      {
        buf[0]=Wire.read();
        buf[1]=Wire.read();
        buf[2]=Wire.read(); 
        buf[3]=Wire.read(); 
        buf[4]=Wire.read(); 
        
        if(verify_checksum(buf,5))
        {
            union {
                byte buf[4];
                unsigned long unixtime;
            } 
            timeU;
            
            timeU.buf[0]=buf[0];
            timeU.buf[1]=buf[1];
            timeU.buf[2]=buf[2];
            timeU.buf[3]=buf[3];
            transmit.data.unixtime=timeU.unixtime;
        }
        else
        {
          say(DEBUG_IMPORTANT,"checksum failed on time");
          good = 0;      
        }
        readAttoPilot(); 
        
        snprintf(printbuf,PRINTBUF_SIZE,"time=%ld hdg=%d wind=%d roll=%d pitch=%d sail=%d rudder=%d cv=%d mv=%d ci=%d mi=%d",transmit.data.unixtime,heading,wind_dir,roll,pitch,sail,rudder,controlVoltage,motorVoltage,controlCurrent,motorCurrent);
                
        dprintf(DEBUG_MINOR,"good=%d msgCount=%d\r\n",good,msgCount);
        dprintf(DEBUG_MINOR,"%s",printbuf);
        openLog.print(printbuf);
        openLog.print(" lat=");
        openLog.print(transmit.data.lat,5);           
        openLog.print(" lon=");
        openLog.println(transmit.data.lon,5);           
        
        #if DEBUG_THRESHOLD >= DEBUG_MINOR
        printf(" lat=");
        myDebug.print(transmit.data.lat,5);           
        #endif
        #if DEBUG_THRESHOLD >= DEBUG_MINOR
        printf(" lon=");
        myDebug.println(transmit.data.lon,5);           
        #endif
        
        
        
      }      
      break;

    }      
    delay(100);    
    msgCount++;

  }
}

#ifdef MARK_IRIDIUM
//mark's iridium stuff, ignore if using IridiumSBD library (recommended)
int getSignalQuality()
{
  int buffpos; 
  char buff[16];
  int c;
  int sigquality;
  String value = "";

  for (int i = 0; i < 16; i++)
    buff[i] = 0;

  Serial1.println("AT+CSQ");

  while (!Serial1.available()); // throw away echo

   /* for (int i = 0; i < 10; i++)
   {
   while(!Serial1.available());
   Serial.write(Serial1.read());
   }*/
  while (!Serial1.available()); // wait for response
  for (int i = 0; i < 14; i++)
  {
    while(!Serial1.available());
    c = Serial1.read();
    buff[buffpos++] = c;       
  }
  //  Serial.println(buff);
  // Serial.println(buff[5]);
  sigquality = atoi(&buff[5]);

  return sigquality;
}

void composeBinaryMessage(byte *message,byte length)
{
  int buffpos; 
  int i = 0;
  char c;
  dprintf(DEBUG_IMPORTANT,">> AT+SBDWD=%d\r\n",length);
  
  Serial1.print("AT+SBDWB=");
  Serial1.println(length);
  while (!Serial1.available()); // throw away response

  while (Serial1.available())
  {
    
    //Serial.write(Serial1.read());
    c=Serial1.read();
    myDebug.print(c);
  }

  for (i = 0; i < length; i++)
  {
    Serial1.write(message[i]);
    while (!Serial1.available());
    //Serial.write(Serial1.read());
    Serial1.read();
  }

  Serial1.println("");

  while (!Serial1.available()); // throw away response
  //  delay(2000);
  //for (i = 0; i < 1024; i++)
  while (Serial1.available())
  {
    //Serial.write(Serial1.read());
    Serial1.read();
  }
  //while (!Serial1.available()); // throw away response
  //while(1)
  //while (Serial1.available())
  //Serial.write(Serial1.read());

  //Serial.println(buff);

}


void radioOn()
{

  Serial1.println("AT*R1");

  while (!Serial1.available()); // throw away response
  while (Serial1.available())
  {
    //Serial.write(Serial1.read());
    Serial1.read();
  }

}

void radioOff()
{

  Serial1.println("AT*R0");

  while (!Serial1.available()); // throw away response
  while (Serial1.available())
  {
    //Serial.write(Serial1.read());
    Serial1.read();
  }

}

void noCIER()
{
  Serial1.println("AT+CIER=0");

  while (!Serial1.available()); // throw away response
  while (Serial1.available())
  {
    //Serial.write(Serial1.read());
    Serial1.read();
  }

}

void sendReceiveMessage()
{
  int buffpos = 0; 
  char buff[64];
  int c = ' ';
  String value = "";
  int i = 0;

  Serial1.println("AT+SBDI");

  while (!Serial1.available()); // throw away echo

  for (int i = 0; i < 9; i++)
  {
    while(!Serial1.available());
    //Serial.write(Serial1.read());
  }
  while (!Serial1.available()); // throw away echo

  for (int i = 0; i < 8; i++)
  {
    while(!Serial1.available());
    //Serial.write(Serial1.read());
  }

  while (!Serial1.available()); // wait for response

    while (Serial1.available())   // print out response
  {
    while (!Serial1.available()); // wait for response
    c = Serial1.read(); 
    buff[buffpos++] = c; 
    if (isDigit(c))
    {
      value = value + (char) c; 
    }
    if (c == ',')
    {
      SBDIstatus[i++] = value.toInt();
      value = "";
    }

  }

  /*if (SBDIstatus[2] == 1)
   {
   Serial.println("Message Received from GSS");
   }*/

}
#endif

/*void setup()  
 {
 Serial.begin(9600);
 Serial.println("Hello");
 
 // set the data rate for the SoftwareSerial port
 Serial1.begin(4800);
 
 
 }
 
 void 9602test() // run over and over
 {
 
 
 char inChar = ' ';
 String inString;
 int value;
 String messtring = "Raw pressures for last hour: ";
 
 for (int i = 0; i < 60; i ++){
 while(!Serial.available());
 while (inChar!='\n') {
 inChar = Serial.read();
 if (isDigit(inChar)) {
 // convert the i2c_data byte to a char 
 // and add it to the string:
 inString += (char)inChar; 
 }
 // if you get a newline, print the string,
 // then the string's value:
 if (inChar == '\n') {
 //Serial.print(" ");
 value = inString.toInt();
 //Serial.print(value);
 inString = ""; 
 }
 }
 inChar = ' ';
 messtring += value;
 messtring += " ";
 delay(59000);
 Serial.println(".");
 }
 
 messtring += "\r\n";
 
 Serial.println(messtring);
 
 delay(1000);
 
 //  composeMessage("Testing again: ignore. FUCKING MOSQUITOS.");
 composeMessage(messtring);
 Serial.println("Attempting to send message");
 delay(5000);  
 // while(sigquality < 2)
 // {
 // getSignalQuality();
 // Serial.print("Signal Quality: ");
 // Serial.println(sigquality);
 // }
 while(Serial1.available())
 Serial1.read();
 sendReceiveMessage();
 
 Serial.println("SBDI status returned:");
 
 for (int i = 0; i < 6; i++)
 {
 Serial.println(SBDIstatus[i]);
 }
 read:
 readMessage();
 
 }*/

