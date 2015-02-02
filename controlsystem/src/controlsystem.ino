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

#include <stdio.h>
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>
#include "Time.h"
#include "TinyGPS.h"

#define HMC6343_ADDRESS 0x19
#define HMC6343_HEADING_REG 0x50
#define WIND_SENSOR_PIN 4
#define GPS_ENABLE_PIN 12

#define GPS_READ_INTERVAL 15 //how many seconds to leave between GPS reads
#define WP_THRESHOLD 15 //how close (in metres) should we get before we change waypoint?

#define rad2deg(x) (180/M_PI) * x
#define deg2rad(x) x * M_PI/180

Servo rudderServo; // create servo object to control a servo
Servo sailServo; // a maximum of eight servo objects can be created 
SoftwareSerial myDebug(7, 8);

TinyGPS gps;

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

byte ledState=0;
union {
  struct Data{
    uint16_t heading;
    uint16_t wind_dir;
    int8_t roll;
    int8_t pitch;
    int8_t rudder;
    byte sail;
    float lat;
    float lon;
    long unixtime;
  } 
  data;
  byte buffer[sizeof(struct Data)];
} 
i2c_data;



//make printf work
static FILE uartout = {
  0} 
;

static int uart_putchar (char c, FILE *stream)
{
  myDebug.write(c) ;
  return 0 ;
}

static void say(byte level, char* msg)
{
  if(level<=DEBUG_THRESHOLD)
  {
    myDebug.print("Debug");
    myDebug.print(level);
    myDebug.print(": [Ctrl] ");
    myDebug.println(msg);
  }
}

//debugging printf that prepends "Debug:" to everything and can be easily turned off
void dprintf(byte level, const char *fmt, ...)
{      
  
  if(level<=DEBUG_THRESHOLD)
  {
    printf("Debug%d: [Ctrl] ",level);
    va_list ap;
    va_start(ap, fmt);
    
    vprintf(fmt, ap);
    va_end(ap);
  }
}

void dprintf2(const char *fmt, ...)
{      
  
    //printf("Debug%d: [Ctrl] ",level);
    va_list ap;
    va_start(ap, fmt);
    
    printf(fmt, ap);
    va_end(ap);
}


void setup() 
{
  Serial.begin(9600); //baud rate makes no difference on 32u4

  Serial1.begin(4800); //for GPS

  myDebug.begin(4800); //debug UART

  say(DEBUG_CRITICAL,"Control system start up");

  //required for printf  
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout ;
  dprintf(DEBUG_IMPORTANT,"Printf configured \r\n");

  delay(5000);

  say(DEBUG_IMPORTANT,"Setting up servos...");
  //Use .attach for setting up connection to the servo
  rudderServo.attach(5, 1060, 1920); // Attach, with the output limited
  // between 1000 and 2000 ms
  sailServo.attach(6, 1050, 1930); // Same, but between 1000 and 1850 ms
  rudderServo.writeMicroseconds(1500);
  sailServo.writeMicroseconds(1500);
  say(DEBUG_IMPORTANT,"Done");

  say(DEBUG_IMPORTANT,"Setting up wind sensor...");
  pinMode(WIND_SENSOR_PIN, INPUT); //wind sensor PWM input
  say(DEBUG_IMPORTANT,"Done");

  say(DEBUG_IMPORTANT,"Setting up I2C...");
  Wire.begin(); // Initialise i2c for compass and comms to other arduino (AS MASTER)
  say(DEBUG_IMPORTANT,"Done");

  say(DEBUG_IMPORTANT,"Setting up GPS...");
  pinMode(GPS_ENABLE_PIN, OUTPUT); //GPS on/off line
  //setup GPS
  digitalWrite(GPS_ENABLE_PIN,1);
  delay(1000);
  //turn off VTG
  Serial1.println("$PSRF103,05,00,00,01*21\r");
  //turn off RMC
  Serial1.println("$PSRF103,04,00,00,01*20\r");

  //turn off GSV
  Serial1.println("$PSRF103,03,00,00,01*27\r");

  //turn off GSA
  Serial1.println("$PSRF103,02,00,00,01*26\r");

  //turn off GLL
  Serial1.println("$PSRF103,01,00,00,01*25\r");

  //turn off GGA
  Serial1.println("$PSRF103,00,00,00,01*24\r");
  delay(1000);

  //leave GPS on to get its initial fix
  //digitalWrite(GPS_ENABLE_PIN,0);
  say(DEBUG_IMPORTANT,"Done");

  say(DEBUG_IMPORTANT,"Setup Complete\n");
}

//computes an NMEA checksum
byte compute_checksum(byte *data,byte length)
{                
  byte computed_checksum=0;

  for (byte i = 0; i < length; i++)
  {
    computed_checksum = (byte)computed_checksum ^ data[i];
  }

  return computed_checksum;
}

//reads heading from HMC6343 compass
int readCompass() {
  byte buf[6];

  Wire.beginTransmission(HMC6343_ADDRESS); // Start communicating with the HMC6343 compasss
  Wire.write(HMC6343_HEADING_REG); // Send the address of the register that we want to read
  //Wire.write(0x55); // Send the address of the register that we want to read
  Wire.endTransmission();

  Wire.requestFrom(HMC6343_ADDRESS, 6); // Request six bytes of data from the HMC6343 compasss
  for(int i=0;i<6;i++)
  {
    while(Wire.available() < 1); // Busy wait while there is no byte to receive
    buf[i]=Wire.read();
    //printf("buf[%d]=%d\r\n",i,buf[i]);
  }
  int heading = ((buf[0] << 8) + buf[1]); // the heading in degrees
  int pitch =   ((buf[2] << 8) + buf[3]); // the pitch in degrees
  int roll = ((buf[4] << 8) + buf[5]); // the roll in degrees*/
  
   heading=heading/10;
   roll=roll/10;
   pitch=pitch/10;
   
   //myDebug.print("Heading = ");
   //myDebug.print(heading);

  i2c_data.data.roll=(int8_t)roll;
  i2c_data.data.pitch=(int8_t)pitch;
  i2c_data.data.heading=(uint16_t)heading;
  
  //dprintf(DEBUG_IMPORTANT,"Heading: %d Roll: %d Pitch: %d\r\n",i2c_data.data.heading,i2c_data.data.roll,i2c_data.data.pitch);
  //printf("heading=%d\r\n",heading);
  //dprintf2("heading=%d\r\n",heading);

  delay(100);
  
  return (int)heading; // Print the sensor readings to the serial port.
}


int readWindSensor() //get a raw reading from the wind sensor
{
  int offset=50; //offset for position of the wind sensor when its pointing straight ahead
  int pulseLength=0;
  int windAngle=0;
  pulseLength = pulseIn(WIND_SENSOR_PIN, HIGH, 2000);
  int magic = 29;
  windAngle =((pulseLength*10)/29); // 29 is the magic number where pulse time of 1036 = 359
  windAngle = windAngle - offset;//Compensate for offset
  windAngle = mod(windAngle); // Wrap Arround
  return (windAngle);
}

int getTrueWind()  //do wind averaging and keep track of true wind
{
  static float wind_avg_sin,wind_avg_cos;
  float cur_sin,cur_cos,avg_angle;
#define WIND_DECAY_RATE 250.0

  int i=mod(readWindSensor() + i2c_data.data.heading); //get the wind sensor reading (relative to boat) and add it to the compass heading to get it relative to north

  cur_sin=sin(deg2rad((double)i)); 
  cur_cos=cos(deg2rad((double)i));

  //take a running average of the sine and cosine of the wind angle
  wind_avg_sin = wind_avg_sin + ((cur_sin-wind_avg_sin) / WIND_DECAY_RATE);
  wind_avg_cos = wind_avg_cos + ((cur_cos-wind_avg_cos) / WIND_DECAY_RATE);

  avg_angle=rad2deg(atan2(wind_avg_sin,wind_avg_cos)); //recombine them into a single angle

  if(avg_angle<0.0)
  {
    avg_angle=avg_angle+360.0;
  }

  return (int) avg_angle;
}


void readGPS() {
  unsigned long fix_age=9999,time,date;

  say(DEBUG_MINOR,"About to read GPS");
  digitalWrite(GPS_ENABLE_PIN,1); //turn the GPS on
  delay(1000);


  while(fix_age == TinyGPS::GPS_INVALID_AGE||fix_age>3000) //make sure the GPS has a fix, this might cause a wait the first time, but it should be quick any subsequent time
  {
    Serial1.println("$PSRF103,04,01,00,01*21\r");
    dprintf(DEBUG_MINOR,"NMEA string: ");
    unsigned long start = millis();
    while(millis()<start+2000)
    {
      if(Serial1.available())
      {
        int c = Serial1.read();
        gps.encode(c);
        if(DEBUG_THRESHOLD>=DEBUG_MINOR)
        {
          myDebug.write(c);
        }
        if(c=='\n')
        {
          break;
        }
      }
    }
    gps.get_datetime(&date,&time,&fix_age);

    dprintf(DEBUG_MINOR,"fix age = %ld\r\n",fix_age);
    if(fix_age == TinyGPS::GPS_INVALID_AGE)
    {
      dprintf(DEBUG_IMPORTANT,"Invalid fix, fix_age=%ld\r\n",fix_age);
      say(DEBUG_IMPORTANT,"No GPS fix");
    }
  }

  digitalWrite(GPS_ENABLE_PIN,0); //turn the GPS off
   

  gps.get_datetime(&date,&time,&fix_age);
  gps.f_get_position(&i2c_data.data.lat,&i2c_data.data.lon,&fix_age);
   

  if(fix_age == TinyGPS::GPS_INVALID_AGE)
  {
    say(DEBUG_IMPORTANT,"Invalid fix");
  }

  else
  {
    
    /*Serial.println(date);

    Serial.print("time = ");
    Serial.println(time);

    Serial.print("fix_age = ");
    Serial.println(fix_age);*/

    say(DEBUG_IMPORTANT,"Fix Valid");
    dprintf(DEBUG_IMPORTANT,"lat=%ld lon=%ld\r\n",(long)(i2c_data.data.lat*1000),(long)(i2c_data.data.lon*1000));

    int year;
    byte month,day,hour,min,sec;
    unsigned long age;
      
    gps.crack_datetime(&year,&month,&day,&hour,&min,&sec,NULL,&age);
    
    setTime(hour,min,sec,day,month,year); //sets the time in the time library, lets us get unix time

  }

}

int mod(int value){ //keeps angles betweeen 0 and 360
  int newValue;
  if(value < 0){
    newValue = value + 360;
  }
  else if(value >= 360){
    newValue = value - 360;
  }
  else{
    newValue = value;
  }
  return newValue;
}

//calculates difference between two headings taking wrap around into account
int get_hdg_diff(int heading1,int heading2)
{
  int result;

  result = heading1-heading2;

  if(result<-180)
  {
    result = 360 + result;
    return result;
  } 

  if(result>180)
  {
    result = 0 - (360-result);
  }

  return result;
}

//send the buckets data structure to the other arduino
void transmit_data()
{
  byte buf[10];
  i2c_data.data.unixtime=now(); //timestamp the data


  say(DEBUG_MINOR,"Sending empty wakeup message");
  Wire.beginTransmission(4); // transmit to device #4
  Wire.endTransmission();

  delay(5);
  say(DEBUG_MINOR,"About to transmit");

  Wire.beginTransmission(4); // transmit to device #4
  buf[0]=i2c_data.buffer[0]; 
  buf[1]=i2c_data.buffer[1];
  dprintf(DEBUG_MINOR,"heading[0]=%d heading[1]=%d\r\n",buf[0],buf[1]);
  buf[2]=compute_checksum(buf,2); //compute a checksum 
  Wire.write(HEADING);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.write(buf[2]);
  Wire.endTransmission();    // stop transmitting  
  delay(5);

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[2]; 
  buf[1]=i2c_data.buffer[3];
  dprintf(DEBUG_MINOR,"wind[0]=%d wind[1]=%d\r\n",buf[0],buf[1]);
  buf[2]=compute_checksum(buf,2); //compute a checksum 
  Wire.write(WIND_DIR);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.write(buf[2]);
  Wire.endTransmission();    // stop transmitting  
  delay(5);

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[4];
  dprintf(DEBUG_MINOR,"roll[0]=%d \r\n",buf[0]);
  buf[1]=compute_checksum(buf,1); //compute a checksum 
  Wire.write(ROLL);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.endTransmission();   
  delay(5);    

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[5];
  dprintf(DEBUG_MINOR,"pitch[0]=%d \r\n",buf[0]);
  buf[1]=compute_checksum(buf,1); //compute a checksum 
  Wire.write(PITCH);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.endTransmission();   
  delay(5);    

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[6];
  dprintf(DEBUG_MINOR,"rudder[0]=%d \r\n",buf[0]);
  buf[1]=compute_checksum(buf,1); //compute a checksum 
  Wire.write(RUDDER);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.endTransmission();   
  delay(5);    

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[7];
  dprintf(DEBUG_MINOR,"sail[0]=%d \r\n",buf[0]);  
  buf[1]=compute_checksum(buf,1); //compute a checksum 
  Wire.write(SAIL);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.endTransmission();   
  delay(5); 

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[8];
  buf[1]=i2c_data.buffer[9];
  buf[2]=i2c_data.buffer[10];
  buf[3]=i2c_data.buffer[11];
  buf[4]=compute_checksum(buf,4); //compute a checksum 
  Wire.write(LAT);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.write(buf[2]);
  Wire.write(buf[3]);
  Wire.write(buf[4]);
  Wire.endTransmission();   
  delay(5);       

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[12];
  buf[1]=i2c_data.buffer[13];
  buf[2]=i2c_data.buffer[14];
  buf[3]=i2c_data.buffer[15];
  buf[4]=compute_checksum(buf,4); //compute a checksum 
  Wire.write(LON);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.write(buf[2]);
  Wire.write(buf[3]);
  Wire.write(buf[4]);
  Wire.endTransmission();   
  delay(5);   

  Wire.beginTransmission(4); 
  buf[0]=i2c_data.buffer[16];
  buf[1]=i2c_data.buffer[17];
  buf[2]=i2c_data.buffer[18];
  buf[3]=i2c_data.buffer[19];
  buf[4]=compute_checksum(buf,4); //compute a checksum 
  Wire.write(TIME);
  Wire.write(buf[0]);
  Wire.write(buf[1]);
  Wire.write(buf[2]);
  Wire.write(buf[3]);
  Wire.write(buf[4]); 
  Wire.endTransmission();   
  delay(5);     

  say(DEBUG_MINOR,"Transmit complete");
/*
  //just test code???
  i2c_data.data.heading++;
  if(i2c_data.data.heading>359)
  {
    i2c_data.data.heading=0;
  }

  i2c_data.data.wind_dir++;
  if(i2c_data.data.wind_dir>359)
  {
    i2c_data.data.wind_dir=0;
  }

  i2c_data.data.roll++;
  if(i2c_data.data.roll>62)
  {
    i2c_data.data.roll=-62;
  }

  i2c_data.data.pitch++;
  if(i2c_data.data.pitch>62)
  {
    i2c_data.data.pitch=-62;
  }

  i2c_data.data.rudder++;
  if(i2c_data.data.rudder>5)
  {
    i2c_data.data.rudder=-5;
  }

  i2c_data.data.sail++;
  if(i2c_data.data.sail>5)
  {
    i2c_data.data.sail=0;
  }*/
}

void sailLogic(int relwind)
{
  if(relwind >= 0 && relwind < 15) // in irons, help us tack?
  {
    i2c_data.data.sail = 0;
  }
  else if (relwind >= 15 && relwind < 50) //close hauled
  {
    i2c_data.data.sail = 1;
  }
  else if (relwind >= 50 && relwind < 80) //close reach
  {
    i2c_data.data.sail = 2;
  }
  else if (relwind >= 80 && relwind < 120) //beam reach
  {
    i2c_data.data.sail = 3;
  }
  else if (relwind >= 120 && relwind < 145) //broad reach
  {
    i2c_data.data.sail = 4;
  }
  else if (relwind < 180) //run
  {
    i2c_data.data.sail = 5;
  }
  else if (relwind >= 180 && relwind < 215) // run
  {
    i2c_data.data.sail = 5;
  }
  else if (relwind >= 215 && relwind < 240) //broad reach
  {
    i2c_data.data.sail = 4;
  }
  else if (relwind >= 240 && relwind < 280) //beam reach
  {
    i2c_data.data.sail = 3;
  }
  else if (relwind >= 280 && relwind < 310) //close hauled
  {
    i2c_data.data.sail = 2;
  }
  else if (relwind <= 345)
  {
    i2c_data.data.sail = 1;
  }
  else
  {
    i2c_data.data.sail = 0;
  }

  //dprintf(DEBUG_MINOR,"sailLogic: relwind=%d sailpos=%d\r\n",relwind,i2c_data.data.sail);

  sailServo.writeMicroseconds(1000+(200*i2c_data.data.sail)); //set the servo, data is abs'ed as we can't control which tack the sail is actually on

}

void loop()
{
  unsigned long last_gps_read=0;
  unsigned long last_time=0,time_now=0;
  int wp_hdg=0;
  float wp_dist=0.0;
  int wp_num=0;

  float igain=0.01;
  float pgain=0.1;
  float running_err=0.0;
  int hdg_err=0;
  int relwind;
  
  long last_telemetry=0;
  #define TELEMETRY_INTERVAL 10
  #define TARGET_LOOP_INTERVAL 100 //number of milliseconds between loop intervals

  #define NUM_OF_WAYPOINTS 1


  float wp_lats[NUM_OF_WAYPOINTS]; 
  float wp_lons[NUM_OF_WAYPOINTS];

  wp_lats[0]=52.4;
  wp_lons[0]=-4.4;


  while(1)
  {
    
    //make loop execute at constant speed
    time_now=millis();

    if(time_now-last_time>0&&time_now-last_time<TARGET_LOOP_INTERVAL)
    {
      delay(TARGET_LOOP_INTERVAL-(time_now-last_time));
    }

    last_time=millis();
    
    readCompass();
    

    i2c_data.data.wind_dir=getTrueWind();

    relwind=mod(i2c_data.data.wind_dir - i2c_data.data.heading);
    

    if(millis()-last_gps_read>(GPS_READ_INTERVAL*1000)||millis()<last_gps_read) //read the GPS at the specified interval or whenever the millis count wraps around 
    {
      say(DEBUG_MINOR,"Reading GPS");
      readGPS();
      wp_hdg = (int) TinyGPS::course_to(i2c_data.data.lat, i2c_data.data.lon, wp_lats[wp_num],wp_lons[wp_num]);
      wp_dist = TinyGPS::distance_between(i2c_data.data.lat, i2c_data.data.lon, wp_lats[wp_num],wp_lons[wp_num]);
      if(wp_dist<WP_THRESHOLD)
      {       
        wp_num++;      
        if(wp_num==NUM_OF_WAYPOINTS) //reached last waypoint already
        {
          wp_num--;          
        }
        else //reached new waypoint
        {
          wp_hdg = (int) TinyGPS::course_to(i2c_data.data.lat, i2c_data.data.lon,wp_lats[wp_num],wp_lons[wp_num]);
          wp_dist = TinyGPS::distance_between(i2c_data.data.lat, i2c_data.data.lon, wp_lats[wp_num],wp_lons[wp_num]);
        }
      }
      last_gps_read=millis();
    }



    //sail logic
    sailLogic(relwind);

    //rudder logic
    hdg_err = get_hdg_diff(wp_hdg,i2c_data.data.heading);

    running_err = running_err + (float)hdg_err;
    if (abs(running_err > 4000))
    {
      running_err = 4000; // limit integral component
    }
    running_err = running_err * 0.9;   

    /*dprintf("hdg_err = %d running_err = ",hdg_err);
    myDebug.println(running_err);*/

    i2c_data.data.rudder = (int) round((pgain * (float)hdg_err) + (igain * running_err));
    if(i2c_data.data.rudder<-5)
    {
      i2c_data.data.rudder=-5;
    }
    else if(i2c_data.data.rudder>5)
    {
      i2c_data.data.rudder=5;
    }

    rudderServo.writeMicroseconds(1500+(i2c_data.data.rudder*100));

    if(last_telemetry+(TELEMETRY_INTERVAL*1000)<millis())
    {
      dprintf(DEBUG_CRITICAL,"time=%ld hdg=%d hdg_err=%d roll=%d pitch=%d truewind=%d relwind=%d sail=%d rudder=%d wp_num=%d wp_hdg=%d wp_dist=%ld ",now(),i2c_data.data.heading,hdg_err,i2c_data.data.roll,i2c_data.data.pitch,i2c_data.data.wind_dir,relwind,i2c_data.data.sail,i2c_data.data.rudder,wp_num,wp_hdg,(long)wp_dist);
      
      //time=181734082 hdg=-14836 hdg_err=9217 roll=-22526 pitch=24182 truewind=-25261 relwind=27648 sail=-6656 rudder=-7937 wp_hdg=2768 wp_dist=-284423467 lat=52.41648 lon=-4.06522 wplat=52.40000 wplon=-4.40000 running_err=966.57
      
      // time=1398700112 hdg=158 hdg_err=107 roll=-26 pitch=-32 truewind=302 relwind=144 sail=4 rudder=5 Debug1: [Ctrl] wp_hdg=265 wp_dist=22842 lat=52.41666 lon=-4.06445 wplat=52.40000 wplon=-4.40000 running_err=963.96

      if(DEBUG_THRESHOLD>=DEBUG_CRITICAL)
      {
        myDebug.print("lat=");
        myDebug.print(i2c_data.data.lat,5);
        myDebug.print(" lon=");
        myDebug.print(i2c_data.data.lon,5);
        myDebug.print(" wplat=");
        myDebug.print(wp_lats[wp_num],5);
        myDebug.print(" wplon=");
        myDebug.print(wp_lons[wp_num],5);
        myDebug.print(" running_err=");
        myDebug.println(running_err);
      }
      
      transmit_data();
      last_telemetry=millis();
    }
    /*else
    {
      printf("time=%ld\r\n",now());
    }*/
   

  }
}


