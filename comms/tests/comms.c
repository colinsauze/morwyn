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

/*test program to check the bucketing code is working*/

#include <stdio.h>
#include <stdint.h>
#define byte uint8_t

union BucketData {
  struct Data{ 
    byte heading_bucket[10]; //10 degree intervals, 0-359, 10
    byte wind_bucket[10]; //10 degree intervals, 0-359, 20
    byte roll_bucket[14]; //10 degree intervals, <-60, -60 to +60 and >60, 32
    byte pitch_bucket[14]; //10 degree intervals, <-60, -60 to +60 and >60, 44
    byte motor_batt_current[16]; //0.2 amp intervals, 0-3 amps and over 3 amps, multiply by 10. 60
    byte motor_batt_voltage[17]; //0.2 volt intervals, <6, 6-9 and >9 volts, multiply by 10. 77
    byte comp_batt_current[6]; //0.2 amp intervals, 0-1 amps and over 1 amps, multiply by 10. 83
    byte comp_batt_voltage[10]; //0.2 volt intervals, <3, 3-4.6 and >4.6 volts, multiply by 10. 93
    byte rudder_position[11]; //-5 to +5 104
    byte sail_position[6]; //0 to 5 110
    float lat,lon; //current lat/lon 118
    int temp1,temp2; //temperature readings 122
  }data;
  
  byte buffer[sizeof(struct Data)];
  
}transmit;


void decode(byte *buffer)
{
    union BucketData *bucket_data;
    
    bucket_data = buffer;
     
    printf("\nBucket summary:\n");
    
    printf("Heading:\n");
    for(int i=0;i<10;i++)
    {
        printf("%d-%d: %d\n",i*36,((i+1)*36)-1,bucket_data->data.heading_bucket[i]);
    }
    
    printf("Wind:\n");
    for(int i=0;i<10;i++)
    {
        printf("%d-%d: %d\n",i*36,((i+1)*36)-1,bucket_data->data.wind_bucket[i]);
    }
    
    printf("Roll:\n");
    printf("<=-60: %d\n",bucket_data->data.roll_bucket[0]);
    
    for(int i=1;i<13;i++)
    {
        printf("%d to %d: %d\n",((i-1)*10)-60,((i-1)*10)-51,bucket_data->data.roll_bucket[i]);
    }
    printf(">=60: %d\n",bucket_data->data.roll_bucket[13]);
    
    printf("Pitch:\n");
    printf("<=-60: %d\n",bucket_data->data.pitch_bucket[0]);
    
    for(int i=1;i<13;i++)
    {
        printf("%d to %d: %d\n",((i-1)*10)-60,((i-1)*10)-51,bucket_data->data.pitch_bucket[i]);
    }
    printf(">=60: %d\n",bucket_data->data.pitch_bucket[13]);
    
    
    
}

void composeBinaryMessage(byte *buffer,int size)
{
    printf("0: ");
    for(int i=0;i<size;i++)
    {
        printf("%X ",buffer[i]);
        if(i%10==0)
        {
            printf("\n%d: ",i+1);
        }
    }
    decode(buffer);
    
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
    
    
    byte test_data[] = { 103,1,191,0,1,196,251,5,154,153,81,66,102,102,134,192,250,0,170,0};
 
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
      int16_t temp1;
      int16_t temp2;
    } data;
    byte recv_buffer[sizeof(struct Data)];
  } incoming;
  
  /*incoming.data.heading=359;
  incoming.data.wind_dir=191;
  incoming.data.roll=60;
  incoming.data.pitch=-60;
  incoming.data.rudder=-5;
  incoming.data.sail=5;
  incoming.data.lat=52.4;
  incoming.data.lon=-4.2;
  incoming.data.temp1=250;
  incoming.data.temp2=170;*/
  
   for(int i=0;i<20;i++)
  {
      //printf("%d,",incoming.recv_buffer[i]);
      incoming.recv_buffer[i]=test_data[i];
  }
  
  //process incoming data
  
  transmit.data.heading_bucket[incoming.data.heading/36]++;
  transmit.data.wind_bucket[incoming.data.wind_dir/36]++;

  if(incoming.data.roll<=-60)
  {
      transmit.data.roll_bucket[0]++;
  }
  else if(incoming.data.roll>=60)
  {
      transmit.data.roll_bucket[13]++;
  }
  else
  {
      transmit.data.roll_bucket[((incoming.data.roll+60)/10)+1]++;
  }

  
  if(incoming.data.pitch<=-60)
  {
      transmit.data.pitch_bucket[0]++;
  }
  else if(incoming.data.pitch>=60)
  {
      transmit.data.pitch_bucket[13]++;
  }
  else
  {
      transmit.data.pitch_bucket[((incoming.data.pitch+60)/10)+1]++;
  }
  
  transmit.data.rudder_position[incoming.data.rudder+5]++;
  transmit.data.sail_position[incoming.data.sail]++;
  
  composeBinaryMessage(transmit.buffer,122);
  
}

void main()
{
    receiveEvent(16);
}
