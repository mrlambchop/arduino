/*
Copyright 2014 Nick Lambourne

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <CapacitiveSensor.h>

/*********************
** 
**
***********************/

//do we support the basic serial console? 
#define USE_SERIAL_CONSOLE

//the GPIO pin mapping
const int rf_tx_pin = 12;
const int cap_rx_pin = 2;
const int cap_tx_pin = 4;
const int led_pin = 13;

#define LEADING_PULSE_IN_USEC 10000
#define SHORT_PULSE_IN_USEC 500
#define LONG_PULSE_IN_USEC 2000

#define ON LOW
#define OFF HIGH

#define PRINT_LONG( s, l ) Serial.print( s ); Serial.println( l, DEC )
#define PRINT_LONG_HEX( s, l ) Serial.print( s ); Serial.println( l, HEX )


CapacitiveSensor  cs = CapacitiveSensor(cap_tx_pin,cap_rx_pin);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired

static void rf_tx_bit( unsigned char b )
{
    //Serial.println( b ? "1" : "0" );
  
    if( b == 0 )
    {
        digitalWrite( rf_tx_pin, 1 );
        delayMicroseconds( LONG_PULSE_IN_USEC );
        digitalWrite( rf_tx_pin, 0 );
        delayMicroseconds( SHORT_PULSE_IN_USEC );
    }
    else
    {
        digitalWrite( rf_tx_pin, 1 );
        delayMicroseconds( SHORT_PULSE_IN_USEC );
        digitalWrite( rf_tx_pin, 0 );
        delayMicroseconds( LONG_PULSE_IN_USEC );
    }    
}

static void rf_tx( unsigned short d )
{
    PRINT_LONG_HEX( "TX: ", d );
  
    digitalWrite( rf_tx_pin, 0 );  
    delayMicroseconds( LEADING_PULSE_IN_USEC );
    
    for( int i = 0; i < 16; i++ )
    {
        if( d & 0x8000 )
            rf_tx_bit( 1 );
        else
            rf_tx_bit( 0 );

        d <<= 1;
    }  
    
    digitalWrite( rf_tx_pin, 0 );  
    delayMicroseconds( LEADING_PULSE_IN_USEC );    
}


void setup()
{
  pinMode(rf_tx_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  
  digitalWrite( rf_tx_pin, 0 );
  digitalWrite( led_pin, 1 ); delay( 500 );   digitalWrite( led_pin, 0 );

  // Open serial communications
  Serial.begin(115200);
  Serial.println( "Welcome" ); 
  
  cs.reset_CS_AutoCal();
}



void control_light( byte light, byte on_off )
{
    static unsigned short lights[3][2] = { {0x688C, 0x684C}, {0x682C, 0x681C}, {0x690C, 0x6A0C} }; 
    
    PRINT_LONG_HEX("control_light: ", light );
    PRINT_LONG_HEX("on_off: ", on_off );
    
    for( int i = 0; i < 10; i++ )
    {
       rf_tx( ~lights[light][on_off ? 0 : 1] );
    }
}

static int on = 0;

bool sensor_triggered( byte on )
{
    //2 counters - one for timeout, one for time in
    int time_remaining = 10;
    int timeout = 10;        
    
    if( !on )
        timeout = 1000; //huge timeout in the case where nobody releases the wire
    
    while( time_remaining && timeout )
    {
        bool ok = false;
        long total =  cs.capacitiveSensor(30);  
        
        PRINT_LONG( "t: ", total );
       
        if( on && (total > 400) )
            ok = true;
       
        if( !on && (total < 400) )
            ok = true;       
        
        if( ok )
        {
            time_remaining--; 
        }
        else
        {
            timeout--;
        }
        
        PRINT_LONG( "time_remaining: ", time_remaining );            
        PRINT_LONG( "timeout: ", timeout );                        
        
        delay(20);
    }
    
    if( 0 == timeout )
        return false;
    else
        return true;
}

//the main loop
void loop()
{
   //wait for the sensor to trigger
   if( sensor_triggered( 1 ) )
   {
       //toggle the state!
      if( on )
      {
          control_light( 0, OFF ); 
          control_light( 1, OFF ); 
          control_light( 2, OFF );
      }
      else
      {
          control_light( 0, ON ); 
          control_light( 1, ON ); 
          control_light( 2, ON );
      }      
      
      on = !on;       
     
       while( sensor_triggered( 0 ) ); 
   }  

   digitalWrite( led_pin, on );   
}

/********************************************************************
** Serial console stuff for debug
*********************************************************************/

#ifdef USE_SERIAL_CONSOLE
void add_digit( unsigned int *val, char c )
{
    if( (c >= '0') && (c <= '9') )
    {
        *val *= 10;
        *val +=  ((c - '0') & 0xff);
    }
}

#define IS_SPACE( c ) (c == ' ')

//tmp serial console buffer
static char buffer[16] = { 0 };
static byte buf_index = 0;
static byte last_buffer_index = 0;

//format is:
//<t> <0 to n>
void do_rf( void )
{
    unsigned char index = 0;
    static unsigned short addr[] = { 0x688C, 0x684C, 0x682C, 0x681C, 0x690C, 0x6A0C };  
    #define NUM_ADDR sizeof(addr) / sizeof(unsigned short)
  
    if( IS_SPACE( buffer[1] ) )  
    {
       index = buffer[2] - '0';
       
       if( (index >= 0) && (index < NUM_ADDR) )
       {   
            PRINT_LONG( "index: ", index );
            for( int i = 0; i < 40; i++ )
            {
              rf_tx( ~addr[index] );
            }
       }
    }
    else
        Serial.println("relay bad");
}

//<l> <number> <0 or 1 for off/on>
void do_light( void )
{
    unsigned char light = 0;
    unsigned char on_off = 0;    
  
    if( IS_SPACE( buffer[1] ) && IS_SPACE( buffer[3] ) )  
    {
       light = buffer[2] - '0';
       on_off = buffer[4] - '0';       
       
       control_light( light, on_off );
    }
    else
        Serial.println("relay bad");
}

#endif //USE_SERIAL_CONSOLE


void serialEvent()
{
  #ifdef USE_SERIAL_CONSOLE
  if (Serial.available())
  {
      char c = Serial.read();
      
      if( (c == '\r') || (c == '\n') )
      {
         if( buf_index == 0 )
            buf_index = last_buffer_index;
         
         buffer[ buf_index ] = 0;         
         Serial.println("");

         switch( buffer[0] )
         {
            //rf
            case 't':
                do_rf();
            break;

            //light
            case 'l':
                do_light();
            break;            
           
            //CONSOLE
            default:
                Serial.print(": ");
            break;  
         }  
         
         last_buffer_index = buf_index;

         memset( buffer, 0, sizeof( buffer ) );
         buf_index = 0;
      }
      else
      {
         if( buf_index <= 15 )        
         {
             buffer[ buf_index ] = c;
             
             buf_index++;
         }
      }

      Serial.write(c);
  }
  #endif
}

