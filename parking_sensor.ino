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

#include <Arduino.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

/*********************
** The parking distance helper works by turning on a large 60W bulb when the car is close to the target. In this case, the target is the rear of the garage.
**
** In normal mode, the sensor turns on every 1 second and checks to see if there is an obstacle coming close to it. If so, it switches into a high speed
**  tracking mode where it polls the distance sensor every 50ms, averaging the last X results to get a fairly accurate reading. 
**
** When a object is within X cm's of the sensor, a relay is controlled that turns on a 60W bulb. This continues to light up until the object either leaves the 30cm distance
**  or 30 seconds elapses.
**
** When an object is within 30cm all the time, the sensor checks every 1 second still and once the object leaves the area, it starts looking for a new object after a grace period of 10 seconds.
**
** This currently doesn't support any low power operation - the arduino is always powered by a USB wallwort at the moment.
**
***********************/

//do we support the basic serial console? 
#define USE_SERIAL_CONSOLE
#define USE_HCSR04

//Sensor defines for the object
#define TOO_CLOSE 30 //cm
#define WAKEUP_RANGE 100 //cm

#define TEN_SEC_IN_MS (10 * 1000)
#define THIRTY_SEC_IN_MS (10 * 1000)

#define IS_TOO_CLOSE( d ) (d <= TOO_CLOSE)
#define IN_RANGE( d )     (d <= WAKEUP_RANGE)

enum {
    STATE_OUT,
    STATE_IN_RANGE,
    STATE_TOO_CLOSE,
    STATE_SETTLED,
    STATE_MAX
};

//the GPIO pin mapping
const int relay_0 = 11;
const int relay_1 = 12;  

//hcs04 pins
const int trig = 8;
const int echo = 9;

//led pins
const int sensor_active_led = 13;
const int status_led = 6;

//times used for sensor polling
#define SLOW_POLL    1000
#define FAST_POLL    50
#define SETTLED_POLL 10000

//LED control
#define SENSOR_LED 0
#define STATUS_LED 1

#define ON LOW
#define OFF HIGH

#define RED_LED( level )  digitalWrite(r, level )

static void relay_control( unsigned char relay, unsigned char on_off )
{
    Serial.print( "relay: " );
    Serial.print( relay, DEC );
    
    if( on_off )
        Serial.println( " on" );
    else
        Serial.println( " off" );    

    digitalWrite( relay == 0 ? relay_0 : relay_1, on_off ? ON : OFF );    
}

static void led_control( unsigned char led, unsigned char on_off )
{
    Serial.print( "led: " );
    Serial.print( led, DEC );
    
    if( on_off )
        Serial.println( " on" );
    else
        Serial.println( " off" );    

    digitalWrite( led == SENSOR_LED ? sensor_active_led : status_led, on_off ? ON : OFF );
}

#define PRINT_LONG( s, l ) Serial.print( s ); Serial.println( l, DEC )

long values[4] = { 200, 200, 200, 200 };
#define NUM_VALUES (sizeof( values ) / sizeof( values[0] ))
int values_index = 0;

#define VALUE_BY_HISTORY( t ) values[ ((values_index + NUM_VALUES) - t) % NUM_VALUES ]

static long calc_weight_average( void )
{
    long distance = 0;
    
    PRINT_LONG( "0: ", VALUE_BY_HISTORY( 0 ) );
    PRINT_LONG( "-1: ", VALUE_BY_HISTORY( -1 ) );
    PRINT_LONG( "-2: ", VALUE_BY_HISTORY( -2 ) );
    PRINT_LONG( "-3: ", VALUE_BY_HISTORY( -3 ) );    

    distance += VALUE_BY_HISTORY( 0 ) * 2; //x4 - pretty important
    distance += VALUE_BY_HISTORY( -1 ) * 1; //x2
    distance += VALUE_BY_HISTORY( -2 ) * 1; 
    distance += VALUE_BY_HISTORY( -3 ) * 0; 
    
    distance /= 4;
    
    return distance;
}

static long hcsr04_get_distance( void )
{
  long ret = 0;
  long duration = 0;
  long distance = 0;  
  
  led_control( SENSOR_LED, ON );
  
  digitalWrite( trig, 1 );
  delayMicroseconds( 10 );
  digitalWrite( trig, 0 );
  duration = pulseIn( echo, HIGH);
  distance = (duration/2) / 29.1;
  
  if( distance < 200 ) //max is 400cm, but 220cm seems to be really the max limit
  {  
    Serial.print( "Duration: " ); Serial.println( duration, DEC );
    Serial.print( "Distance: " ); Serial.println( distance, DEC );  
    
    values[values_index++] = distance;
    values_index %= (sizeof( values ) / sizeof( values[0] ));
  }
  
  //calculated weighted average
  ret = calc_weight_average();
  
  Serial.print( "Avg: " ); Serial.println( ret, DEC );    
  
  led_control( SENSOR_LED, OFF );  
  
  return ret;
}


void setup()
{
  pinMode(relay_0, OUTPUT);
  pinMode(relay_1, OUTPUT);
  
  pinMode(trig, OUTPUT);
  digitalWrite( trig, 0 );
  pinMode(echo, INPUT);  
  
  // Open serial communications
  Serial.begin(115200);
  Serial.println( "Welcome" );
  
  //off by default
  relay_control( 0, 0 );
  relay_control( 1, 0 );
  
  led_control( SENSOR_LED, OFF );
  led_control( STATUS_LED, OFF );
  
  //at startup, we flash the LED on and off for 10 seconds
  //this needs the sensor to be cleared so it can start the process
  for( int i = 0; i < 5; i++ )
  {
      led_control( STATUS_LED, ON );
      delay( 1000 );
      led_control( STATUS_LED, OFF );
      delay( 1000 );      
  }
}

static unsigned int state = STATE_OUT;
static long state_last_change_time = 0; //how long in each state period.

const long delays[STATE_MAX] = { SLOW_POLL, FAST_POLL, FAST_POLL, SETTLED_POLL };
const bool long_range_check[STATE_MAX] = { true, false, false, true };

const char *state_names[STATE_MAX] = { "OUT", "IN_RANGE", "TOO_CLOSE", "SETTLED" };
#define PRINT_STATE( s ) Serial.println( state_names[s] )


static unsigned int range_check( bool long_check )
{
    unsigned int new_status;
    long averaged_distance = 0;
    
     Serial.print("range_check: "); Serial.println( long_check ? "long" : "normal" ); 
    
    for( int i = 0; i < (long_check ? 8 : 1); i++ )
    {
        averaged_distance = hcsr04_get_distance();
        Serial.print("range_check: "); Serial.println( long_check ? "long" : "normal" ); 

        if( (i > 1) && (i < 8) )
        {
            delayMicroseconds( 25 );
        }
    }
  
    if( IS_TOO_CLOSE( averaged_distance ) )
    {
        new_status = STATE_TOO_CLOSE;
        Serial.println("Calc: Too close");
    }
    else if( IN_RANGE( averaged_distance ) )
    {
        new_status = STATE_IN_RANGE;
        Serial.println("Calc: In range");
    }
    else
    {
        new_status = STATE_OUT;
        Serial.println("Calc: Range out");
    } 
    
    return new_status;
}


//the main loop
void loop()
{
    unsigned int old_state = state;
    
    Serial.print("Delay: "); Serial.println(delays[state], DEC); 

    delay( delays[state] );

    state = range_check( long_range_check[state] /*poll for a while or just a single pulse*/ );
   
    if( state != old_state )
        state_last_change_time = millis(); //record the last state change time
        
    PRINT_STATE( old_state );

    switch( old_state ) 
    {
        case STATE_TOO_CLOSE:

            //if we've just entered this state, turn on the light
            if( old_state == STATE_OUT )
            {
                relay_control( 0, ON );
            }

        //FALL THROUGH!!!
        case STATE_IN_RANGE:

            //if we've stuck in this state for 30 seconds, we can assume the object has settled
            if( (state_last_change_time + THIRTY_SEC_IN_MS) > millis() )
            {
                state = STATE_SETTLED;
            }
        break;
        
        case STATE_SETTLED:
        case STATE_OUT:
            relay_control( 0, OFF );
        break;
    }
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

//format is:
//<r> <0 or 1 - relay number> <1 or 0 - on or off>
void do_relay( void )
{
    unsigned char relay = 0;
    unsigned char on_off = 0;
  
    if( IS_SPACE( buffer[1] ) && IS_SPACE( buffer[3] ) )
    {
       relay = buffer[2] == '0' ? 0 : 1;
       on_off = buffer[4] == '0' ? 0 : 1;
    
       relay_control( relay, on_off );   
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
         buffer[ buf_index ] = 0;         
         Serial.println("");

         switch( buffer[0] )
         {
            //relay
            case 'r':
                do_relay();
            break;
           
            //CONSOLE
            default:
                Serial.print(": ");
            break;  
         }  

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

