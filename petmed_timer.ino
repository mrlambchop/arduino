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

/*********************
** A very small app that is used to time pet medication when multiple people look after a pet.
**
** A pet has the medication every X days, in the morning only.
** Every morning, the person feeding the pet presses the button to see if its medication day.
** If the light illuminates 'red', its not medication day. If it lights green, give the tablet
**
** The day of the medication is stored in EEPROM so it works over a power cycle
** To reset the medication day, hold the button for 2+ seconds. When released, the LED will flash blue
**   and then green (restting the medication day to now).
**
***********************/

#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

//use the button ISR, not the watchdog
//#define USE_WATCHDOG_WAKEUP

//do we support the basic serial console? 
#define USE_SERIAL_CONSOLE

static void sleep_helper( void );

//the GPIO pin mapping
const int r =  11;      
const int g =  10;      
const int b =  6;      
const int button = 2;

//number of days between medication
const int days = 3;

//LED control

#define ON LOW
#define OFF HIGH

#define RED_LED( level )  digitalWrite(r, level )
#define GREEN_LED( level )  digitalWrite(g, level )
#define BLUE_LED( level )  digitalWrite(b, level )

//EEPROM helper macros

#define READ_COUNT() EEPROM.read(0)
#define WRITE_COUNT( c ) EEPROM.write(0, c)

static long previousMillis = 0;        // will store last time LED was updated
static int cancel = 0;

//this is only used if USE_WATCHDOG_WAKEUP is set

#ifdef USE_WATCHDOG_WAKEUP
ISR( WDT_vect ) {
  /* dummy */
}
#endif

int int_level = 1;
void pin2Interrupt(void)
{
    int_level = !int_level;
    attachInterrupt(0, pin2Interrupt, int_level ? LOW : HIGH );
}

unsigned long last_wakeup = 0;
unsigned int count = 0;

void setup()
{
  //RGB LED control
  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
  
  //bright green led to say "hi, we've booted"
  digitalWrite(r, 1 );
  digitalWrite(g, 0 );
  digitalWrite(b, 1 );
    
  pinMode(button, INPUT_PULLUP );
  
  // Open serial communications
  Serial.begin(115200);
  Serial.println( "Welcome" );
  
  //resetting this here means we'll run for a single wakeup period before sleeping (keeps the green light on for a little while)
  last_wakeup = millis();
}


int check_if_button_held_down( void )
{
    volatile long now = millis();
  
    while( digitalRead( button ) == LOW );
  
    //if the button is pressed for a while, we reset our pet feeder count
    if( (now + 1000) < millis() )
    {
      Serial.println( now, DEC );
      Serial.println( millis(), DEC );      
      
      Serial.println( "Reset" );
      
      BLUE_LED( ON );
      delay( 250 ); 
      BLUE_LED( OFF );   
      delay( 250 );      
      BLUE_LED( ON );
      delay( 250 );
      BLUE_LED( OFF );
      
      WRITE_COUNT( 0 ); //reset the counter
      
      return 1;
    }
    
    return 0;
}

static void sleep_helper( void )
{
  #ifdef USE_WATCHDOG_WAKEUP
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);             // select the watchdog timer mode
    MCUSR &= ~(1 << WDRF);                           // reset status flag
    WDTCSR |= (1 << WDCE) | (1 << WDE);              // enable configuration changes
    WDTCSR = (1<< WDP0) | (1 << WDP1) | (1 << WDP2); // set the prescalar = 7
    WDTCSR |= (1 << WDIE);                           // enable interrupt mode
    sleep_enable();                                  // enable the sleep mode ready for use
    sleep_mode();                                    // trigger the sleep
    /* ...time passes ... */
    
    sleep_disable();                                 // prevent further sleeps
  #else
    /* Setup pin2 as an interrupt and attach handler. */
    attachInterrupt(0, pin2Interrupt, int_level ? LOW : HIGH );
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    
    sleep_enable();
    
    sleep_mode();
    
    /* The program will continue from here. */
    
    /* First thing to do is disable sleep. */
    sleep_disable(); 
  #endif
  
  Serial.println("Sleep out");
}

//the main loop
void loop()
{
  if( ((millis() - last_wakeup) > 1000) && (cancel == 0) )
  {
     RED_LED( OFF );
     GREEN_LED( OFF );
     BLUE_LED( OFF );   
    
     Serial.write("Sleeping...");
    
     do_sleep();
        
     check_if_button_held_down();

     last_wakeup = millis();
     
     //always do this
     {   
        int count = READ_COUNT();
       
        if( (count % days) == 0 )
           GREEN_LED( ON );
        else
           RED_LED( ON );
           
        count++;
        WRITE_COUNT( count );
     }
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

//tmp serial console buffer
static char buffer[16] = { 0 };
static byte buf_index = 0;

void do_rgb( int led )
{
   unsigned int val = 0;  

   add_digit( &val, buffer[ 1 ] );
   add_digit( &val, buffer[ 2 ] );
   add_digit( &val, buffer[ 3 ] );
   add_digit( &val, buffer[ 4 ] );

   val = 1023 - val;

   analogWrite(led, val);
}


void do_button( void )
{
    if( digitalRead( button ) )
        Serial.println( "No" );
    else
        Serial.println( "Yes" );
}


void do_sleep( void )
{
    sleep_helper();
}

//UART FORMAT
//e <r|w> <addr> <val>
void do_eeprom( void )
{
   unsigned int addr = 0;
   unsigned int val = 0;   
   
   if( buf_index != 9 )
   {
       Serial.println("EEPROM bad values");
       return;
   }
   
   if( (buffer[1] != ' ') || (buffer[3] != ' ') || (buffer[6] != ' ') )
   {
       Serial.println("EEPROM bad format");
       return;
   }   

   add_digit( &addr, buffer[ 4 ] );
   add_digit( &addr, buffer[ 5 ] );
   
   add_digit( &val, buffer[ 7 ] );
   add_digit( &val, buffer[ 8 ] );
  
   switch( buffer[2] )
   {
       case 'w':
           EEPROM.write(addr, val );     
           //fall through
     
       case 'r':
           Serial.println( EEPROM.read(addr), HEX );
       break;       
       
       case 'c':
         // write a 0 to all 512 bytes of the EEPROM
          for (int i = 0; i < 512; i++)
              EEPROM.write(i, 0);
       break;
   }
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
            //RGB
            case 'r':
                do_rgb( r );
            break; 
            case 'g':
                do_rgb( g );    
            break;     
            case 'b':
                do_rgb( b );       
            break;         
         
            case 'i':
               do_button();
            break;   
            
            case 'e':
               do_eeprom();
            break;               
            
            case 's':
                cancel = 0;
                do_sleep();
            break;
            
            case 'c':
                cancel = 1;
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
      {`
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
