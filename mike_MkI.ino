// =============================================================================
// mikeMkI v0.1
//
// Collision avoiding robot
// =============================================================================
//  
// Copyright (C) 2014 Alexandre Lopes
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// =============================================================================


#include <LiquidCrystal.h>
 
// Constants
const int COLL_DIST = 15;   // trigger collision, in cm
const int PING_DUR  = 10;  // ping duration in ms:
const int WAIT_TIME = 5;   // time we wait after sending ping
                           // according to the datasheet, time between
                           // measurements should be at least 60 ms
                           // of course the time it takes is dependent on how
                           // long the code after measurement takes
const int MIN_DIST = 2;    // minimum distance we can measure, in cm
const int MAX_DIST = 400;  // maximum distance we can measure, in cm
const int SPEED = 100;     // speed at which it moves
const unsigned long BCK_TIME = 3000; // how much time we should go back when 
                                     // we find an obstacle (in ms)

// Pins
// Note that analog pins can be used as digital ones
// 0 -> 14, 1 -> 15, ...
const int P_LCD_D4  = 0;
const int P_LCD_D5  = 1;
const int P_LCD_D6  = 2;
const int P_LCD_D7  = 4;
const int P_LCD_RS  = 5;
const int P_LCD_E   = 6;
const int P_HC_TRIG = 10;
const int P_HC_ECHO = 7;
const int P_LED     = 14;
const int P_BUTTON  = 16;
const int P_DIR_A   = 12;
const int P_DIR_B   = 13;
const int P_BRAKE_A = 9;
const int P_BRAKE_B = 8;
const int P_SPEED_A = 3;
const int P_SPEED_B = 11;

long dist;     // distance to object

char text[16]; // LCD text - one line
char blankLine[] = "                ";
// initialize library
LiquidCrystal lcd(P_LCD_RS, P_LCD_E, P_LCD_D4, P_LCD_D5, P_LCD_D6, P_LCD_D7);

enum states
{
    fwd,
    bck,
    left,
    right
};

// Initialize state - go forward
states state = fwd;

// Motors stopped or working
bool stop;

// Back time
unsigned long bckTime = 0;

// Button states
int prevState;
int currentState;

// setup routine
void setup() 
{
    // initialize button states
    prevState    = HIGH;
    currentState = HIGH;
       
    // set number of rows and columns
    lcd.begin(16,2);   

    // Initialize output digital pins
    pinMode(P_HC_TRIG, OUTPUT);
       
    // Initialize input digital pins
    pinMode(P_HC_ECHO, INPUT);
    pinMode(P_BUTTON, INPUT_PULLUP);
        
   
    // Robot starts stopped
    stop = true;
}

// the loop routine
void loop()
{
    if (stop == true)
    {
        // Print to LCD
        lcd.setCursor(0,0);
        lcd.print("Stopped!        ");
        lcd.setCursor(0,1);
        lcd.print(blankLine);

        // Detects button press - action only on button release
        
        // Reads value from the pin connected to the button
        // LOW if button is depressed, HIGH if not
        currentState = digitalRead(P_BUTTON);
        
        // If button is released and it was depressed before
        if (currentState == HIGH && prevState == LOW)
        {
            stop = false;
            // starts moving forward
            spinMotorA(SPEED,LOW);
            spinMotorB(SPEED,LOW);
            state = fwd;
            prevState = HIGH;

        }
        // If button is depressed
        else if (currentState == LOW)
        {
           lcd.print("Pressed.");
           prevState = LOW;
        }
    }
    else
    {
        switch (state)
        {
            case fwd:
                
                // Get distance to object
                dist = ping();
              
                // outside working range
                if ( (dist < MIN_DIST) || (dist > MAX_DIST) )
                {
                    // Print to LCD
                    lcd.setCursor(0,0);
                    lcd.print("No measurement! ");
                    lcd.setCursor(0,1);
                    lcd.print(blankLine);

                }
                // collision
                else if (dist < COLL_DIST)
                {
                    // Print to LCD
                    lcd.setCursor(0,0);
                    lcd.print("Collision!      ");
                    lcd.setCursor(0,1);
                    lcd.print(blankLine);
                    
                    // Change state and start moving bck
                    state   = bck;
                    bckTime = millis();
                    spinMotorA(SPEED,HIGH);
                    spinMotorB(SPEED,HIGH);

                }
                // good measurement
                else
                {
                    // Print to LCD
                    lcd.setCursor(0,0);
                    lcd.print("Measuring ...   ");
                    lcd.setCursor(0,1);
                    sprintf(text, "%d cm    ", dist);
                    lcd.print(text);

                }
            
            
                break;
            case bck:
                // Get distance to object
                dist = ping();
                // If we are have driven enough time backwards
                if ( millis()-bckTime > BCK_TIME)
                {
                    // As soon as we have gone enough backwards, go forward
                    spinMotorA(SPEED,LOW);
                    spinMotorB(SPEED,LOW);
                    state   = fwd;
                    bckTime = 0;
                }
                break;
            case left:
                break;
            case right:
                break;            
        }    
    
        // Detects button press - action only on button release
        
        // Reads value from the pin connected to the button
        // LOW if button is depressed, HIGH if not
        currentState = digitalRead(P_BUTTON);
        
        // If button is released and it was depressed before
        if (currentState == HIGH && prevState == LOW)
        {
            stop = true;
            // Engage breaks
            spinMotorA(0, 0);
            spinMotorB(0, 0);

            prevState = HIGH;
        }
        // If button is depressed
        else if (currentState == LOW)
        {
           prevState = LOW;
        }
    
    }
    
    
}

// Sends ping and captures echo. Outputs distance to object
long ping()
{
    long rtt; // round-trip time
    
    // Send ping
    // We set value to LOW before and after, to ensure a clean signal
    digitalWrite(P_HC_TRIG, LOW);
    delay(5);
    digitalWrite(P_HC_TRIG, HIGH);
    delay(PING_DUR);
    digitalWrite(P_HC_TRIG, LOW);

    // Capture echo
    rtt = pulseIn(P_HC_ECHO, HIGH);
    
    // Speed: 29 ms/cm. Divide by 2 to get distance to object, that is by 58.
    // Obviously not accurate if there's relative movement between detector
    // and object
    return rtt/58;
}

void spinMotorA(int speed, int direction)
{
    // Note that HIGH = 1 and LOW = 0, both int.
    // direction = 0 - forward
    // direction = 1 - backwards
    // speed \in [0, 255]
    
    // Sets direction
    digitalWrite(P_DIR_A, direction); 
    
    // Disengage brake
    digitalWrite(P_BRAKE_A, LOW);
    
    // Set spin velocity
    analogWrite(P_SPEED_A, speed);
}

void spinMotorB(int speed, int direction)
{
    // Note that HIGH = 1 and LOW = 0, both int.
    // direction = 0 - forward
    // direction = 1 - backwards
    // speed \in [0, 255]
    
    // Sets direction
    digitalWrite(P_DIR_B, direction);
    
    // Disengage brake
    digitalWrite(P_BRAKE_B, LOW);
    
    // Set spin velocity
    analogWrite(P_SPEED_B, speed);
}