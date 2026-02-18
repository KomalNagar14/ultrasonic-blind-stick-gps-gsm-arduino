// Phase 2 : Unified Blind Stick Code (With Wheels + Sensors)

/*
Features Included:
    Triple Ultrasonic Obstacle Detection (Front, Left, Right)
    Water Sensor Detection
    Emergency SOS Button
    GPS Location Reading (via TinyGPS)
    GSM SMS Sending for SOS
    Voice Alerts using Talkie Library
    Optional Movement Control using Wheel Motors

This program is made for an advanced Blind Stick prototype
that can move autonomously when the ON switch is pressed,
and stops + alerts when obstacles/water/danger is detected.
*/


/* ------------------------- LIBRARIES ------------------------- */
#include <TinyGPS.h>            // Library to read GPS data (latitude, longitude)
#include <SoftwareSerial.h>     // Serial communication for GSM, To create serial communication on any digital pins
#include <Arduino.h>

#include "Talkie.h"             // Voice synthesis library
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include "Vocab_US_TI99.h"

// GSM module connected using SoftwareSerial
#define GSM_RX_PIN A3
#define GSM_TX_PIN A4
SoftwareSerial Gsm(GSM_RX_PIN, GSM_TX_PIN);     // A4 = Arduino RX, A5 = Arduino TX for GSM module

char phone_no[] = "+91XXXXXXXXXX";  // Replace with emergency contact number

TinyGPS gps;                    // To decode GPS NMEA data
Talkie voice;                   // Speech object for voice alerts



/* ------------------------- ULTRASONIC SENSOR PIN ------------------------- */
// Ultrasonic sensors (HC-SR04)
const int trigger_front = 7;
const int echo_front = 6;
long distance_front;

const int trigger_left = 9;
const int echo_left = 8;
long distance_left;

const int trigger_right = 5;
const int echo_right = 4;
long distance_right;


// ---------- ALERT DEVICES ----------
const int buzzer_pin = 13;          // Buzzer for sound alert
const int On = 10;              // Movement ON/OFF switch
int oN = 0;

// Water Sensor + Emergency
#define Water_Sensor_Pin A0     // Water detection sensor

// Emergency SOS button (active LOW)
const int switch_pin = 12;    // Emergency button (press to send SMS)
int State = 0;     // Stores emergency button status

// Wheel motors
const int leftM = A1;
const int rightM = A2;



/* ------------------------- SETUP ------------------------- */
void setup()
{
  Serial.begin(9600);           // Debug monitor
  Gsm.begin(57600);             // GSM communication baud rate

  // Setup Ultrasonic sensor pins
  pinMode(trigger_front, OUTPUT);
  pinMode(echo_front, INPUT);

  pinMode(trigger_left, OUTPUT);
  pinMode(echo_left, INPUT);

  pinMode(trigger_right, OUTPUT);
  pinMode(echo_right, INPUT);

  // Water sensor input + alert device
  pinMode(Water_Sensor_Pin, INPUT);
  pinMode(buzzer_pin, OUTPUT);   // Buzzer output
  //pinMode( motor, OUTPUT);

  // Wheel motors output
  pinMode(leftM, OUTPUT);
  pinMode(rightM, OUTPUT);

  // Default OFF states
  digitalWrite(buzzer_pin, LOW);
  // digitalWrite( motor,LOW);

  digitalWrite(leftM, LOW);
  digitalWrite(rightM, LOW);

  // Switches
  // INPUT_PULLUP = switch pressed -> LOW
  pinMode(switch_pin, INPUT_PULLUP);
  pinMode(On, INPUT_PULLUP);

  delay(1000);   // Allow module stabilization
}



/* ------------------------- ULTRASONIC FUNCTIONS ------------------------- */
// These 3 functions generate ultrasonic trigger pulses,
// read the echo time, and convert it to distance in centimeters.

void check_distance_front()
{
  long dis;
  digitalWrite(trigger_front, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_front, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger_front, LOW);

  dis = pulseIn(echo_front, HIGH);
  distance_front = dis / 29 / 2;   // Speed of sound 29µs/cm
  //Serial.print("distance front:");
  //Serial.println(distance_front);
}

void check_distance_left()
{
  long dis;
  digitalWrite(trigger_left, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_left, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger_left, LOW);

  dis = pulseIn(echo_left, HIGH);
  distance_left = dis / 29 / 2;
  //Serial.print("distance left:");
  // Serial.println(distance_left);
}

void check_distance_right()
{
  long dis;
  digitalWrite(trigger_right, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_right, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger_right, LOW);

  dis = pulseIn(echo_right, HIGH);
  distance_right = dis / 29 / 2;
  //Serial.print("distance right:");
  //Serial.println(distance_right);
}



/* ------------------------- MAIN LOOP ------------------------- */
void loop()
{
  
  // 1) CHECK EMERGENCY SWITCH FIRST
  
  State = digitalRead(switch_pin);

  if (State == LOW)     // Emergency Button Pressed
  {
    digitalWrite(leftM, LOW);
    digitalWrite(rightM, LOW);

    // Voice warning
    voice.say(sp2_DANGER);
    voice.say(sp2_DANGER);
    voice.say(spt_STOP);
    voice.say(sp2_ALERT);

    Serial.println("HELP");

    delay(1000);
    send_message();     // Send SOS with GPS link
    delay(1000);
  }


  // 2) IF MOVEMENT SWITCH IS ON – Enable movement + sensors
  oN = digitalRead(On);

  if (oN == LOW)   // Movement ON
  {
    digitalWrite(leftM, HIGH);
    digitalWrite(rightM, HIGH);

    check_distance_front();  // Check forward obstacle



    // 2A) WATER SENSOR CHECK
    // HIGH = Water detected
    if (digitalRead(Water_Sensor_Pin) == HIGH)
    {
      // digitalWrite( motor_pin,HIGH);
      digitalWrite(buzzer_pin, HIGH);
      Serial.println("WATER DETECTED");
      delay(2000);
    }
    else
    {
      //  digitalWrite( motor_pin,LOW);
      digitalWrite(buzzer_pin, LOW);
    }



    // 2B) FRONT OBSTACLE CHECK
    if (distance_front <= 50)
    {
      digitalWrite(leftM, LOW);
      digitalWrite(rightM, LOW);

      voice.say(sp2_DANGER);
      voice.say(sp2_DANGER);
      voice.say(spt_FRONT);
      voice.say(sp2_ALERT);

      Serial.println("FRONT OBSTACLE");
      delay(1000);

      // Check side sensors after stopping
      check_distance_left();
      check_distance_right();



      // 2C) TURN LEFT IF LEFT SIDE ALSO BLOCKED
      if (distance_left <= 50)
      {
        digitalWrite(leftM, HIGH);
        digitalWrite(rightM, LOW);

        voice.say(sp2_DANGER);
        voice.say(sp2_DANGER);
        voice.say(sp2_LEFT);
        voice.say(sp2_ALERT);

        Serial.println("LEFT BLOCKED - TURNING LEFT");
        delay(1000);

        digitalWrite(leftM, LOW);
        digitalWrite(rightM, LOW);
      }



      // 2D) TURN RIGHT IF RIGHT SIDE ALSO BLOCKED
      else if (distance_right <= 50)  //80
      {
        digitalWrite(leftM, LOW);
        digitalWrite(rightM, HIGH);

        voice.say(sp2_DANGER);
        voice.say(sp2_DANGER);
        voice.say(sp2_RIGHT);
        voice.say(sp2_ALERT);

        Serial.println("RIGHT BLOCKED - TURNING RIGHT");
        delay(1000);

        digitalWrite(leftM, LOW);
        digitalWrite(rightM, LOW);
      }
    }

    else
    {
      // No obstacle → Keep motors OFF (stick only vibrates when needed)
      digitalWrite(leftM, LOW);
      digitalWrite(rightM, LOW);
    }

    delay(10);
  }

  delay(10);
}



/* ------------------------- SEND SOS MESSAGE ------------------------- 
This function:
    1) Reads GPS for 1 second
    2) If location found → send Google Maps link via GSM
    3) If not → send plain text SOS
*/

void send_message()
{
  int count = 0;
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      Serial.print(c);
      count++;

      if (gps.encode(c))
        newData = true;

      if (count >= 10000) break;
    }
  }


  // If newData is true that is If GPS data received
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);

    Gsm.print("AT+CMGF=1\r");    // SMS text mode
    delay(400);
    Gsm.print("AT+CMGS=\"");
    Gsm.print(phone_no);
    Gsm.println("\"");
    delay(300);

    // Send Google Maps location
    Gsm.print("http://maps.google.com/maps?q=loc:");
    Gsm.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Gsm.print(",");
    Gsm.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    delay(200);

    Gsm.print(" PLEASE HELP ME");
    delay(200);

    Gsm.println((char)26);       // CTRL+Z to send SMS // End AT command with a ^Z, ASCII code 26
    delay(200);

    Gsm.println();
    delay(20000);
  }


  // If NO GPS data → Send basic SOS message
  else
  {
    Gsm.print("AT+CMGF=1\r");
    delay(400);
    Gsm.print("AT+CMGS=\"");
    Gsm.print(phone_no);
    Gsm.println("\"");
    delay(300);

    Gsm.print("PLEASE HELP ME");
    delay(200);

    Gsm.println((char)26);  // End AT command with a ^Z, ASCII code 26
    delay(200);

    Gsm.println();
    Serial.println("SENDING SOS WITHOUT GPS");
    delay(20000);
  }
}