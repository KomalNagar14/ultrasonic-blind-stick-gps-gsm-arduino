/* blind_stick_basic.ino
   BASIC VERSION OF BLIND STICK
   Features:
   - Obstacle detection (front, left, right)
   - Water detection
   - Voice alerts using Talkie library
   - Emergency button + GPS + GSM SMS alert
*/

#include <TinyGPS.h>               // Library to read GPS data (latitude, longitude)
#include <SoftwareSerial.h>        // To create serial communication on any digital pins
#include <Arduino.h>

#include "Talkie.h"                // Speech synthesis library
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include "Vocab_US_TI99.h"

// GSM module connected using SoftwareSerial
#define GSM_RX_PIN A3
#define GSM_TX_PIN A4
SoftwareSerial Gsm(GSM_RX_PIN, GSM_TX_PIN);

char phone_no[] = "+91XXXXXXXXXX"; // Phone number to receive emergency SMS

TinyGPS gps;       // To decode GPS NMEA data
Talkie voice;      // Speech object for voice alerts

// ---------- ULTRASONIC SENSOR PINS ----------
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
const int buzzer_pin = 13;     // Buzzer for sound alert
const int motor_pin = 10;      // Vibration motor for haptic alert

// ---------- WATER + EMERGENCY ----------
#define Water_Sensor_Pin A0    // Water detection sensor

// Emergency SOS button (active LOW)
const int switch_pin = A1;     // Emergency button (press to send SMS)
int State = 0;                 // Stores emergency button status


/* ------------------------- SETUP ------------------------- */
void setup()
{
  Serial.begin(9600);          // Debug output
  Gsm.begin(57600);            // GSM module baud rate

  // Setup ultrasonic sensor pins
  pinMode(trigger_front, OUTPUT);
  pinMode(echo_front, INPUT);

  pinMode(trigger_left, OUTPUT);
  pinMode(echo_left, INPUT);

  pinMode(trigger_right, OUTPUT);
  pinMode(echo_right, INPUT);

  // Water sensor + alert devices
  pinMode(Water_Sensor_Pin, INPUT);
  pinMode(buzzer_pin, OUTPUT);   // Buzzer output
  pinMode(motor_pin, OUTPUT);

  // Turn everything OFF initially
  digitalWrite(buzzer_pin, LOW);
  digitalWrite(motor_pin, LOW);
  
  // Emergency switch
  pinMode(switch_pin, INPUT_PULLUP);   // Internal pullup -> button pressed = LOW

  delay(1000);
}


// ------------------ ULTRASONIC READ FUNCTIONS ------------------
// Sends pulse -> reads echo -> converts to cm distance

void check_distance_front()
{
  long dis;
  digitalWrite(trigger_front, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_front, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger_front, LOW);

  dis = pulseIn(echo_front, HIGH);
  distance_front = dis / 29 / 2; // Speed of sound conversion to cm
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


// ------------------------ MAIN LOOP ------------------------

void loop()
{
  check_distance_front();
  check_distance_left();
  check_distance_right();

  // -------- WATER DETECTION --------
  // Water sensor becomes HIGH when water is detected
  if (digitalRead(Water_Sensor_Pin) == HIGH)
  {
    digitalWrite(motor_pin, HIGH);
    digitalWrite(buzzer_pin, HIGH);
    Serial.println("WATER DETECTED");
    delay(2000);
  }
  else
  {
    digitalWrite(motor_pin, LOW);
    digitalWrite(buzzer_pin, LOW);
  }

  // -------- EMERGENCY BUTTON --------
  State = digitalRead(switch_pin);

  if (State == LOW) // Button pressed
  {
    voice.say(sp2_DANGER);
    voice.say(sp2_DANGER);
    voice.say(spt_STOP);
    voice.say(sp2_ALERT);

    Serial.println("HELP - EMERGENCY PRESSED");

    delay(1000);
    send_message();    // Sends GPS SMS
    delay(1000);
  }

  // -------- OBSTACLE ALERTS --------
  else if (distance_front <= 80)
  {
    voice.say(sp2_DANGER);
    voice.say(sp2_DANGER);
    voice.say(spt_FRONT);
    voice.say(sp2_ALERT);

    // digitalWrite( motor,HIGH);
    //digitalWrite(Buzzer,HIGH);

    Serial.println("Obstacle FRONT");
    delay(1000);
  }

  else if (distance_left <= 80)
  {
    voice.say(sp2_DANGER);
    voice.say(sp2_DANGER);
    voice.say(sp2_LEFT);
    voice.say(sp2_ALERT);

    Serial.println("Obstacle LEFT");
    delay(1000);
  }

  else if (distance_right <= 80)
  {
    voice.say(sp2_DANGER);
    voice.say(sp2_DANGER);
    voice.say(sp2_RIGHT);
    voice.say(sp2_ALERT);

    Serial.println("Obstacle RIGHT");
    delay(1000);
  }

  // -------- NO PROBLEM AREA --------
  else
  {
    digitalWrite(motor_pin, LOW);
    digitalWrite(buzzer_pin, LOW);
  }

  delay(100);
}


// ------------------------ SEND EMERGENCY SMS ------------------------

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

      if (gps.encode(c))       // If valid GPS packet is received
        newData = true;
      if (count >= 10000)      // Safety break
        break;
    }
  }

  // ---------- If GPS data available ----------
  if (newData)
  {
    float flat, flon;
    unsigned long age;

    gps.f_get_position(&flat, &flon, &age);

    // Configure GSM SMS mode
    Gsm.print("AT+CMGF=1\r");
    delay(400);

    Gsm.print("AT+CMGS=\"");
    Gsm.print(phone_no);
    Gsm.println("\"");
    delay(300);

    // Google Maps live location link
    Gsm.print("http://maps.google.com/maps?q=loc:");
    Gsm.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Gsm.print(",");
    Gsm.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

    Gsm.print("  PLEASE HELP ME");
    delay(200);

    // End SMS with Ctrl+Z
    Gsm.println((char)26); // End AT command with a ^Z, ASCII code 26
    delay(200);
    Gsm.println("\"");
    delay(300);
    Gsm.println();
    delay(20000);

  }

  // ---------- If NO GPS data found ----------
  else
  {
    Gsm.print("AT+CMGF=1\r");
    delay(400);

    Gsm.print("AT+CMGS=\"");
    Gsm.print(phone_no);
    Gsm.println("\"");
    delay(300);

    Gsm.print("PLEASE HELP ME");  // No coordinates sent
    delay(200);

    Gsm.println((char)26); // End AT command with a ^Z, ASCII code 26
    delay(200);
    Gsm.println();

    Serial.println("SENDING SOS WITHOUT GPS");
    delay(20000);
  }
}