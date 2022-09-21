//#include <NewPing.h>                //Include the NewPing library
//#define PING_PIN 8

#include <IRremote.h>

#define TRIG_PIN 8 // Connect TRIG pin to digital pin 4
#define ECHO_PIN 9
int sound = 11;
int led = 10;
// NewPing sonar(PING_PIN, PING_PIN ); //Create a NewPing object

int sensor = 7;
// int soundSensor=7;
int LEDd = 6;
boolean LEDStatus = false;

int redLed = 2;
int greenLed = 3;
int yellowLed = 13;
int buzzer = 4;
int smokeA0 = A0;
// Your threshold value
int sensorThres = 399;

const int RECV_PIN = 15;
IRrecv irrecv(RECV_PIN);
decode_results results;
const int redPin = 5;
const int yellowPin = 19;

void setup()
{
  irrecv.enableIRIn();
  // irrecv.blink13(true);
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT); // Set the TRIG pin to OUTPUT mode
  pinMode(ECHO_PIN, INPUT);  // Set the ECHO pin to INPUT mode
                            // pinMode(12,OUTPUT);
  pinMode(sensor, INPUT);
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(sound, OUTPUT);

  // pinMode(soundSensor,INPUT);
  pinMode(LEDd, OUTPUT);

  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);
}
void loop()
{

  if (irrecv.decode(&results))
  {

    switch (results.value)
    {
    case 0xFFA25D: // Keypad button "1"
      digitalWrite(redPin, HIGH);
      digitalWrite(yellowPin, LOW);
      digitalWrite(led, LOW);
      digitalWrite(sound, LOW);
    }
    switch (results.value)
    {
    case 0xFF629D: // Keypad button "2"
      digitalWrite(yellowPin, HIGH);
      //          delay(2000);
      //          digitalWrite(yellowPin, LOW);
      digitalWrite(redPin, LOW);
    }

    switch (results.value)
    {
    case 0xFF9867: // Keypad button "2"
      //          digitalWrite(yellowPin, HIGH);
      //          delay(2000);
      digitalWrite(yellowPin, LOW);
      digitalWrite(redPin, LOW);
    }

    irrecv.resume();
  }

  if (digitalRead(sensor) == HIGH)
  {
    digitalWrite(LEDd, LOW);

    delay(10);
  }
  else
  {

    digitalWrite(LEDd, HIGH);
    delay(10);
  }

  // obstacle detection
  // Send a short (10 microseconds) ultrasonic burst
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  float microseconds = pulseIn(ECHO_PIN, HIGH, 100000); // Mesure the duration of a HIGH pulse in echo pin in microseconds. Timeout in 0,1 seconds
  float seconds = microseconds / 1000000;               // Convert microseconds to seconds
  float meters = seconds * 343;                         // Get the distance in meters using the speed of sound (343m/s)
  float cm = meters * 100;                              // Convert meters to cm
  cm = cm / 2;                                          // We only want the distance to the obstacle and not the roundtrip
  // Serial.println(cm);                                   //Print distance in cm to the serial monitor
  delay(50); // Delay 50 milliseconds until the next mesurement
  if (cm <= 70)
  {
    digitalWrite(led, HIGH);
    if (cm < 30)
    {
      digitalWrite(sound, HIGH);
    }
  }
  else
  {
    digitalWrite(led, LOW);
    digitalWrite(sound, LOW);
  }

  //  int SensorData=digitalRead(soundSensor);
  //  if(SensorData==1){
  //
  //    if(LEDStatus==false){
  //        LEDStatus=true;
  //        digitalWrite(LEDd,HIGH);
  //    }
  //    else{
  //        LEDStatus=false;
  //        digitalWrite(LEDd,LOW);
  //    }
  //  }

  //  unsigned int uS = sonar.ping();
  // int cm = uS / US_ROUNDTRIP_CM;    //Get the distance in cm using the library
  // Serial.println(cm);               //Print the distance on the serial monitor
  // delay(50);                        //Delay 50 milliseconds for the next distance mesurement
  //
  // if(cm <= 100){
  //  digitalWrite(led,HIGH);
  //   if(cm< 30){
  //     digitalWrite(sound,HIGH);
  //   }
  //  }else{
  //   digitalWrite(led,LOW);
  //  digitalWrite(sound,LOW);
  // }

  // smoke detection
  int analogSensor = analogRead(smokeA0);

  Serial.print("Pin A0: ");
  Serial.println(analogSensor);
  //   Checks if it has reached the threshold value
  if (analogSensor >= 340 && analogSensor < 398)
  {
    digitalWrite(yellowLed, HIGH);
    digitalWrite(greenLed, LOW);
    digitalWrite(redLed, LOW);
    //    tone(buzzer, 1000, 200);
  }
  else if (analogSensor > sensorThres)
  {
    digitalWrite(yellowLed, LOW);
    digitalWrite(greenLed, LOW);
    digitalWrite(redLed, HIGH);
    tone(buzzer, 1000, 200);
  }
  else
  {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    digitalWrite(yellowLed, LOW);

    noTone(buzzer);
  }
  delay(100);
}
