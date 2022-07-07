//#include <NewPing.h>                //Include the NewPing library
//#define PING_PIN 8    

#define TRIG_PIN 8            //Connect TRIG pin to digital pin 4
#define ECHO_PIN 12
int sound =11;
int led =10;
//NewPing sonar(PING_PIN, PING_PIN ); //Create a NewPing object

int soundSensor=7;
int LEDd=6;
boolean LEDStatus=false;


int redLed = 2;
int greenLed = 3;
int buzzer = 4;
int smokeA0 = 15;
// Your threshold value
int sensorThres = 150;


void setup()
{
   pinMode(TRIG_PIN, OUTPUT);  //Set the TRIG pin to OUTPUT mode
 pinMode(ECHO_PIN, INPUT);   //Set the ECHO pin to INPUT mode
  //pinMode(12,OUTPUT);
 // pinMode(7,INPUT);
  Serial.begin(9600);
 pinMode(led,OUTPUT);
 pinMode(sound,OUTPUT);

  pinMode(soundSensor,INPUT);
 pinMode(LEDd,OUTPUT);

   pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);

}
void loop()
{
//  if (digitalRead(7)== LOW)
//  {
//    digitalWrite(12,HIGH);
//    
//    delay(10);
//  }
//  else 
//  {
//    
//    digitalWrite(12,LOW);
//    delay(10);
//    
//  }


  int SensorData=digitalRead(soundSensor);
  if(SensorData==1){

    if(LEDStatus==false){
        LEDStatus=true;
        digitalWrite(LEDd,HIGH);
    }
    else{
        LEDStatus=false;
        digitalWrite(LEDd,LOW);
    }
  }

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


//Send a short (10 microseconds) ultrasonic burst 
 digitalWrite(TRIG_PIN, HIGH); 
 delayMicroseconds(10);        
 digitalWrite(TRIG_PIN, LOW);
 float microseconds = pulseIn(ECHO_PIN, HIGH, 100000); //Mesure the duration of a HIGH pulse in echo pin in microseconds. Timeout in 0,1 seconds
 float seconds = microseconds / 1000000;               //Convert microseconds to seconds
 float meters = seconds * 343;                         //Get the distance in meters using the speed of sound (343m/s)
 float cm = meters * 100;                              //Convert meters to cm
 cm = cm/2;                                            //We only want the distance to the obstacle and not the roundtrip
// Serial.println(cm);                                   //Print distance in cm to the serial monitor
 delay(50);                                            //Delay 50 milliseconds until the next mesurement
 if(cm <= 100){
  digitalWrite(led,HIGH);
   if(cm< 30){
     digitalWrite(sound,HIGH);
   }
  }else{
   digitalWrite(led,LOW);
  digitalWrite(sound,LOW);
 }
 
 int analogSensor = analogRead(smokeA0);

  Serial.print("Pin A0: ");
  Serial.println(analogSensor);
  // Checks if it has reached the threshold value
  if (analogSensor > sensorThres)
  {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    tone(buzzer, 1000, 200);
  }
  else
  {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    noTone(buzzer);
  }
  delay(100);
}
