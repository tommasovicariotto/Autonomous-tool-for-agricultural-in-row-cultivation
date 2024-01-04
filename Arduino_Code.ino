//Group6 MARS SS2022
//Updated 18/07/2022 
//Mech. tool for in row coltivation
//Arduino Code Respondable: Mirco Pozzoli, Tommaso Vicariotto

#include "SR04.h"                                   //Include Ultrasonic sensor library
#define TRIG_PIN 13                                 //Set pin 12 as trig 
#define ECHO_PIN 12                                 //Set pin 11 as echo
#include <Servo.h>                                  //Include servo library
#include <Wire.h>                                   // Include Arduino Wire library for I2C 
#define SLAVE_ADDR 9                                // Define Slave I2C Address
#define ANSWERSIZE 5                                // Define Slave answer size


  Servo Lifting_servo;                              //Declare servo object to control

const int Emergency_red_LED = 6;                    //On pin 6 red LED is connected
const int Emergency_yellow_LED = 5;                 //On pin 5 yellow LED is connected
const int Running_LED = 4;                          //On pin 5 green LED is connected
const int Safety_Switch = 3;                        //On pin 3 limit switch is conected
const int Relay = 7;                                //On pin 7 the relay is connected; it activate the DC motor
const int Home_servo =90;                           //Declare the starting position of the servo
int angle = 0;                                      //Variable needed to control the servo
int alert = 0;                                      //Variable needed to exit fromt the loop if the safety switch is pushed
unsigned long Running_Time;                         //Total count of seconds since the the board is powered on
int x=5;                                            //Initial default position of the robot is set to Stop

const int scratching_time = 1000;                   //%%SETUP INPUT%% time the rake stays on the soil while scratching [ms]
const int Stop_distance = 14.5;                     //%%SETUP INPUT%% cm distance to the soil to stop the arm lowering
unsigned long time_step = 5000;                     //%%SETUP INPUT%% time step between each lowering operation [ms]

SR04 Ultrasonic_S = SR04(ECHO_PIN,TRIG_PIN);        //Define the ultrasonic sensor
long distance;                                      //Define distance read by the sensor

void setup() 
  {
      Serial.begin(9600);                           //Initilize the serial communication with the computer
      Lifting_servo.attach(2);                      //Set pin 2 as servo control
      pinMode(Emergency_red_LED, OUTPUT);           //Set pin as output
      pinMode(Emergency_yellow_LED, OUTPUT);        //Set pin as output
      pinMode(Running_LED, OUTPUT);                 //set pin as output
      pinMode(Safety_Switch, INPUT);                //set pin as input
      pinMode(Relay, OUTPUT);                       //set pin as output
      Wire.begin();                                 // Initialize I2C communications as Master 
      Serial.println("I WRITE DATA AS MASTER");     //Testing your serial

      
  }

void loop() 
  {     
     Running_Time = millis();                                       //Total count of milliseconds since the power on of the board

                                                                    // Tell Arduino Robot to go forward 
      x=2;
      Wire.beginTransmission(SLAVE_ADDR); 
      Wire.write(x);
      Wire.endTransmission();
      Serial.println(x);                                            //Printing the I2C command on the Arduino Serial, for testing 
 
     Lifting_servo.write(Home_servo);                               //Start from home position
     digitalWrite(Relay, HIGH);                                     //Turn off the motor
                            
     int Safety_Switch_status = digitalRead(Safety_Switch);         // Read the status of the switch
          
     if (Safety_Switch_status == LOW)                               //If emergency switch pushed: turn on red led and stop everithing
        {
          digitalWrite(Running_LED, LOW);                                //Turn off the green LED
          digitalWrite(Emergency_red_LED, HIGH);                         //Turn on the red LED
          digitalWrite(Emergency_yellow_LED, HIGH);                      //Turn on the yellow LED
          digitalWrite(Relay, HIGH);                                     //Turn off the motor
          Lifting_servo.write(Home_servo);                               //Return to home position
        }
     else if (Safety_Switch_status == HIGH)                         //If emergency switch NOT pushed: turn on green led and proceed the operation
        {
          digitalWrite(Emergency_red_LED, LOW);                          //Turn off the red LED
          digitalWrite(Emergency_yellow_LED, LOW);                       //Turn off the yellow LED
          digitalWrite(Running_LED, HIGH);                               //Turn on the green LED

          Serial.print(Running_Time);                                    //Print the time passed since the board is powered on
          Serial.print("\n");                                       
         
          if (Running_Time % time_step <=20)                        //If a time frame circa equal to time step has passed
              {                                                        
                  digitalWrite(Relay, LOW);                              //Turn on the DC motor
                  distance = Ultrasonic_S.Distance();                    //Read distance from ultrasonic sensor
                  
                  for (angle=Home_servo; (distance < Stop_distance-1 or distance > Stop_distance+1) and (0<=angle<=180)and (alert==0);)  //When the distance arm-ground needs to be correct
                    {
                      distance = Ultrasonic_S.Distance();           //Read distance from ultrasonic sensor
                      Serial.print(distance);                       //Print on the monitor the distance
                      Serial.println("cm");                         //Print on the monitor the measurement unit
                      if (distance < Stop_distance-1)               //If is too close to the ground
                        {
                          Lifting_servo.write(angle);               //Servo is lifting the arm
                          angle++;
                        }
                      else if (distance > Stop_distance+1)          //If is too far from the ground
                        {
                          Lifting_servo.write(angle);               //Servo is lowering the arm
                          angle--;
                        }
                      
                      int Safety_Switch_status = digitalRead(Safety_Switch);     // Read the status of the switch
                      
                      if (Safety_Switch_status == LOW)                           //If emergency switch pushed: turn on red and yellow led and stop everithing
                        {
                         alert=1;
                        }
                    }
                  if (alert==1)
                    {
                      digitalWrite(Running_LED, LOW);           //Turn off the green LED
                      digitalWrite(Emergency_red_LED, HIGH);    //Turn on the red LED
                      digitalWrite(Emergency_yellow_LED, HIGH); //Turn on the yellow LED
                      digitalWrite(Relay, HIGH);                //Turn off the motor
                      Lifting_servo.write(Home_servo);          //Try to return to home position
                    }
                  if (alert==0)
                    {
                      delay(scratching_time);                   //Wait: the rake is scratching the soil
                      digitalWrite(Relay, HIGH);                //Turn off the motor
                      delay(5);
                      Lifting_servo.write(Home_servo);          //Lift the arm to home position
                    }
              }
          alert=0;
          Serial.flush();   
        }
  }
