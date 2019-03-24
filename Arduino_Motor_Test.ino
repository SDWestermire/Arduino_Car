
#include <Servo.h> 

/*MY NOTES;  The analog inputs can also be used as digital pins: analog input A0 as digital pin 14 through analog input 5 as digital pin 19.
 * Although I chose not to utilize this configuration it's worth noting :)

   Motor drive module                   Arduino UNO      Code Variable and pin Def
   IN1--------------------------------------A3-------------pinRF = 8
   IN2--------------------------------------A2-------------pinRB = 9
   IN3--------------------------------------A1-------------pinLF = 6
   IN4--------------------------------------A0-------------pinLB = 7

*/

//MOTOR VARIABLES
int pinLB=6;     // Define a motor pin for Left Motor on Pin 7
int pinLF=7;     // Define a motor pin for Left Motor on Pin 6

int pinRB=8;    // Define a 16 Pin which is Analog Pin A2
int pinRF=9;    // Define a 17 Pin which is Analog Pin A3

int MotorLPWM=5;  //Define a PWM to motor Enable Pin B on Pin 5
int MotorRPWM=10;  //Define a PWM to motor Enable Pin A on Pin 13

//ULTRASONIC VARIABLES

int inputPin = A1;      //Declare and initalize usbDistance to Pin A1 
int outputPin = A0;     //Declare and initialize outputPin on Pin A0 - send high/low pulse to HS R04 rangefinder

float speedOfSound = 770.5;   //speed of sound in MPH
float pingTime = 0.0;  //create var for echo translation
float targetForwardDistance = 0.0;  //create var for echo translation
float targetLeftDistance = 0.0;  //create var for echo translation
float targetRightDistance = 0.0;  //create var for echo translation


//GLOBAL AND LOCAL VARIABLES

int fwdTargetDistance = 0;      // Go forward -- Advance
int rightTargetDistance = 0;      // Turn to the right
int leftTargetDistance = 0;      // Turn to the left
int carDirection = 0;   // After FWD = 8; BKW = 2; LHT = 4; RHT = 6

int delay_time = 250; // After the servo motor to the stability of the time
int min_dist = 3;     // this is 3 inches returned by the ultrasound reading...not three cm!

// bool collision = False;  //set while loop variable as 'collision'...loop forever until there is a collision.


int FWD = 8;         // go FORWARD
int RHT = 6;         // The right to
int LHT = 4;         // Turn left to
int BKW = 2;         // BACKWARD

//SERVO VARIABLES

int  myServoPin = 4;  //initalize the servo pin to 3
Servo myServo;        // Set up the myservo

void setup()
 {
  Serial.begin(9600);     // Initialize 
  pinMode(pinLB,OUTPUT); // Define 7 pin for the output (PWM)
  pinMode(pinLF,OUTPUT); // Define 6 pin for the output (PWM)
  pinMode(pinRB,OUTPUT); // Define 11 pin for the output (PWM) 
  pinMode(pinRF,OUTPUT); // Define 12 pin for the output (PWM)
  
  pinMode(MotorLPWM,  OUTPUT);  // Define 5 pin for PWM output 
  pinMode(MotorRPWM,  OUTPUT);  // Define 13 pin for PWM output
  
  pinMode(inputPin, INPUT);    // Define the ultrasound enter pin
  pinMode(outputPin, OUTPUT);  // Define the ultrasound output pin   

  myServo.attach(myServoPin);    // Define the servo motor output 3 pin(PWM)
 }
void advance(int a)     // go Forward
    {
     digitalWrite(pinRB,HIGH);   // ...
     digitalWrite(pinRF,LOW);    // ...
     analogWrite(MotorRPWM,100); //Set the output speed(PWM)
     digitalWrite(pinLB,HIGH);  //  ...
     digitalWrite(pinLF,LOW);   // ...
     analogWrite(MotorLPWM,100);//Set the output speed(PWM)
     delay(a * 1);     
    }

void right(int b)        //right
    {
     digitalWrite(pinRB,LOW);   
     digitalWrite(pinRF,HIGH);
     analogWrite(MotorRPWM,50);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,LOW);
     delay(b * 100);
    }
void left(int c)         //left
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);   
     digitalWrite(pinLF,HIGH);
     analogWrite(MotorLPWM,50);
     delay(c * 100);
    }
void turnR(int d)        //right
    {
     digitalWrite(pinRB,HIGH);  
     digitalWrite(pinRF,LOW);
     analogWrite(MotorRPWM,50);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,HIGH);  
     analogWrite(MotorLPWM,50);
     delay(d * 50);
    }
void turnL(int e)        //left
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,HIGH);   
     analogWrite(MotorRPWM,220);
     digitalWrite(pinLB,HIGH);   
     digitalWrite(pinLF,LOW);
     analogWrite(MotorLPWM,220);
     delay(e * 50);
    }    
void stopp(int f)         //stop
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,LOW);
     delay(f * 100);
    }
void back(int g)          //back
    {

     digitalWrite(pinRB,LOW);  
     digitalWrite(pinRF,HIGH);
     analogWrite(MotorRPWM,150);
     digitalWrite(pinLB,LOW);  
     digitalWrite(pinLF,HIGH);
     analogWrite(MotorLPWM,150);
     delay(g * 1000);     
    }
    
void detection()        //Measuring three angles(5.92.170)
    {      
      int delay_time = 400;   // After the servo motor to the stability of the time
      ask_pin_F();            // Read in front of the distance
      
     if(fwdTargetDistance >0 && fwdTargetDistance < min_dist)         // If the front distance less than 5 inches
      {
      stopp(1);               // Remove the output data 
      back(2);                // Then back two milliseconds
      }
           
      if(fwdTargetDistance >= min_dist && fwdTargetDistance < 10)         // If the front distance 10 inches
      {
        stopp(1);               // Remove the output data
        ask_pin_L();            // Read the left distance
        delay(delay_time);      // Waiting for the servo motor is stable
        ask_pin_R();            // Read the right distance  
        delay(delay_time);      //  Waiting for the servo motor is stable  
        
        if(leftTargetDistance > rightTargetDistance)   //If the distance is greater than the right distance on the left
        {
         carDirection = LHT;      //Left Hand Turn = LHT
        }
        
        if(leftTargetDistance <= rightTargetDistance)   //If the distance is less than or equal to the distance on the right
        {
         carDirection = RHT;      //Right Hand Turn = RHT
        } 
        
        if (leftTargetDistance < 2.5 && rightTargetDistance < 2.5 )   /*If the left front distance and distance and the right distance is less than 15 cm */
        {

         carDirection = BKW;      //BKW = Backwards        
        }          
      }
      else                      //If the front is not less than 25 cm (greater than)    
      {
        carDirection = FWD;        //FWD = Go or Move forward    
      }
     
    }    
void ask_pin_F()   // Measure the distance ahead
        {

        myServo.write(90);   //Set myServo to the most forward looking position
        
        digitalWrite(outputPin, LOW);   //Set R04 sensor to low
        delayMicroseconds(2);           // Delay to settle lines
        digitalWrite(outputPin, HIGH);  //Send pulse 
        delayMicroseconds(10);            //pulse enroute!
        digitalWrite(outputPin, LOW);    //stop pulse
        delayMicroseconds(20);  //settle down pulse with this pause
        
         pingTime = pulseIn(inputPin, HIGH);  //pingTime is presented in microceconds
         pingTime=pingTime/1000000; //convert pingTime to seconds by dividing by 1000000 (microseconds in a second)
         pingTime=pingTime/3600; //convert pingtime to hourse by dividing by 3600 (seconds in an hour)
         targetForwardDistance= speedOfSound * pingTime;  //This will be in miles, since speed of sound was miles per hour
         targetForwardDistance=targetForwardDistance/2; //Remember ping travels to target and back from target, so you must divide by 2 for actual target distance.
         targetForwardDistance= targetForwardDistance*63360;    //Convert miles to inches by multipling by 63360 (inches per mile)
          
         Serial.print("The Distance to FRONT Target is: ");
         Serial.print(targetForwardDistance);
         Serial.println(" inches");
         fwdTargetDistance = targetForwardDistance;
          
        }

    
 void ask_pin_L()   // Measure the distance on the left 
    {
      myServo.write(180);

      digitalWrite(outputPin, LOW);   //Set R04 sensor to low
      delayMicroseconds(2);  
      digitalWrite(outputPin, HIGH);  //Send pulse 
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);
      delayMicroseconds(20);  //settle down pulse with this pause
    
      // pingTime = 0.;
      pingTime = pulseIn(inputPin, HIGH);  //pingTime is presented in microceconds
      //Serial.print("MINE: ");
      //Serial.print(pingTime, 10);
      //Serial.println();
      
      pingTime=pingTime/1000000.0; //convert pingTime to seconds by dividing by 1000000 (microseconds in a second)
      //Serial.print("pingTime div 1mil :");
      //Serial.print(pingTime, 10);
      //Serial.println();
      
      pingTime=pingTime/3600; //convert pingtime to hourse by dividing by 3600 (seconds in an hour)
      targetLeftDistance= speedOfSound * pingTime;  //This will be in miles, since speed of sound was miles per hour
      targetLeftDistance=targetLeftDistance/2; //Remember ping travels to target and back from target, so you must divide by 2 for actual target distance.
      targetLeftDistance= targetLeftDistance*63360;    //Convert miles to inches by multipling by 63360 (inches per mile)
      
      Serial.print("The Distance to LEFT Target is: ");
      Serial.print(targetLeftDistance);
      Serial.println(" inches");

      leftTargetDistance = targetLeftDistance;              // Will reading leftTargetDistance distance
    }  
void ask_pin_R()   // Measure the distance on the right 
    {
      myServo.write(0);

      digitalWrite(outputPin, LOW);   //Set R04 sensor to low
      delayMicroseconds(2);  
      digitalWrite(outputPin, HIGH);  //Send pulse 
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);
      delayMicroseconds(20);  //settle down pulse with this pause

      
      pingTime = pulseIn(inputPin, HIGH);  //pingTime is presented in microceconds
      pingTime=pingTime/1000000; //convert pingTime to seconds by dividing by 1000000 (microseconds in a second)
      pingTime=pingTime/3600; //convert pingtime to hourse by dividing by 3600 (seconds in an hour)
      targetRightDistance = speedOfSound * pingTime;  //This will be in miles, since speed of sound was miles per hour
      targetRightDistance = targetRightDistance/2; //Remember ping travels to target and back from target, so you must divide by 2 for actual target distance.
      targetRightDistance = targetRightDistance*63360;    //Convert miles to inches by multipling by 63360 (inches per mile)
        
      Serial.print("The Distance to RIGHT Target is: ");
      Serial.print(targetRightDistance);
      Serial.println(" inches");


      rightTargetDistance = targetRightDistance;              
    }  
    
void loop()
 {
    myServo.write(90);  /*Make the servo motor ready position Prepare the next measurement 
    Serial.println("Starting Main Loop...");
    Serial.println("Set Servo to 90....");
    Serial.println("  executing detection function!");*/
    detection();        //Measuring Angle And determine which direction to go to
        
     if(carDirection == 2)  //If carDirection (direction) = 2 (back)          
     {
       back(8);                    //  back
       turnL(2);                   //Move slightly to the left (to prevent stuck in dead end lane)
       Serial.println(" Reversing Direction ");   //According to the direction (reverse)
     }
     if(carDirection == 6)           //If carDirection (direction) = 6 (right)   
     {
       back(1); 
       turnR(6);                   // right
       Serial.println(" Right Turn ");    //According to the direction (Right)
     }
     if(carDirection == 4)          //If carDirection (direction) = 4 (left)   
     {  
       back(1);      
       turnL(6);                  // left
       Serial.println(" Left Turn ");     //According to the direction (Left)  
     }  
     if(carDirection == 8)          //If carDirection (direction) = 8 (forward)      
     { 
      advance(1);                 // go 
      Serial.print(" Advancing Forward ");  //According to the direction (Advance)
      Serial.print(" ---  ");    
   }
 }
