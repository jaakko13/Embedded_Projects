#include <LiquidCrystal.h>
  //LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
  LiquidCrystal lcd(12, 11, 5, 4, 25, 26); //changed wires to make space for encoder

/////////////////////////
// Compass Definitions //
/////////////////////////

#include <TimerOne.h>
#include <Wire.h>

#define CMPS14_address 0x60

/////////////////////////
//  Motor Definitions  //
/////////////////////////

#define R_count_inter_pin 3   // R encoder pulse counting interrupt pin
#define L_count_inter_pin 2

#define direction_R 23        //encoder direction pins
#define direction_L 24

//switched the pwm and direction pins for left & right motors..
#define PWM_Motor_R 9 
#define Dir_Motor_R 7
#define PWM_Motor_L 10
#define Dir_Motor_L 8

#define forward 1
#define backward 0

//pin 10 is for left motor

volatile int R_pulse = 0, L_pulse = 0;
volatile byte count = 0;
int truePulses = 0;


void Joystick(int& x_val, int& y_val); //returns x_val & y_val by reference. -255 to 255 can be used by DriveDistance()
void DriveDistance(int distL, int distR); //Moves left motor distL pulses, & right motor distR pulses
void HelpCommand(); //prints to serial the commands that can be performed
void JoystickController(); //function to control joystick, ends when user types "exit"
bool TurnTo(int desiredDegree); //turns car to the degree user wants, returns false if error/couldn't accomplish
bool turnRight(int desiredDegree); //takes the desired degree and tells if car should turn right (true) or left (false), which path is easier
void DegreeOnLCD(); //just prints degree on LCD, might not be used
int compass_val(); //reads raw compass value, translates it from 0 - 360 and returns that int
void TurnAmount(int turnDegrees); //tells car to turn "turnDegrees" amount from current position. negative = left, pos = right
void encoderVroom(int amount);//moves car fixed distance
void pulsing_R();
void pulsing_L();


void setup() {
   Serial.begin(9600); // open the serial port at 9600 bps:
  Serial.println("+----------------------------------+");
  Serial.println("|         Software Americans       |");
  Serial.println("+----------------------------------+"); 
  
  lcd.begin(20,4);

  Wire.begin(); //Wire library

  pinMode(R_count_inter_pin, INPUT);
  pinMode(L_count_inter_pin, INPUT);

  pinMode(direction_R,INPUT);    
  digitalWrite(direction_R,HIGH); 

  pinMode(direction_L,INPUT);    
  digitalWrite(direction_L,HIGH);

  attachInterrupt(digitalPinToInterrupt(R_count_inter_pin), pulsing_R, RISING);
  attachInterrupt(digitalPinToInterrupt(L_count_inter_pin), pulsing_L, RISING); 

  Serial.println("Enter a command into the Serial monitor to move the car!");
  Serial.println("Enter \"help\" or \"help:\" for details.");
}

void loop() {
int userInp;
    String command;

   
    if(Serial.available() > 0) {
      command  = Serial.readStringUntil(':');
      userInp = Serial.parseInt();
      command.toUpperCase(); //everything capital for standard
      Serial.read(); //flush the '\n' buffer

      if(command == "MOVE") {
        Serial.println("MOVE == MOVE");
        //DriveDistance(userInp, -userInp);
        encoderVroom(userInp);
      }
      else if(command == "TURN") {
        Serial.println("TURN == TURN");
        TurnAmount(userInp);
      }
      else if(command == "JOYSTICK" || command == "JOYSTICK\n") {
        Serial.println("JOYSTICK == JOYSTICK");
        JoystickController();
      }
      else if(command == "LCD" || command == "LCD\n"){
        lcd.print("Hello. The LCD works. I promise.");
        delay(5000);
        }
      else if(command == "HELP" || command == "HELP\n") {HelpCommand();}
      else {
        Serial.print("ERROR: \""); Serial.print(command); Serial.println("\" is not a valis command.");
        Serial.println("Enter: \"MOVE\", \"TURN\" or \"HELP\"");
      }
    }

    delay(100);
    
    lcd.clear();
}


//encoder code made to move fixed distance.
  void encoderVroom(int amount){


      int inInt, distL, distR;
      inInt = amount;
      distL = amount; 
      distR = amount;
      
      if (distL > 0) digitalWrite(Dir_Motor_L, forward); 
      else digitalWrite(Dir_Motor_L, backward);
      if (distR > 0) digitalWrite(Dir_Motor_R, backward); 
      else digitalWrite(Dir_Motor_R, forward);

      
      if(truePulses == 0){
        //inInt = Serial.parseInt();
        truePulses = inInt * 12;
      }

      




      while(L_pulse < truePulses){
        analogWrite(PWM_Motor_L, 100); //Left thing goes vroom vroom.
        analogWrite(PWM_Motor_R, 100); //Right thing goes vroom vroom.
        //Serial.println(R_pulse);
        }


        analogWrite(PWM_Motor_L, 0); //Left thing doesnt go vroom vroom. 
        analogWrite(PWM_Motor_R, 0); //Right thing doesnt go vroom vroom.
     /*
      if(R_pulse < truePulses){
        analogWrite(PWM_Motor_L, 100); //Left thing goes vroom vroom.
        analogWrite(PWM_Motor_R, -100); //Right thing goes vroom vroom.

      }
      if(R_pulse > truePulses){
        analogWrite(PWM_Motor_L, 0); //Left thing doesnt go vroom vroom. 
        analogWrite(PWM_Motor_R, 0); //Right thing doesnt go vroom vroom.
        }
*/

       
        truePulses = 0;
  }


bool TurnTo(int desiredDegree) {
  //read_compass_raw()
  int difference = desiredDegree - compass_val();
  for(int i = 0; difference > 5 || difference < -5; i++) {
    //accomplish turning here
    //
    //DegreeOnLCD();
    if(turnRight(desiredDegree)) {
      //left motor -> forward & right motor -> reverse
      Serial.println("turning right... ");
      DriveDistance(5, 5); //because a motor isn't working, the broken motor may have to have its driveDistance = 0
    }
    else {
      //left motor -> reverse & right motor -> forward
      Serial.println("turning left... ");
      DriveDistance(-5, -5); //this might be opposite
    }
    if(i > 180) {
      //error taking too long
      Serial.println("error in TurnTo()...");
      return false;
    }
    difference = desiredDegree - compass_val();
  }
  for(int i = 0; difference > 2 || difference < -2; i++) {
    //DegreeOnLCD();
    if(turnRight(desiredDegree)) {
      //left motor -> forward & right motor -> reverse
      Serial.println("slightly turning right... ");
      DriveDistance(1, 1); //because a motor isn't working, the broken motor may have to have its driveDistance = 0
    }
    else {
      //left motor -> reverse & right motor -> forward
      Serial.println("slightly turning left... ");
      DriveDistance(-1, -1); //this might be opposite
    }
    if(i > 180) {
      //error taking too long
      Serial.println("error in TurnTo()...");
      return false;
    }
    difference = desiredDegree - compass_val();
  }
  return true;
}

//determines if car should rotate left or right to achieve desired degree
bool turnRight(int desiredDegree) {
  int currentDegree = compass_val();
  int difference = desiredDegree - currentDegree;

  if(difference == 0) {
    //error: send an error message car is already alligned
    
    return false;
  }
      
  if(abs(difference) > 180 && difference > 0) {
    //desiredDegree is ahead in degrees and should turn left
    return false;
  }
  else if(abs(difference) > 180 && difference < 0) {
    //currentDegree is ahead in degrees and should turn right
    return true;
  }
  else if(abs(difference) < 180 && difference > 0) {
    //desiredDegree is ahead in degrees and should turn right
    return true;
  }
  else if(abs(difference) < 180 && difference < 0) {
    //currentDegree is ahead in degrees and should turn left
    return false;
  }
  else {
    //error: send error message with difference, desiredDegree, & currentDegree\\
    return false;
  }
}

void pulsing_R(void) {
  if(digitalRead(direction_R) == 0) { R_pulse++; } else R_pulse--;//had to change when it adds to it and when it lowers it for R_pulse to count correctly. Always went down with encoderVroom
  lcd.setCursor(0, 0);                                             // originally -- then ++
  lcd.print("pulsing_R"); 
  //delay(400);
  //Serial.println("pulsing_R");
}

void pulsing_L(void) {
  if(digitalRead(direction_L) == 0) { L_pulse++; } else L_pulse--;//had to change when it adds to it and when it lowers it for L_pulse to count correctly. Always went down with encoderVroom
  lcd.setCursor(0, 1);                                            //originally -- then ++
  lcd.print("pulsing_L");
  //delay(400);
  //Serial.println("pulsing_L");
}

//rotate joystick 45 degrees and it will work
void Joystick(int& x_val, int& y_val) {
  float sensorValueX = analogRead(A8); //A8
  float sensorValueY = analogRead(A9); //A9

  x_val = ((sensorValueX - 491.0) / 532.0) * 255; //adjusted for joystick calibration
  y_val = ((sensorValueY - 512.0) / 511.0) * 255;

  //returns range from -255 to 255
}

//function that accepts int distance for each motor and performs the task
void DriveDistance(int distL, int distR) {
  // distance units in this function are PULSES
  R_pulse=0;
  L_pulse=0;

  //determine forwards or reverse for each motor
  if (distL > 0) digitalWrite(Dir_Motor_L, forward); 
  else digitalWrite(Dir_Motor_L, backward);
  if (distR > 0) digitalWrite(Dir_Motor_R, forward); 
  else digitalWrite(Dir_Motor_R, backward);

  analogWrite( PWM_Motor_L, 69);
  analogWrite( PWM_Motor_R, 69);

  byte bk_L=1;
  byte bk_R=1;
  
  while(bk_R != 0 && bk_L != 0) {
   if (abs(L_pulse)>abs(distL)) {    
        analogWrite( PWM_Motor_L, 0);  // set PWM value M2PWM  pin 10
        bk_L = 0; 
        Serial.print(" Stop L = "); Serial.println(L_pulse);  
        
     }

    if (abs(R_pulse)>abs(distR)) {    
        analogWrite( PWM_Motor_R, 0);  // set PWM value M2PWM  pin 10
        bk_R = 0;
        Serial.print(" Stop R = "); Serial.println(R_pulse); 
     }

  }
}

//reads the raw compass value & converts it from int to degrees
int compass_val()
{
 
  Wire.beginTransmission( (uint8_t) CMPS14_address);
  Wire.write( (uint8_t) 1);
  Wire.endTransmission(false);

  Wire.requestFrom( (uint8_t) CMPS14_address,(uint8_t) 1, (uint8_t) true);

  if ( Wire.available() >=1 )
  {
    byte OrigCompassVal = Wire.read();//compass value between 0 - 255
  
    int TrueCompassVal = OrigCompassVal; //cast the orig compass value into a long int
  
    TrueCompassVal = TrueCompassVal * 360/256; //calculation to get the actual degrees

    Serial.print("raw compass value: ");
    Serial.println(TrueCompassVal);
  
    return TrueCompassVal; //final value to get the compass value between 0 - 360
  };
}

void DegreeOnLCD() {
  lcd.clear();
  lcd.setCursor(0, 2);
  lcd.print("current degree blw"); 
  lcd.setCursor(0,3);
  int i_compassValue;
  i_compassValue = compass_val();
  lcd.print(i_compassValue);
  delay(400);
}

///probably still some bugs but IS WORKING
void TurnAmount(int turnDegrees) {
  Serial.print("TurnAmount(int turnDegrees = ");
  Serial.println(turnDegrees);
  if(turnDegrees == 0) return true;
  
  int currentDegree = compass_val();
  Serial.print("currentDegree = ");
  Serial.println(currentDegree);
  int desiredDegree = currentDegree + turnDegrees;
  int low, high;

  low = 0;
  high = 0;
  
  low = desiredDegree - 5;
  high = desiredDegree + 5;

  int amountOf360s, baseDesiredDegree; //variables for if > =-360 
   
  if(desiredDegree < 0) { baseDesiredDegree = desiredDegree + (amountOf360s * 360); } // i.e. 750 degrees = 30 degrees
  else { baseDesiredDegree = desiredDegree - (amountOf360s * 360); } // i.e. 750 degrees = 30 degrees

  //for if the desired degree > 360
  if(abs(desiredDegree) > 360 && turnDegrees > 0) {
    //turning right a whole 360 i amount of times...
    amountOf360s = abs(desiredDegree) % 360;
    
    for(int i = 0; i < amountOf360s; i++) {
      DriveDistance(10, 10); //initialize it set to not eual
      while(compass_val() != currentDegree || compass_val() != (currentDegree + 1) || compass_val() != (currentDegree - 1)) 
      { DriveDistance(1, 1); } //keep turning right until a full 360 is accomplished
      if(desiredDegree > 360) { desiredDegree = desiredDegree - 360; }
      else if(desiredDegree < -360) {desiredDegree = desiredDegree + 360;}      
    }
  }
  else if(abs(desiredDegree) > 360 && turnDegrees < 0) {
    //turning left a whole 360 i amount of times
    amountOf360s = abs(desiredDegree) % 360;
    
    for(int i = 0; i < amountOf360s; i++) {
      DriveDistance(-10, -10); //initialize it set to not eual
      while(compass_val() != currentDegree || compass_val() != (currentDegree + 1) || compass_val() != (currentDegree - 1)) 
      { DriveDistance(-1, -1); } //keep turning right until a full 360 is accomplished
      if(desiredDegree > 360) { desiredDegree = desiredDegree - 360; }
      else if(desiredDegree < -360) {desiredDegree = desiredDegree + 360;}      
    }
  }

  
  Serial.println("at part when everything is < 360");
  //this part has all degrees < 360
  
  if(turnDegrees > 0) {
    //turn right
    if(baseDesiredDegree < 0) baseDesiredDegree += 360;
    Serial.println("turn right statement");
    //difference > 2 || difference < -2
    for(int i = 0; (currentDegree - desiredDegree) < 4 || (currentDegree - desiredDegree) < -4 ; i++) {
      DriveDistance(1, 1);
      //Jaakko replace this ^
      currentDegree = compass_val();
      Serial.print("currentDegree = ");
      Serial.println(currentDegree);
      Serial.print("baseDesiredDegree = ");
      Serial.println(baseDesiredDegree);
      if(i > 360) { Serial.println("error in turning right to amount"); return false; } //error taking way too long to reach
    } 
    Serial.println("########################################");
    Serial.println("########################################");
    Serial.println("########################################");
    if(desiredDegree > low && desiredDegree < high) {
      //DriveDistance(0, 0);
      Serial.println("Returning true");/////////////////////////////////// to stop it 
      analogWrite(PWM_Motor_L, 0); //Left thing doesnt go vroom vroom. 
      analogWrite(PWM_Motor_R, 0); //Right thing doesnt go vroom vroom.
      return true;
    }
    //DriveDistance(0, 0);
    Serial.println("What're you doing here");
    
  }
  else if(turnDegrees < 0) {
    //turn left
    Serial.println("turn left statement");
    if(baseDesiredDegree < 0) baseDesiredDegree += 360;
    for(int i = 0; (currentDegree - desiredDegree) < 4 || (currentDegree - desiredDegree) < -4 ; i++) {
      DriveDistance(-1, -1);
      //Jaakko replace this ^
      currentDegree = compass_val();
      Serial.print("currentDegree = ");
      Serial.println(currentDegree);
      Serial.print("baseDesiredDegree = ");
      Serial.println(baseDesiredDegree);
      if(i > 360) { Serial.println("error in turning left to amount"); return false; }//error taking way too long to reach 
    } 
    Serial.println("########################################");
    Serial.println("########################################");
    Serial.println("########################################");
    if(desiredDegree > low && desiredDegree < high) {
      //DriveDistance(0, 0);
      Serial.println("Returning true");/////////////////////////////////// to stop it
      analogWrite(PWM_Motor_L, 0); //Left thing doesnt go vroom vroom. 
      analogWrite(PWM_Motor_R, 0); //Right thing doesnt go vroom vroom.
      return true; //analog write 0 to stop it do also for below
    }
    //DriveDistance(0, 0);
    Serial.println("What're you doing here");
  }
  else {
    //already at zero, techinically it succeeded?
    return false; //temp
  }
  return false;
}


void JoystickController() {
  int pw1L = 0, pw2R = 0;
  String command = "";
  Serial.println("Car is controllable by joystick!");
  Serial.println("Enter \"stop\" to stop controlling by joystick.");

  while(command != "STOP") {
    Joystick(pw1L, pw2R);
    Serial.print(pw1L); Serial.print("\t"); Serial.println(pw2R);
    
    if (pw1L > 0) digitalWrite(Dir_Motor_L, forward); 
    else digitalWrite(Dir_Motor_L, backward);
    if (pw2R > 0) digitalWrite(Dir_Motor_R, forward); 
    else digitalWrite(Dir_Motor_R, backward);
  
    analogWrite( PWM_Motor_L, pw1L);
    analogWrite( PWM_Motor_R, pw2R);
    
    if(Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      command.toUpperCase(); //everything capital for standard
    }
    
    Serial.println("seven");
    delay(100);
    
  }
   Serial.println("Leaving Joystick.");
   
   analogWrite( PWM_Motor_L, 0);
   analogWrite( PWM_Motor_R, 0);
}

void HelpCommand() {
  Serial.println("Use commands to move the car!");
  Serial.println("Enter MOVE or TURN, followed by a ':' (colon), then an integer.");
  Serial.println("   i.e. \"MOVE:10\" to move the car by 10 units.");
  Serial.println("   i.e. \"TURN:-50\" to turn the car left by 50 degrees.");
}
