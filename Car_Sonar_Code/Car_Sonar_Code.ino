/* Search Algorithm

Side Sonar Search Algorithm
-As we move forward, compare left and right
-If there's a change greater than some threshold (25cm or so)
-Slow down, and continue checking this reading for a certain period of time to ensure that it was not corrupt data
-Check the back sonar and map data to ensure that at this distance, the ramp isn't what we're seeing
-If this is not where the ramp is, and the threshold is still met, then the base must have been found
-Rotate and move towards the base

Front Sonar Search Algorithm
-Keep moving forward, comparing previous distance seen to future distance seen
-If the sensors aren't seeing anything, and then they begin seeing something, compare with back sonar to 
 understand what distance they should be seeing, from here, determine if it's the base or not to verify measurement
-If seeing something through the whole drive, and the there is some jump (using some threshold) in the distance seen,
 once again confirm with the back sonar what distance they should be seeing at this time
 
 Functions()
 checkSideSonars()
 checkFrontSonars()
 verifyData(sonarID) //Change detected for sonar with ID sonarID, slow car down and verify this measurement
 sonarGetDistance (sonarID) //Get distance of sonarID
 
 initializeTracking() //Center the car so that one or both side sonars sees 150cm (or whatever they should see to be equal)
 

 */

///////////////////////////////////////////////////////////////
/////////             Start of Program           //////////////
///////////////////////////////////////////////////////////////
#include <Servo.h>
#include <NewPing.h>

//Define the Trigger Pins for the Sonar, Assuming the Echo Pin is (Trigger_Pin + 1) [i.e. Echo for Front Right Sonar = sonarFR + 1]
#define sonarFL 0  //Front Left Sonar, 0th index with regards to the sonar[] array, pins 30 and 31
#define sonarFR 1  //Front Right Sonar, 1st index with regards to the sonar[] array, pins 32 and 33
#define sonarSL 2  //Side Left Sonar, 2nd index with regards to the sonar[] array, pins 34 and 35
#define sonarSR 3  //Side Right Sonar, 3rd index with regards to the sonar[] array, pins 36 and 37
#define sonarB  4   //Back Sonar, 4th index with regards to the sonar[] array, pins 38 and 39

#define SONAR_NUM     5 // Number or sensors.
#define MAX_DISTANCE 250 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define SOBERSTATE_0 0 //Go Fwd, Straight
#define SOBERSTATE_1 1 //Full Turn90 Left
#define SOBERSTATE_2 2 //Full Turn90 Right
#define SOBERSTATE_3 3 //Slight Correction to the Left Has Started
#define SOBERSTATE_4 4 //Finishing the Slight Correct Left by Turning Right to Straighten out
#define SOBERSTATE_5 5 //Slight Correction to the Right Has Started
#define SOBERSTATE_6 6 //Finishing the Slight Correct Right by Turning Left to Straighten out


int TurnLEFT = 1;
int TurnRIGHT = 2;
int TurnCOMPLETE = 3;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int readSonars[SONAR_NUM];         // Where the ping distances are stored.
unsigned int sonarVal[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(30, 31, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(32, 33, MAX_DISTANCE),
  NewPing(34, 35, MAX_DISTANCE),
  NewPing(36, 37, MAX_DISTANCE),
  NewPing(38, 39, MAX_DISTANCE),
};


unsigned int pastSonarVal1[SONAR_NUM]; 
unsigned int pastSonarVal2[SONAR_NUM]; 
unsigned int pastSonarVal3[SONAR_NUM]; 
int numPastVal = 0;

//int slowMotorSpeed = 5; //90 + this Value = 95
//int fastMotorSpeed = 60; //90 + this Value = 150
//
//int slowTurn = 5;
//int fastTurn = 10;

int left_fast_fwd = 80;//52;
int left_med_fwd = 80;//63;
int left_slow_fwd = 85;//77;
int left_slow_rev = 95;//101;
int left_med_rev = 117;
int left_fast_rev = 130;

int right_fast_fwd = 100;//130;
int right_med_fwd = 100;//120;
int right_slow_fwd = 95;//104;
int right_slow_rev = 85;//80;
int right_med_rev = 60;
int right_fast_rev = 50;

Servo leftMotor;
Servo rightMotor;

int backSensorCentered = 0; //Value the back sensor should see in order to center the car

int soberState = 0; //0 = Not correcting, 1 = Correcting Left, 2 = Correcting Right
int accumulatedCenteredValue = 0; //value that if the car was centered, the sonars would see the same thing
int numReadings = 0;
bool isTurning_90 = 0;
int Turning_Dir = 0;

///////////////////////////////////////////////////////////////
///////////             Initializations           /////////////
///////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
  leftMotor.attach(10);
  rightMotor.attach(3);
  
}



///////////////////////////////////////////////////////////////
//////////////             Main Loop           ////////////////
///////////////////////////////////////////////////////////////
int atBaseFlag = 0;
void loop(){  
  //while we have not found the base, or stopped at the base to commence grabbing
  while (!atBaseFlag){
    
    /////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////
    //Gather the Data From the Sonars
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
      if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
        if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                          // Sensor being accessed.
        readSonars[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }
    //End of Sonar Data Gathering
    /////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////
    // The rest of your code would go here.
    
    //checkSonarZeroing(newSD1_L, newSD2_R);  //NOT IMPLEMENTED YET
    
    //closestDistanceTracking(sonarVal[sonarFL], sonarVal[sonarFR]);
    //soberDriving(sonarVal[sonarSL], sonarVal[sonarSR], sonarVal[sonarB]);
    shittyIdea(sonarVal[sonarSL], (pastSonarVal1[sonarSL] + pastSonarVal2[sonarSL] + pastSonarVal3[sonarSL])/3);
//                leftMotor.write(left_med_fwd);
//            rightMotor.write(right_med_fwd);
    //atBaseFlag = checkAtBase(newSD1_L, newSD2_R);
  }
}

///////////////////////////////////////////////////////////////
////////          Sonar Checking Function          ////////////
///////////////////////////////////////////////////////////////
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()){
    readSonars[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM ;
    sonarVal[currentSensor] = readSonars[currentSensor];
  }
}

///////////////////////////////////////////////////////////////
//////   What to do after one cycle of Sonar Data   ///////////
///////////////////////////////////////////////////////////////
void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  
  for (int i = 0; i < SONAR_NUM; i++)
    pastSonarVal3[i] = pastSonarVal2[i];
  for (int i = 0; i < SONAR_NUM; i++)
    pastSonarVal2[i] = pastSonarVal1[i];
  for (int i = 0; i < SONAR_NUM; i++)
    pastSonarVal1[i] = sonarVal[i];
    
  /*for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonarVal[i]);
    Serial.print("cm ");
  }
  Serial.println();*/
}

///////////////////////////////////////////////////////////////
/////   Turn 90 Degrees in the Designated Direction   /////////
///////////////////////////////////////////////////////////////
unsigned long time = 0;
bool turn90(int turnDirection){
  //Shawn to Write Functionality For
  if (turnDirection == TurnLEFT){
    Serial.print("Turn90-Left\n\n");
    isTurning_90 = 1;
    Turning_Dir = TurnLEFT;
    time = millis();
    leftMotor.write(left_slow_rev);
    rightMotor.write(right_slow_fwd);
    return 0;
  }
  else if (turnDirection == TurnRIGHT){
    Serial.print("Turn90-Right\n\n");
    isTurning_90 = 1;
    Turning_Dir = TurnRIGHT;
    time = millis();
    leftMotor.write(left_med_fwd);
    rightMotor.write(right_slow_rev);
    return 0;
  }
  else{
    
//        Serial.print("Compare: ");
//        Serial.print(millis());
//        Serial.print(" - ");
//        Serial.print(time);
//        Serial.print(" [");
//        Serial.print(millis() - time);
//        Serial.print("] >= ");
//        Serial.print(3000);
//        Serial.print("\n\n");
    if ((millis() - time) >= 3000){
      Serial.print("Turn90-Complete\n\n");
      isTurning_90 = 0;
      Turning_Dir = 0;
      return 1; //if turn is completep based on SHAWN'S LOGIC 
    }
    else
      return 0;
  }
  
}

///////////////////////////////////////////////////////////////
////////   Tracks and Goes to the Closest Object   ////////////
///////////////////////////////////////////////////////////////
void closestDistanceTracking(float newSD1_L, float newSD2_R){
  
  if (newSD1_L == 0 || newSD2_R == 0){
    if(newSD1_L == 0 && newSD2_R == 0){
    }
    else if (newSD1_L == 0 && newSD2_R != 0){
      leftMotor.write(left_fast_fwd);
      rightMotor.write(right_slow_fwd);
      Serial.print("Rgt");
      Serial.print("\t\t");
    }
    else if (newSD1_L != 0 && newSD2_R == 0){
      leftMotor.write(left_slow_fwd);
      rightMotor.write(right_fast_fwd);
      Serial.print("Lft");
      Serial.print("\t\t");
    }
  }
    
  else{
    if (abs(newSD1_L - newSD2_R) > 3){
      //Turn Right
      if (newSD1_L > newSD2_R){
        leftMotor.write(left_fast_fwd);
        rightMotor.write(right_slow_fwd);
        Serial.print("Rgt");
        Serial.print("\t\t");
      }
      //Turn Left
      else if (newSD1_L < newSD2_R){
        leftMotor.write(left_slow_fwd);
        rightMotor.write(right_fast_fwd);
        Serial.print("Lft");
        Serial.print("\t\t");
      }
    }
    
    //Go Straight
    else{
      leftMotor.write(left_med_fwd);
      rightMotor.write(right_med_fwd);
      Serial.print("Fwd");
      Serial.print("\t\t");
    }
  }  
  Serial.print("S1_L: ");
  Serial.print(newSD1_L); 
  Serial.print("\t\t");
  Serial.print("S2_R: ");
  Serial.print(newSD2_R);
  Serial.print("\n\n");
}



///////////////////////////////////////////////////////////////
////////       Drive Center Between Two Walls      ////////////
///////////////////////////////////////////////////////////////
void shittyIdea(int leftSonarVal, int pastAVGLeftSonarVal){
  
  int threshold = 1;
  //Turn Right since we're getting closer
  if (abs(leftSonarVal - pastAVGLeftSonarVal) >= threshold){
    if (leftSonarVal < pastAVGLeftSonarVal){
      leftMotor.write(left_med_fwd);
      rightMotor.write(right_slow_fwd);
Serial.print("Rght"); 
Serial.print("\t");
    }
    else{ //turn left since we're farther from previous
      leftMotor.write(left_slow_fwd);
      rightMotor.write(right_med_fwd);
Serial.print("Lft"); 
Serial.print("\t");
    }
  }
  else{
    leftMotor.write(left_med_fwd);
    rightMotor.write(right_med_fwd);
Serial.print("Fwd"); 
Serial.print("\t");
  }
  
  Serial.print("leftSonarVal: ");
  Serial.print(leftSonarVal); 
  Serial.print("\t");
  Serial.print("pastAVGLeftSonarVal: ");
  Serial.print(pastAVGLeftSonarVal); 
  Serial.print("\n\n");
  
}


///////////////////////////////////////////////////////////////
////////       Drive Center Between Two Walls      ////////////
///////////////////////////////////////////////////////////////
int correctingDist = 0;
int soberDriving(int leftSonarVal, int rightSonarVal, int backSonarVal){
     
  //if either of th sensors give you zero... just drive forward and wing it for now
  if (leftSonarVal == 0 || rightSonarVal == 0){
    leftMotor.write(left_med_fwd);
    rightMotor.write(right_med_fwd);
    Serial.print("Zro");
    Serial.print("\t");
  }    
  
  //if both sensors are giving non-zero values
  else{    
    
    int straightThreshold = 3; //threshold distance to continue going straight correct
    int slightCorrectThreshold = 50; //threshold distance to slightly turn while moving straight, if greater than this, do a full 90 turn and correct
    
    int centeredValue = 45;//accumulatedCenteredValue / numReadings; //44; //value that if the car was centered, the sonars would see the same thing
    
    if (soberState == 0){
      
      if (isTurning_90){
        if ((Turning_Dir == TurnLEFT) && turn90(TurnCOMPLETE)) //where TurnCOMPLETE tells the function that it is watching to see if the turn has completed
          soberState = 1;
        else if ((Turning_Dir == TurnRIGHT) && turn90(TurnCOMPLETE)) //where TurnCOMPLETE tells the function that it is watching to see if the turn has completed
          soberState = 2;
      }
      
      else{
        //if the difference is less than the desired full correction threshold
//        Serial.print("Compare: ");
//        Serial.print(abs(centeredValue - rightSonarVal));
//        Serial.print("<=");
//        Serial.print(slightCorrectThreshold);
//        Serial.print("\n\n");
        
        int compareVal = 0;
        if (abs(centeredValue - leftSonarVal) > abs(centeredValue - rightSonarVal))
          compareVal = leftSonarVal;
        else
          compareVal = rightSonarVal;
          
        if ((abs(centeredValue - compareVal) <= slightCorrectThreshold)){
          if ((abs(centeredValue - compareVal) >= straightThreshold)){
            
            correctingDist = abs(centeredValue - compareVal);
            
            //Correct to the Left
            if (rightSonarVal < leftSonarVal){
              leftMotor.write(left_slow_fwd);
              rightMotor.write(right_med_fwd);
              soberState = 3;
              Serial.print("Lft");
              Serial.print("\t");
            }
            //Correct to the Right
            else if (rightSonarVal > leftSonarVal){
              leftMotor.write(left_med_fwd);
              rightMotor.write(right_slow_fwd);
              soberState = 5;
              Serial.print("Rgt");
              Serial.print("\t");
            }
          }
          
          //Go Straight
          else{
            leftMotor.write(left_fast_fwd);
            rightMotor.write(right_fast_fwd);
            soberState = 0;
            Serial.print("Fwd");
            Serial.print("\t");
            
            //only get the center value when we're going straight
            accumulatedCenteredValue = accumulatedCenteredValue + ((leftSonarVal + rightSonarVal) / 2);
            numReadings++;
          }
        }//if less then slightCorrectThreshold
      
        //if the difference is greater than the desired full correction threshold,
        //then do a full 90deg turn to correct, and change the soberState
        else{
          //Turn 90deg in the direction needed to center the car     
          //if we're note already turning 90deg, then initialize the turning sequence
          if (!isTurning_90){
            if (leftSonarVal > rightSonarVal)
              turn90(TurnLEFT);
            else
              turn90(TurnRIGHT);
          }
        }   
      }//!isTurning_90 
    }//soberState = 0
    
    //Turned left in order to correct back to the center
    else if (soberState == 1){
      //Software Debounce to ensure the back sensor is actually seeing the value we want,
      //so that we don't prematurely stop the car
      if (backSonarVal >= backSensorCentered){
        delayMicroseconds(4000);
        if (backSonarVal >= backSensorCentered){
          if (isTurning_90){ //if the car is already turning
            if (turn90(TurnCOMPLETE)){ //where TurnCOMPLETE tells the function that it is watching to see if the turn has completed
              soberState = 0;
            }
          }
          else{ //if the car is already turning
            turn90(TurnRIGHT); //We had turned to correct and go left, so turn right in order to look down the center again         
          }  
        }
      }
    }
    
    //Turned right in order to correct back to the center
    else if (soberState == 2){
      //Software Debounce to ensure the back sensor is actually seeing the value we want,
      //so that we don't prematurely stop the car
      if (backSonarVal >= backSensorCentered){
        delayMicroseconds(4000);
        if (backSonarVal >= backSensorCentered){
          if (isTurning_90){ //if the car is already turning
            if (turn90(TurnCOMPLETE)) //where TurnCOMPLETE tells the function that it is watching to see if the turn has completed
              soberState = 0;
          }
          else //if the car is already turning
            turn90(TurnLEFT); //We had turned to correct and go right, so turn left in order to look down the center again
        }
      }
    }
    
    //Slightly Correcting Left
    else if (soberState == 3 || soberState == 4){
      //Start turning right 
      //if (soberState == 3 && (abs(centeredValue - rightSonarVal) < straightThreshold)){
      if (soberState == 3 && (rightSonarVal >= centeredValue)){//(centeredValue + (correctingDist/2)))){
        leftMotor.write(left_med_fwd);
        rightMotor.write(right_slow_fwd);
        Serial.print("Rgt");
        Serial.print("\t");  
        soberState = 4;
      }
      
      //Once the system is centered (both left and right are within the threshold)
      else if (soberState == 4 && ((abs(centeredValue - rightSonarVal) < straightThreshold))){ //&& (abs(centeredValue - leftSonarVal) < straightThreshold))){
        leftMotor.write(left_med_fwd);
        rightMotor.write(right_med_fwd);
        Serial.print("Fwd");
        Serial.print("\t"); 
        soberState = 0;
      }
      else
        Serial.print("C_L\t");
    }

    //Slightly Correcting Right    
    else if (soberState == 5 || soberState == 6){
      //Start turning left 
      
//        Serial.print("Compare: ");
//                Serial.print(centeredValue);
//                      Serial.print("  ");
//                        Serial.print(leftSonarVal);
//                              Serial.print("  ");
//        Serial.print(abs(centeredValue - leftSonarVal));
//        Serial.print("<");
//        Serial.print(slightCorrectThreshold);
//        Serial.print("\n\n");
//      if (soberState == 5 && (abs(centeredValue - leftSonarVal) < straightThreshold)){
      if (soberState == 5 && (leftSonarVal >= centeredValue)){//(centeredValue + (correctingDist/2)))){
        leftMotor.write(left_slow_fwd);
        rightMotor.write(right_med_fwd);
        Serial.print("Lft");
        Serial.print("\t"); 
        soberState = 6;
      }
      
      //Once the system is centered (both left and right are within the threshold)
      else if (soberState == 6 && ((abs(centeredValue - leftSonarVal) < straightThreshold))){// && (abs(centeredValue - rightSonarVal) < straightThreshold))){
        leftMotor.write(left_med_fwd);
        rightMotor.write(right_med_fwd);
        Serial.print("Fwd");
        Serial.print("\t"); 
        soberState = 0;
      }
      else
        Serial.print("C_R\t");
    }
  }
  
  Serial.print("sS: ");
  Serial.print(soberState); 
  Serial.print("\t");
  Serial.print("lSVal: ");
  Serial.print(leftSonarVal); 
  Serial.print("\t");
  Serial.print("rSVal: ");
  Serial.print(rightSonarVal);
  Serial.print("\t");
  Serial.print("bSVal: ");
  Serial.print(backSonarVal);
  Serial.print("\n\n");
  
  return soberState;
}

///////////////////////////////////////////////////////////////
//////   Checks if one of the Sonar's is seeing nothing   /////
///////////////////////////////////////////////////////////////
/*void checkSonarZeroing(float newSD1_L, float newSD2_R){
  
  if ((newSD1_L == 0) && (newSD2_R != 0)){
    int debounced = 0;
    while (!debounced){
      //Turn Right as long as the sensor still sees 0
      while (newSD1_L == 0){
        leftMotor.write(90 - slowMotorSpeed - slowTurn);
        rightMotor.write(90 + slowMotorSpeed - slowTurn);
        Serial.print("Right");
        Serial.print("\t\t");
      }
      
      
   else if (newSD1_L < newSD2_R){
      leftMotor.write(90 - slowMotorSpeed + slowTurn);
      rightMotor.write(90 + slowMotorSpeed + slowTurn);
      Serial.print("Left");
      Serial.print("\t\t");
    }
  }
  
  else{
    leftMotor.write(90 - slowMotorSpeed);
    rightMotor.write(90 + slowMotorSpeed);
    Serial.print("Straight");
    Serial.print("\t\t");
  }
  
  Serial.print("S1_L: ");
  Serial.print(newSD1_L); 
  Serial.print("\t\t");
  Serial.print("S2_R: ");
  Serial.print(newSD2_R);
  Serial.print("\n\n");
}*/


///////////////////////////////////////////////////////////////
/////////////   Checks if We're at the Base   /////////////////
///////////////////////////////////////////////////////////////
int checkAtBase(float newSD1_L, float newSD2_R){
  
  int stopFlag = 0;
  float atBaseDist = 8;
  float sensePastBaseDist = 12;
  
  //if the Left Sonar says it's at the base
  if (newSD1_L < atBaseDist){
    //if the Right Sonar says it's at the base or is sensing over it (at least seeing the lego man)
    if ((newSD2_R < atBaseDist) || (newSD2_R > sensePastBaseDist)){
      stopFlag = 1;
    }      
  }
  
  //if the Right Sonar says it's at the base
  else if (newSD2_R < atBaseDist){
    //if the Left Sonar says it's at the base or is sensing over it (at least seeing the lego man)
    if ((newSD1_L < atBaseDist) || (newSD1_L > sensePastBaseDist)){
      stopFlag = 1;
    }      
  }
    
  if (stopFlag){
    //Stop the Motors
    leftMotor.write(90);
    rightMotor.write(90);
    
    Serial.print("At Base - Grab the Bitch"); 
    Serial.print("\n\n"); 
  }
  
  return stopFlag;
}


///////////////////////////////////////////////////////////////
/////   Tries to identify if the base is seen by either   /////
/////   of the sonars, based on the expected distance to  /////
/////                the base from the car                /////
///////////////////////////////////////////////////////////////
float sonarTrackToDistance (float inputDist, float carSpeed, float newSD1_L, float newSD2_R){
  
  int state = 0; //0 = not found, 1 = leftMotor(1) found, 2 = rightMotor(2) found
  float threshold1 = 10; // 10cm threshold for finding the object
  
  //Identify if any of the Sonars see the object at this point in time
  if (abs(inputDist - newSD1_L) < threshold1)
    state = 1;
  else if (abs(inputDist - newSD2_R) < threshold1)
    state = 2;
  else
    state = 0;
  
  return state;
}
