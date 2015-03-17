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

#define TurnLEFT = 1;
#define TurnRIGHT = 2;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int sonarVal[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(30, 31, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(32, 33, MAX_DISTANCE),
  NewPing(34, 35, MAX_DISTANCE),
  NewPing(36, 37, MAX_DISTANCE),
  NewPing(38, 39, MAX_DISTANCE),
};


//int slowMotorSpeed = 5; //90 + this Value = 110
//int fastMotorSpeed = 60; //90 + this Value = 150
//
//int slowTurn = 5;
//int fastTurn = 10;

int left_fast_fwd = 52;
int left_med_fwd = 63;
int left_slow_fwd = 77;
int left_slow_rev = 101;
int left_med_rev = 117;
int left_fast_rev = 130;

int right_fast_fwd = 130;
int right_med_fwd = 120;
int right_slow_fwd = 104;
int right_slow_rev = 80;
int right_med_rev = 60;
int right_fast_rev = 50;

Servo leftMotor;
Servo rightMotor;


///////////////////////////////////////////////////////////////
///////////             Initializations           /////////////
///////////////////////////////////////////////////////////////
void setup()
{
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
void loop()
{  
  int debug_ldist, debug_rdist;
  //while we have not found the base, or stopped at the base to commence grabbing
  while (!atBaseFlag)
  {
    /////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////
    //Gather the Data From the Sonars
    for (uint8_t i = 0; i < SONAR_NUM; i++) // Loop through all the sensors.
    { 
      if (millis() >= pingTimer[i]) // Is it this sensor's time to ping?
      {         
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
        if (i == 0 && currentSensor == SONAR_NUM - 1) 
        {
          oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
          debug_ldist = sonarVal[2];
          debug_rdist = sonarVal[3];
        }
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                          // Sensor being accessed.
        sonarVal[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }
    
    //End of Sonar Data Gathering
    /////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////
    // The rest of your code would go here.
    
    //checkSonarZeroing(newSD1_L, newSD2_R);  //NOT IMPLEMENTED YET
    
    closestDistanceTracking(debug_ldist, debug_rdist);
    
    //atBaseFlag = checkAtBase(newSD1_L, newSD2_R);
  }
}

///////////////////////////////////////////////////////////////
////////          Sonar Checking Function          ////////////
///////////////////////////////////////////////////////////////
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    sonarVal[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM ;
}

///////////////////////////////////////////////////////////////
//////   What to do after one cycle of Sonar Data   ///////////
///////////////////////////////////////////////////////////////
void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
//    Serial.print(i);
//    Serial.print(,);
//    Serial.print(sonarVal[i]);
//    Serial.print(",");
  }
//  Serial.println();
}

///////////////////////////////////////////////////////////////
////////   Tracks and Goes to the Closest Object   ////////////
///////////////////////////////////////////////////////////////
void closestDistanceTracking(float newSD1_L, float newSD2_R){
  
  
  if (abs(newSD1_L - newSD2_R) > 1)
  {
    //Turn Left
    if (newSD1_L > newSD2_R)
    {
      leftMotor.write(left_slow_fwd);
      rightMotor.write(right_fast_fwd);
      Serial.print("Lft");
      Serial.print("\t\t");
    }
    //Turn Right
    else if (newSD1_L < newSD2_R)
    {
      leftMotor.write(left_fast_fwd);
      rightMotor.write(right_slow_fwd);
      Serial.print("Rgt");
      Serial.print("\t\t");
    }
  }
  
  //Go Straight
  else
  {
    leftMotor.write(left_med_fwd);
    rightMotor.write(right_med_fwd);
    Serial.print("Fwd");
    Serial.print("\t\t");
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
/*
int soberState = 0; //0 = Not correcting, 1 = Correcting Left, 2 = Correcting Right
void soberDriving(int leftSonarVal, int rightSonarVal, int backSonarVal){
  
  if (soberState = 0){
    int straightThreshold = 3 //threshold distance to continue going straight correct
    int slightCorrectThreshold = 8 //threshold distance to slightly turn while moving straight, if greater than this, do a full 90 turn and correct
    
    int centeredValue = (leftSonarVal + rightSonarVal) / 2 //value that if the car was centered, the sonars would see the same thing
    
    //if the difference is less than the desired full correction threshold
    if (abs(centeredValue - rightSonarVal) < slightCorrectThreshold){
      if (abs(centeredValue - rightSonarVal) > straightThreshold){
        //Correct to the Left
        if (rightSonarVal < leftSonarVal){
          leftMotor.write(90 - slowMotorSpeed + slowTurn);
          rightMotor.write(90 + slowMotorSpeed + slowTurn);
          soberState = 1;
          Serial.print("Left");
          Serial.print("\t\t");
        }
        //Correct to the Right
        else if (rightSonarVal > leftSonarVal){
          leftMotor.write(90 - slowMotorSpeed - slowTurn);
          rightMotor.write(90 + slowMotorSpeed - slowTurn);
          soberState = 2;
          Serial.print("Right");
          Serial.print("\t\t");
        }
      }
      
      //Go Straight
      else{
        leftMotor.write(90 - slowMotorSpeed);
        rightMotor.write(90 + slowMotorSpeed);
        soberState = 0;
        Serial.print("Straight");
        Serial.print("\t\t");
      }
    }
    
    //if the difference is greater than the desired full correction threshold, then do a full 90deg turn to correct
    else{
      
      
    
  }
  
  
  else if (soberState = 1){
    
    if (check90TurnComplete()){
      int backSensorShouldSee = 150;
      //Keep moving forward until the soberState changes
      leftMotor.write(90 - slowMotorSpeed);
      rightMotor.write(90 + slowMotorSpeed);
      
      if (backSonarVal > backSensorShouldSee){
        delayMicroseconds(4000);
        if (backSonarVal > backSensorShouldSee){
          turn90(TurnRIGHT);
          soberState = 0;
        
        
      
      }
      
    
    
    
  
  

  
  Serial.print("leftSonarVal: ");
  Serial.print(leftSonarVal); 
  Serial.print("\t\t");
  Serial.print("rightSonarVal: ");
  Serial.print(rightSonarVal);
  Serial.print("\n\n");
  
  return soberState;
}
*/
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
