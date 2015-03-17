bool turn90(int turnDirection){
  //Shawn to Write Functionality For
  if (turnDirection == TurnLEFT){
    prev_right = sonarVal[sonarSR];
    prev_back = sonarVal[sonarSB];
    Serial.print("Turn90-Left\n\n");
    isTurning_90 = 1;
    Turning_Dir = TurnLEFT;
    
    leftMotor.write(left_slow_rev);
    rightMotor.write(right_slow_fwd);
    
    return 0;
  }
  else if (turnDirection == TurnRIGHT){
    prev_left = sonarVal[sonarSL];
    prev_back = sonarVal[sonarSB];
    Serial.print("Turn90-Right\n\n");
    isTurning_90 = 1;
    Turning_Dir = TurnRIGHT;
    
    leftMotor.write(left_slow_fwd);
    rightMotor.write(right_slow_rev);
    
    return 0;
  }
  else{
    Serial.println("Check-Turn90-Complete");
    
    if (Turning_Dir == TurnLEFT){ //if completing a left turn
      if ((abs(sonarVal[sonarSL] - prev_back) < straightThreshold) && (abs(sonarVal[sonarSB] - prev_right) < straightThreshold)){
        //if new left ~equals old back and new back ~equals old right, you have now turned left 90 degrees
        Serial.println("Left-Turn90-Complete \n");
        
        isTurning_90 = 0;
        Turning_Dir = 0;
      }
      else{
        Serial.println("Left-Turn90-Incomplete \n");
      }
    }
    else{ //if completing a right turn
      if ((abs(sonarVal[sonarSR] - prev_back) < straightThreshold) && (abs(sonarVal[sonarSB] - prev_left) < straightThreshold)){
        //if new right ~equals old back and new back ~equals old left, you have now turned right 90 degrees
        Serial.println("Right-Turn90-Complete \n");
        
        isTurning_90 = 0;
        Turning_Dir = 0;
      }
      else{
        Serial.println("Right-Turn90-Incomplete \n");
      }
    }
    
    return 1;
  }
  
}
