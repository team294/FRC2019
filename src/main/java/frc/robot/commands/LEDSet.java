/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// TODO  change blue to green.  Red/Green is more useful  That will change purple also. check
// TODO  add blink function 
// TODO  add call to green when robot is in scoring position
// TODO  add call to flash green when vision tracking is acquired


package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.LedSet;
import edu.wpi.first.wpilibj.command.Command;

public class LEDSet extends Command {

  private double colorValue;
  private boolean ifBlink;
  private boolean blinkOn;
  private long currentTime = 0;
  private long initialTime = 0;
  
  
  public LEDSet(double myColorValue) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.leds);
    colorValue = myColorValue;
    if (colorValue<0 || colorValue>3) {
      colorValue = 0;
    }
    ifBlink = false;
  }
  public LEDSet(double myColorValue, boolean myIfBlink) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.leds);
    colorValue = myColorValue;
    ifBlink = myIfBlink;
    if (colorValue<0 || colorValue>3) {
      colorValue = 0;
    } if (colorValue==0 && ifBlink==true) {
      ifBlink = false;
    }
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    /* COLOR VALUE :
    0 = OFF
    1 = GREEN
    2 = BLUE
    3 = RED
    Else/Default = OFF */

    if(!ifBlink) {
      if(colorValue == 0){
        Robot.leds.setOff();
      } else if(colorValue == 1){
        Robot.leds.setOff();
        Robot.leds.setGreen();
      } else if (colorValue == 2){
        Robot.leds.setOff();
        Robot.leds.setBlue();
      } else if(colorValue == 3){
        Robot.leds.setOff();
        Robot.leds.setRed();
      }else{
      }
    }
    
    initialTime = System.currentTimeMillis();
    blinkOn = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (ifBlink) {
    currentTime = System.currentTimeMillis();
      if(currentTime > initialTime + 800){
        if(blinkOn == true){
          blinkOn = false;
        } else {
          blinkOn = true;
        }
        currentTime = System.currentTimeMillis();
        initialTime = System.currentTimeMillis();
      }
      if(blinkOn == true){
        Robot.leds.setOff();
      } else {
        if(colorValue == 1){
          //Robot.leds.setOff();
          Robot.leds.setGreen();
        } else if (colorValue == 2){
          //Robot.leds.setOff();
          Robot.leds.setBlue();
        } else if(colorValue == 3){
          //Robot.leds.setOff();
          Robot.leds.setRed();
        }else{
        }
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
