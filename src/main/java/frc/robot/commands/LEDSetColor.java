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

public class LEDSetColor extends Command {

  private double colorValue;
  
  public LEDSetColor(double myColorValue) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.leds);
    colorValue = myColorValue;
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // 0 is purple; 1 is green; 2 is red; defaults to yellow
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

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

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
