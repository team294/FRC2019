/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LedSet;

public class LEDBlink extends Command {
  
  private long currentTime = 0;
  private long initialTime = 0;
  private boolean lightRed = false;
  private int colorValue = 0;

  public LEDBlink(int pColorValue) {
    // Use requires() here to declare subsystem  
    // eg. requires(chassis);
    requires(Robot.leds);
    colorValue = pColorValue;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialTime = System.currentTimeMillis();
    lightRed = false;
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      currentTime = System.currentTimeMillis();

        //switches the boolean between true and false every 1800 seconds
        //resets the time after every switch
        if(currentTime > initialTime + 800){
          if(lightRed == true){
            lightRed = false;
          } else {
            lightRed = true;
          }
          currentTime = System.currentTimeMillis();
          initialTime = System.currentTimeMillis();
        }

        if(lightRed == true){
          System.out.println("Setting off in blink");
            Robot.leds.setOff();
        } else {
          if(colorValue == 1){
            Robot.leds.setGreen();
          } else {
            Robot.leds.setRed();
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