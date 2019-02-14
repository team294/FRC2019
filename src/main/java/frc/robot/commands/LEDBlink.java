/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Date;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LEDBlink extends Command {
  
  private long currentTime = 0;
  private long initialTime = 0;
  private Date date = new Date();

  public LEDBlink() {
    // Use requires() here to declare subsystem  
    // eg. requires(chassis);
    requires(Robot.leds);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialTime = date.getTime();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentTime = date.getTime();
        System.out.println("Current Time is: " + currentTime);
        if (currentTime > initialTime + 800){
            Robot.leds.setRed();
        } else {
            Robot.leds.setGreen();
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