/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;

public class ClimbLift extends Command {
  
  double targetAng;
  
  public ClimbLift(double targetAng) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climb);
    this.targetAng = targetAng;
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climb.enableCompressor(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {  
    if (Robot.climb.getClimbAngle() >= (targetAng - 10)) {
      Robot.climb.setClimbMotorPercentOutput(0.2);
      Robot.climb.enableVacuum(true);
    }
    else {
      Robot.climb.setClimbMotorPercentOutput(0.5);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.climb.isVacuumAchieved();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climb.stopClimbMotor();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
