/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ClimbMoveUntilVacuum extends Command {
  
  double targetAng;
  
  public ClimbMoveUntilVacuum(double targetAng) {
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
    Robot.climb.setClimbPos(targetAng);
    if (Robot.climb.getClimbAngle() <= (targetAng + 10)) {
      Robot.climb.enableVacuum(true);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.climb.isVacuumAchieved());// || (Robot.climb.getClimbAngle() <= targetAng + 5));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.climb.stopClimbMotor();
  }
}
