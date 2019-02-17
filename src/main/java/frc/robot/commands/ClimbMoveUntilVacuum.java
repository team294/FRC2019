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
  
  private double targetAng;
  private boolean isAtClimbAngle;   // have we reached the climb angle?
  private double timeAtClimbAngle;  // the time that we achieved the climb angle
  
  public ClimbMoveUntilVacuum(double targetAng) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climb);
    this.targetAng = targetAng;
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climb.enableCompressor(false);
    Robot.climb.setClimbPos(targetAng);
    isAtClimbAngle = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Turn on vacuum pump when we get close to the climb angle
    if (Robot.climb.getClimbAngle() <= (targetAng + 10)) {
      Robot.climb.enableVacuum(true);
    }

    // Record the first time when we are within 4 degrees of climb angle, 
    // in case vacuum switch disconnects from Rio and we need a timeout
    if (!isAtClimbAngle && Robot.climb.getClimbAngle() <= (targetAng + 4) {
      timeAtClimbAngle = timeSinceInitialized();
      isAtClimbAngle = true;
    }

    Robot.climb.updateClimbLog();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Finish if we have vacuum, or if we time out after 6 seconds being at the target angle
    return (Robot.climb.isVacuumPresent() || 
      (isAtClimbAngle && (timeSinceInitialized() > timeAtClimbAngle + 6)));
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
