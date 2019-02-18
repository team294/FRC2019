/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CargoIntake extends Command {

  private boolean hasBallTime = false;    // Did we see the ball yet?
  private double timeWhenGrabbedBall;     // Time to start counting when we have the ball

  /**
   * Run cargo intake until we grab a ball in the intake.
   * Actually, run the intake a little longer (1 sec?) after we see the 
   * ball, so, that it is fully grabbed.
   */
  public CargoIntake() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cargo);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //TODO Change percent power when we get a cargo intake
    Robot.cargo.setCargoMotorPercent(0.5, 0.3);
    hasBallTime = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //TODO Change percent power when we get a cargo intake
    Robot.cargo.setCargoMotorPercent(0.5, 0.3);

    // Record time when we grabbed the ball
    if (!hasBallTime && Robot.cargo.hasBall()) {
      hasBallTime = true;
      timeWhenGrabbedBall = timeSinceInitialized();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (hasBallTime && (timeSinceInitialized() > timeWhenGrabbedBall + 1));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.cargo.stopCargoIntake();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
