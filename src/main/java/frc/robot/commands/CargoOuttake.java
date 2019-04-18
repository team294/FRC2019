/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CargoOuttake extends Command {

  private double timeWithoutBall = 0; // the amount of time from the photo switch not sensing a ball
  private boolean startTime = false; // have we started to count the time from 
  private double initialPercentOutput, finalPercentOutput, delayTime;

  /**
   * Run cargo outtake 5 seconds, or until we no longer see the ball plus 1 additional second.
   * @param percentOutput power to kick the ball out, 0 to -1 (should be negative!)
   */
  public CargoOuttake(double percentOutput) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cargo);
    initialPercentOutput = percentOutput;
    finalPercentOutput = percentOutput;
    delayTime = 10.0;
  }

  /**
   * Run cargo outtake 5 seconds, or until we no longer see the ball plus 1 additional second.
   * Increase speed after a delay to ensure that the ball is ejected.
   * @param initialPercentOutput when command starts, power to kick the ball out, 0 to -1 (should be negative!)
   * @param delayTime time (in seconds) to transition from initialPercentOutput to finalPercentOutput
   * @param finalPercentOutput after delay has elapsed, power to kick the ball out, 0 to -1 (should be negative!)
   */
  public CargoOuttake(double initialPercentOutput, double delayTime, double finalPercentOutput) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cargo);
    this.initialPercentOutput = initialPercentOutput;
    this.finalPercentOutput = finalPercentOutput;
    this.delayTime = delayTime;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.cargo.setCargoMotorPercentOutput(initialPercentOutput);
    startTime = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (timeSinceInitialized() <= delayTime) {
      Robot.cargo.setCargoMotorPercentOutput(initialPercentOutput);
    } else {
      Robot.cargo.setCargoMotorPercentOutput(finalPercentOutput);
    }

    if(!Robot.cargo.hasBall() && !startTime){
      timeWithoutBall = timeSinceInitialized();
      startTime = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (timeSinceInitialized() >= 5 || (startTime && timeSinceInitialized() - timeWithoutBall > 1)) {
      Robot.cargo.stopCargoIntake();
      return true;
    } else {
      return false;
    }
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
