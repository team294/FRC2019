/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.RobotPreferences.TurnDirection;


public class TurnWithGyro extends Command {

  double targetAngle;
  double inputAngle;
  boolean isRelativeGyroAngle;
  double originalGyroAngle;
  TurnDirection turnDirection;

  /**
   * Turns the robot in place
   * Default direction is the shortest turn
   * @param targetAngle in degrees
   * @param isRelativeAngle true = turn relative to current robot heading (+ = turn right, - = turn left).  
   * false = turn to absolute robot heading on the gyro.
   */
  public TurnWithGyro(double targetAngle, boolean isRelativeAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    inputAngle = targetAngle;
    isRelativeGyroAngle = isRelativeAngle;
    turnDirection = TurnDirection.shortest;
  }

  /**
   * Turns the robot in place (allows you to choose direction of turn)
   * @param targetAngle in degrees
   * @param isRelativeAngle true = turn relative to current robot heading (+ = turn right, - = turn left).  
   * false = turn to absolute robot heading on the gyro. 
   * @param direction left = turn left, right = turn right, shortest = turn to shortest path
   */
  public TurnWithGyro(double targetAngle, boolean isRelativeAngle, TurnDirection direction) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    inputAngle = targetAngle;
    isRelativeGyroAngle = isRelativeAngle;
    turnDirection = direction;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.turnwithGyroReset();

    originalGyroAngle = Robot.driveTrain.getGyroRotation();
    if(isRelativeGyroAngle){
      targetAngle = originalGyroAngle + inputAngle;
    } else {
      targetAngle = inputAngle;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.turnWithGyro(targetAngle, turnDirection);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs( Robot.driveTrain.normalizeAngle(Robot.driveTrain.getGyroRotation() - targetAngle) ) <= 1.5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop();
  }
}
