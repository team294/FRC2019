/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.pathfinder.Pathfinder;
import frc.robot.pathfinder.Trajectory;
import frc.robot.pathfinder.Waypoint;
import frc.robot.pathfinder.followers.DistanceFollower;

public class DrivePathfinder extends Command {
  private DistanceFollower dfLeft, dfRight;
  private boolean hasReset;
  Trajectory trajCenter;
  Trajectory trajRight;
  Trajectory trajLeft;
  boolean resetGyro;
  String logString = "";

  public DrivePathfinder(String pathName, boolean gyroReset) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    trajCenter = Pathfinder.readFromCSV(pathName + ".pf1.csv");
    trajRight = Pathfinder.readFromCSV(pathName + ".left.pf1.csv");
    trajLeft = Pathfinder.readFromCSV(pathName + ".right.pf1.csv");
    resetGyro = gyroReset;
    
    // Create DistanceFollowers for the Trajectories and configure them
    dfLeft = new DistanceFollower(trajLeft);
    dfRight = new DistanceFollower(trajRight);
    dfLeft.configurePIDVA(0.2, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0032); // P = 0.05
    dfRight.configurePIDVA(0.2, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0032); // A = 0.0038
    
    for(int index = 0; index < 1000; index++) {
      logString += " ";
    }

    Trajectory.Segment segLeft = trajLeft.segments[0];
    Trajectory.Segment segRight = trajRight.segments[0];
    double l = 0, r = 0, turn = 0, gyro_heading = 0;

    logString = "time," + ((double)(System.currentTimeMillis() - dfLeft.getStartTimeMillis()) / 1000.0 +
                           ",left power," + l + ",right power," + r + ",turn power," + turn +
                           ",left distance," + Robot.driveTrain.getLeftEncoderInches() + ",right distance," + Robot.driveTrain.getRightEncoderInches()) +
                           ",heading," + gyro_heading + ",left isFinished," + dfLeft.isFinished() +
                           ",left segPos," + segLeft.position + ",left segVel," + segLeft.velocity + ",left segAccel," + segLeft.acceleration + 
                           ",left segJerk," + segLeft.jerk + ",left segHeading," + Pathfinder.boundHalfDegrees(Pathfinder.r2d(segLeft.heading)) + ",left segdt," + segLeft.dt + ",right isFinished," + dfRight.isFinished() + 
                           ",right segPos," + segRight.position + ",right segVel," + segRight.velocity + ",right segAccel," + segRight.acceleration + 
                           ",right segJerk," + segRight.jerk + ",right segHeading," + Pathfinder.boundHalfDegrees(Pathfinder.r2d(segRight.heading)) + ",right segdt," + segRight.dt;
    }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.log.writeLog("Pathfinder", "initialize", "current time," + System.currentTimeMillis() + ",start time," + dfLeft.getStartTimeMillis());
    Robot.driveTrain.setDriveMode(true);
    Robot.driveTrain.zeroLeftEncoder();
    Robot.driveTrain.zeroRightEncoder();
    Robot.driveTrain.setVoltageCompensation(true);
    hasReset = false;

    if (resetGyro) {
      Robot.driveTrain.setGyroRotation(Pathfinder.r2d(trajCenter.segments[0].heading));
    }

    dfLeft.reset();
    dfRight.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!hasReset) {
      dfLeft.reset();
      dfRight.reset();
      hasReset = true;
    }
    double l = dfLeft.calculate(Robot.driveTrain.getLeftEncoderInches());
    double r = dfRight.calculate(Robot.driveTrain.getRightEncoderInches());

    Trajectory.Segment segLeft = dfLeft.getSegment();
    Trajectory.Segment segRight = dfRight.getSegment();

    double gyro_heading = Robot.driveTrain.getGyroRotation();    // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(dfLeft.getHeading());  // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = 0.04 * angleDifference;
    Robot.driveTrain.setLeftMotors(-(l + turn));
    Robot.driveTrain.setRightMotors(-(r - turn));
    logString = "time," + ((double)(System.currentTimeMillis() - dfLeft.getStartTimeMillis()) / 1000.0 +
                           ",left power," + l + ",right power," + r + ",turn power," + turn +
                           ",left distance," + Robot.driveTrain.getLeftEncoderInches() + ",right distance," + Robot.driveTrain.getRightEncoderInches()) +
                           ",heading," + gyro_heading + ",left isFinished," + dfLeft.isFinished() +
                           ",left segPos," + segLeft.position + ",left segVel," + segLeft.velocity + ",left segAccel," + segLeft.acceleration + 
                           ",left segJerk," + segLeft.jerk + ",left segHeading," + Pathfinder.boundHalfDegrees(Pathfinder.r2d(segLeft.heading)) + ",left segdt," + segLeft.dt + ",right isFinished," + dfRight.isFinished() + 
                           ",right segPos," + segRight.position + ",right segVel," + segRight.velocity + ",right segAccel," + segRight.acceleration + 
                           ",right segJerk," + segRight.jerk + ",right segHeading," + Pathfinder.boundHalfDegrees(Pathfinder.r2d(segRight.heading)) + ",right segdt," + segRight.dt;
    Robot.log.writeLog("Pathfinder", "execute", logString);
   }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (dfLeft.isFinished()) {
      Robot.driveTrain.setVoltageCompensation(false);
      Robot.driveTrain.setDriveMode(false);
      return true;
    }
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