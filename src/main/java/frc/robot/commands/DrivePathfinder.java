/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.pathfinder.Pathfinder;
import frc.robot.pathfinder.Trajectory;
import frc.robot.pathfinder.followers.DistanceFollower;

public class DrivePathfinder extends Command {
  private DistanceFollower dfLeft, dfRight;
  private Trajectory trajCenter;
  private Trajectory trajRight;
  private Trajectory trajLeft;
  private boolean resetGyro;
  private static boolean enablePathfinder = true; // true = all paths loaded and allow pathfinder to work,
                                                  // false = a path did not load, stop pathfinder

  double distL = 0, distR = 0;      // Distance traveled
  Trajectory.Segment segLeft, segRight;
  double l = 0, r = 0, turn = 0;    // power to send to drive motors
  double gyro_heading = 0, desired_heading = 0;
  
  /**
   * Drive following a path
   * @param pathName        File name excluding .pf1.csv
   * @param gyroReset       True = reset gyro to first heading on trajectory, False = don't reset gyro
   * @param driveDirection  True = drive forward, False = drive backward
   */
  public DrivePathfinder(String pathName, boolean gyroReset, boolean driveDirection) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    resetGyro = gyroReset;

    // Load Pathfinder paths
    if (driveDirection) {
      trajCenter = Pathfinder.readFromCSV(pathName + ".pf1.csv", driveDirection);
      trajRight = Pathfinder.readFromCSV(pathName + ".left.pf1.csv", driveDirection);
      trajLeft = Pathfinder.readFromCSV(pathName + ".right.pf1.csv", driveDirection);
    } else {
      trajCenter = Pathfinder.readFromCSV(pathName + ".pf1.csv", driveDirection);
      trajRight = Pathfinder.readFromCSV(pathName + ".right.pf1.csv", driveDirection);
      trajLeft = Pathfinder.readFromCSV(pathName + ".left.pf1.csv", driveDirection);
    }
  
    // If a path file was missing, then disable Pathfinder for all paths.
    // Note that enablePathfinder is static, so one copy is shared by all DrivePathfinder objects.
    // We disable all paths, because if one path is missing in a sequence, then the we don't
    // want any subsequent paths to run assuming that we followed the missing path.
    if (trajCenter == null || trajRight == null || trajLeft == null) {
      enablePathfinder = false;
      Robot.robotPrefs.recordStickyFaults("Pathfinder");
      Robot.log.writeLogEcho("Pathfinder", "", "Missing path " + pathName);
      return;
    }

    // Don't do anything if any other path is missing
    if (!enablePathfinder) {
      return;
    }

    // Create DistanceFollowers for the Trajectories and configure them
    dfLeft = new DistanceFollower(trajLeft);
    dfRight = new DistanceFollower(trajRight);

    dfLeft.configurePIDVA(0.08, 0.0, 0.0, 1 / Robot.robotPrefs.max_velocity_ips, 0.0013, 0.0012); // P = 0.2, 0.05
    dfRight.configurePIDVA(0.08, 0.0, 0.0, 1 / Robot.robotPrefs.max_velocity_ips, 0.0013, 0.0012); // A = 0.0032, 0.004
    
    segLeft = dfLeft.getSegment();
    segRight = dfRight.getSegment();

    logData();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Don't do anything if this path or any other path is missing
    if (!enablePathfinder) {
      return;
    }
    
    Robot.log.writeLog("Pathfinder", "initialize", "current time," + System.currentTimeMillis() + ",start time," + dfLeft.getStartTimeMillis());
    Robot.driveTrain.setDriveMode(true);
    Robot.driveTrain.zeroLeftEncoder();
    Robot.driveTrain.zeroRightEncoder();
    Robot.driveTrain.setVoltageCompensation(true);

    dfLeft.reset();
    dfRight.reset();
    distL = 0;
    distR = 0;

    if (resetGyro) {
      Robot.driveTrain.setGyroRotation(Pathfinder.r2d(trajCenter.segments[0].heading));
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Don't do anything if this path or any other path is missing
    if (!enablePathfinder) {
      return;
    }
    
    distL = Robot.driveTrain.getLeftEncoderInches();
    distR = Robot.driveTrain.getRightEncoderInches();

    l = dfLeft.calculate(distL);
    r = dfRight.calculate(distR);

    segLeft = dfLeft.getSegment();
    segRight = dfRight.getSegment();

    gyro_heading = Robot.driveTrain.getGyroRotation();    // Assuming the gyro is giving a value in degrees
    desired_heading = Pathfinder.r2d(dfLeft.getHeading());  // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    turn = 0.01 * angleDifference;

    Robot.driveTrain.setLeftMotors(-(l + turn));
    Robot.driveTrain.setRightMotors(-(r - turn));
    
    logData();
  }


  private void logData() {
    Robot.log.writeLog(false, "Pathfinder", "execute", "time," + ((double)(System.currentTimeMillis() - dfLeft.getStartTimeMillis())) / 1000.0 +
    ",left power," + l + ",right power," + r + ",turn power," + turn +
    ",left distance," + distL + ",right distance," + distR + ",left vel," + Robot.driveTrain.getLeftEncoderVelocity() + ",right vel," + Robot.driveTrain.getRightEncoderVelocity() +
    ",heading," + gyro_heading + ",left isFinished," + dfLeft.isFinished() +
    ",left segPos," + segLeft.position + ",left segVel," + segLeft.velocity + ",left segAccel," + segLeft.acceleration + 
    ",left segJerk," + segLeft.jerk + ",left segHeading," + Pathfinder.boundHalfDegrees(Pathfinder.r2d(segLeft.heading)) + ",left segdt," + segLeft.dt + ",right isFinished," + dfRight.isFinished() + 
    ",right segPos," + segRight.position + ",right segVel," + segRight.velocity + ",right segAccel," + segRight.acceleration + 
    ",right segJerk," + segRight.jerk + ",right segHeading," + Pathfinder.boundHalfDegrees(Pathfinder.r2d(segRight.heading)) + ",right segdt," + segRight.dt);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Don't do anything if this path or any other path is missing
    if (!enablePathfinder) {
      return true;
    }
    
    if (dfLeft.isFinished()) {
      Robot.driveTrain.setVoltageCompensation(false);
      // Robot.driveTrain.setDriveMode(true);
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
    Robot.driveTrain.tankDrive(0, 0);
    Robot.driveTrain.setVoltageCompensation(false);
  }
}