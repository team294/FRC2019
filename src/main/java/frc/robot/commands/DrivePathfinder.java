/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.pathfinder.Pathfinder;
import frc.robot.pathfinder.Trajectory;
import frc.robot.pathfinder.followers.DistanceFollower;
import frc.robot.utilities.RobotPreferences;

public class DrivePathfinder extends Command {
  private DistanceFollower dfLeft, dfRight, dfCenter;
  private Trajectory trajCenter;
  private Trajectory trajRight;
  private Trajectory trajLeft;
  private boolean resetGyro;
  private String pathName;
  private static boolean enablePathfinder = true; // true = all paths loaded and allow pathfinder to work,
                                                  // false = a path did not load, stop pathfinder

  double distL = 0, distR = 0, distC = 0;      // Distance traveled
  double distLPrior = 0, distRPrior = 0, distCPrior = 0;      // Distance traveled in prior iteration of loop
  Trajectory.Segment segLeft, segRight, segCenter;
  double l = 0, r = 0, c = 0, distErrTerm = 0, turn = 0;    // power to send to drive motors
  double gyroHeading = 0, gyroHeadingPrior = 0, desiredHeading = 0;
  double skidGain = 0.28;      // Magnify delta between wheels by this factor to account for skid
  double skidAdjust = 0.0;    // Temp variable for skid adjustments

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
    this.pathName = pathName;

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
      this.pathName = "FILE NOT FOUND: " + this.pathName;
      Robot.robotPrefs.recordStickyFaults("Pathfinder");
      Robot.log.writeLogEcho("Pathfinder", "", this.pathName);
    }

    SmartDashboard.putBoolean("Pathfinder enabled", enablePathfinder);

    // Don't do anything if this path or any other path is missing
    if (!enablePathfinder) {
      return;
    }

    // Create DistanceFollowers for the Trajectories and configure them
    
    dfLeft = new DistanceFollower(trajLeft);
    dfRight = new DistanceFollower(trajRight);
    dfCenter = new DistanceFollower(trajCenter);

    dfLeft.configurePIDVA(0.0, 0.0, 0.0, 1 / Robot.robotPrefs.max_velocity_ips, 0.0013, 0.0012); // P = 0.2, 0.05
    dfRight.configurePIDVA(0.0, 0.0, 0.0, 1 / Robot.robotPrefs.max_velocity_ips, 0.0013, 0.0012); // A = 0.0032, 0.004
    dfCenter.configurePIDVA(0.08, 0.0, 0.0, 1 / Robot.robotPrefs.max_velocity_ips, 0.0013, 0.0012);
    
    segCenter = dfCenter.getSegment();
    segLeft = dfLeft.getSegment();
    segRight = dfRight.getSegment();

    // Prime string building in data logger to prevent delays when executing the command
    logData();
  }

  /**
   * Filename of path file loaded
   * @return
   */
  public String getPathName() {
    return pathName;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Don't do anything if this path or any other path is missing
    if (!enablePathfinder) {
      return;
    }
    
    Robot.log.writeLog("Pathfinder", "initialize", "current time," + System.currentTimeMillis() + ",start time," + dfLeft.getStartTimeMillis());
    Robot.driveTrain.setDriveModeCoast(true);
    Robot.driveTrain.zeroLeftEncoder();
    Robot.driveTrain.zeroRightEncoder();
    Robot.driveTrain.setVoltageCompensation(true);

    dfLeft.reset();
    dfRight.reset();
    dfCenter.reset();
    distL = 0;  distLPrior = 0;
    distR = 0;  distRPrior = 0;
    distC = 0;  distCPrior = 0;

    if (resetGyro) {
      gyroHeadingPrior = Pathfinder.r2d(trajCenter.segments[0].heading);
      Robot.driveTrain.setGyroRotation(gyroHeadingPrior);
    } else {
      gyroHeadingPrior = Robot.driveTrain.getGyroRotation();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Don't do anything if this path or any other path is missing
    if (!enablePathfinder) {
      return;
    }
    
    // Get distance travelled
    distL = Robot.driveTrain.getLeftEncoderInches();
    distR = Robot.driveTrain.getRightEncoderInches();

    // Calculate wheel power from left and right profiles
    l = dfLeft.calculate(distL);
    r = dfRight.calculate(distR);
    c = dfCenter.calculate(distC);

    // Adjust feed-forward for wheel skid when turning
    skidAdjust = (l-r)*skidGain/2.0;

    // Get segments for data logging and feedback calculations
    segLeft = dfLeft.getSegment();
    segRight = dfRight.getSegment();
    segCenter = dfCenter.getSegment();

    // Add feedback for error in robot heading
    gyroHeading = Robot.driveTrain.getGyroRotation();    // Gyro heading in degrees
    desiredHeading = Pathfinder.r2d(dfCenter.getHeading());  // Should also be in degrees
    turn = 0.01 * Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);

    // Add feedback for distance travelled
    // distC = (distL + distR) / 2;
    double deltaL = distL - distLPrior;
    double deltaR = distR - distRPrior;
    if (deltaL > deltaR) {
      // Turning right.  Adjust for skid.
      // Use the distance from the inner wheel from the turn + heading change
      distC = distCPrior +  deltaR + Pathfinder.d2r(Pathfinder.boundHalfDegrees(gyroHeading - gyroHeadingPrior)) * Robot.robotPrefs.wheelbase_in / 2.0;
    } else {
      // Turning left.  Adjust for skid.
      // Use the distance from the inner wheel from the turn + heading change
      distC = distCPrior +  deltaL + Pathfinder.d2r(Pathfinder.boundHalfDegrees(gyroHeadingPrior - gyroHeading)) * Robot.robotPrefs.wheelbase_in / 2.0;
    }
    distErrTerm = 0.08 * (segCenter.position - distC);

    // Set the motor percentage based on feed forward and feedback to follow the profile
    Robot.driveTrain.setLeftMotors(-(l + skidAdjust + distErrTerm + turn));
    Robot.driveTrain.setRightMotors(-(r + - skidAdjust + distErrTerm - turn));
    
    logData();

    // Save distances and heading
    distLPrior = distL;
    distRPrior = distR;
    distCPrior = distC;
    gyroHeadingPrior = gyroHeading;
  }

  private void logData() {
    Robot.log.writeLog(false, "Pathfinder", "execute", "time," + ((double)(System.currentTimeMillis() - dfLeft.getStartTimeMillis())) / 1000.0 +
    ",left power," + l + ",right power," + r + ",turn power," + turn + ",skid power," + skidAdjust + ",dist FB power," + distErrTerm +
    ",left distance," + distL + ",right distance," + distR + ",center distance," + distC +
    ",left vel," + Robot.driveTrain.getLeftEncoderVelocity() + ",right vel," + Robot.driveTrain.getRightEncoderVelocity() +
    ",heading," + gyroHeading + ",center segPos," + segCenter.position +
    ",left isFinished," + dfLeft.isFinished() +
    ",left segPos," + segLeft.position + ",left segVel," + segLeft.velocity + ",left segAccel," + segLeft.acceleration + 
    ",left segJerk," + segLeft.jerk + ",left segHeading," + Pathfinder.boundHalfDegrees(Pathfinder.r2d(segLeft.heading)) + ",left segdt," + segLeft.dt + 
    ",right isFinished," + dfRight.isFinished() + 
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
    Robot.driveTrain.tankDrive(0, 0);
    Robot.driveTrain.setVoltageCompensation(false);
    Robot.driveTrain.setDriveModeCoast(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}