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

public class PathfinderToRocket extends Command {
  private DistanceFollower dfLeft, dfRight;

  public PathfinderToRocket() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.zeroLeftEncoder();
    Robot.driveTrain.zeroRightEncoder();
    Robot.driveTrain.setGyroRotation(0);
    Robot.driveTrain.setVoltageCompensation(true);

    // double maxVelocityPercentLimit = 0.6;   // Limit max velocity to 0.4 of real max velocity (for safety and to obsereve)
    // Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
    //   Trajectory.Config.SAMPLES_HIGH, 0.01, RobotMap.max_velocity_ips*maxVelocityPercentLimit, 
    //   RobotMap.max_acceleration_ipsps, RobotMap.max_jerk_ipspsps);
    // Waypoint[] points = new Waypoint[] {
    //   // new Waypoint(297.85013158065647, 6.998403366802192, Pathfinder.d2r(0)),
    //   // new Waypoint(284.98625378310624, 176.3727943678803, Pathfinder.d2r(16.615842155169048))
    //   // new Waypoint(50, 50, Pathfinder.d2r(0)),
    //   // new Waypoint(30, 70, Pathfinder.d2r(10))
    // };

    // Trajectory trajectory = Pathfinder.generate(points, config);

    // // Save main trajectory for reference
    // File saveFile = new File("/home/lvuser/trajectory.csv");
    // Pathfinder.writeToCSV(saveFile, trajectory);

    // // Wheelbase Width
    // TankModifier modifier = new TankModifier(trajectory).modify(RobotMap.wheelbase_in);

    // // Save left and right side trajectories for reference
    // saveFile = new File("/home/lvuser/trajectory-left.csv");
    // Pathfinder.writeToCSV(saveFile, modifier.getLeftTrajectory());
    // saveFile = new File("/home/lvuser/trajectory-right.csv");
    // Pathfinder.writeToCSV(saveFile, modifier.getRightTrajectory());

    Trajectory trajCenter = Pathfinder.readFromCSV("Test.pf1.csv");
    Trajectory trajLeft = Pathfinder.readFromCSV("Test.left.pf1.csv");
    Trajectory trajRight = Pathfinder.readFromCSV("Test.right.pf1.csv");

    // Create DistanceFollowers for the Trajectories and configure them
    dfLeft = new DistanceFollower(trajLeft);
    dfRight = new DistanceFollower(trajRight);
    // dfLeft.configureEncoder(Robot.driveTrain.getLeftEncoderTicks(), RobotMap.encoderTicksPerRevolution, RobotMap.wheel_diameter_m);
    // dfRight.configureEncoder(Robot.driveTrain.getRightEncoderTicks(), RobotMap.encoderTicksPerRevolution, RobotMap.wheel_diameter_m);
    // dfLeft.configurePIDVA(0.02, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0025);
    // dfRight.configurePIDVA(0.02, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0025);
    dfLeft.configurePIDVA(0.05, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0038);
    dfRight.configurePIDVA(0.05, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0038);
    dfLeft.reset();
    dfRight.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Trajectory.Segment segLeft = dfLeft.getSegment();
    Trajectory.Segment segRight = dfRight.getSegment();

    double l = dfLeft.calculate(Robot.driveTrain.getLeftEncoderInches());
    double r = dfRight.calculate(Robot.driveTrain.getRightEncoderInches());

    double gyro_heading = Robot.driveTrain.getGyroRotation();    // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(dfLeft.getHeading());  // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = -0.016 * angleDifference;
    Robot.driveTrain.setLeftMotors(-(l + turn));
    Robot.driveTrain.setRightMotors(-(r - turn));

    Robot.log.writeLog("Pathfinder", "execute left", "left power," + l + ",right power," + r + ",turn power," + turn +
                       ",left distance," + Robot.driveTrain.getLeftEncoderInches() + ",right distance," + Robot.driveTrain.getRightEncoderInches() +
                       ",heading," + gyro_heading + 
                       ",isFinished," + dfLeft.isFinished() + 
                       ",segPos," + segLeft.position + ",segVel," + segLeft.velocity + ",segAccel," + segLeft.acceleration + 
                       ",segJerk," + segLeft.jerk + ",segHeading," + Pathfinder.r2d(segLeft.heading) + ",segdt," + segLeft.dt);
    SmartDashboard.putNumber("Pathfinder left position", segLeft.position);

    Robot.log.writeLog("Pathfinder", "execute right", "left power," + l + ",right power," + r + ",turn power," + turn +
                       ",left distance," + Robot.driveTrain.getLeftEncoderInches() + ",right distance," + Robot.driveTrain.getRightEncoderInches() +
                       ",heading," + gyro_heading + 
                       ",isFinished," + dfRight.isFinished() + 
                       ",segPos," + segRight.position + ",segVel," + segRight.velocity + ",segAccel," + segRight.acceleration + 
                       ",segJerk," + segRight.jerk + ",segHeading," + Pathfinder.r2d(segRight.heading) + ",segdt," + segRight.dt);
    SmartDashboard.putNumber("Pathfinder right position", segRight.position);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (dfLeft.isFinished()) {
      Robot.driveTrain.setVoltageCompensation(false);
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