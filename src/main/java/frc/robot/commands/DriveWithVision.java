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

public class DriveWithVision extends Command {

  private boolean endOnLine = false;
  private boolean gyro = false;
  private double targetQuad = 0; // The quadrant of the target we want to drive to

  private double stopDistance = 32.0;  // Used 26.0 in the lab, changed to 28 for safety.  Should be able to use 34 with intake low.
  private final double MAX_SPEED = 0.75;
  private final double MIN_SPEED = 0.2;
  private double priorVisionSpeed = MAX_SPEED;
  private double visionSpeed = MAX_SPEED;

  private double lPercentOutput, rPercentOutput;

  private double distance = 0;
  private double area = 0;
  private double xVal = 0;
  private double yVal = 0;
  private double skew = 0;

  /**
   * Vision assisted driving without gyro, keep going and never end on the line
   */
  public DriveWithVision() {
    this(false, false);
  }

  /**
   * Drive towards the vision target
   * @param endOnLine specify whether or not to end on the line target.
   * @param gyro specify whether or not to use gyro curve correction
   *  </br> true means end on line, false means continue to wall (will not exit with false)
   */
  public DriveWithVision(boolean endOnLine, boolean gyro) {
    requires(Robot.driveTrain);
    this.endOnLine = endOnLine;
    this.gyro = gyro;

    // Robot.vision.setPipe(0); // On vision pipeline
    Robot.vision.setLedMode(3); // TODO Change back to 3 to turn on LEDs.  Make sure the LEDs are on before driving

    updateLog();  // Prime StringBuilder to speed up code during execution
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.vision.setPipe(0);
    Robot.vision.setLedMode(3);
    Robot.driveTrain.setDriveModeCoast(false);
    SmartDashboard.putBoolean("Ready to Score", false);
    Robot.driveTrain.clearEncoderList(); // May not be necessary to clear
    //Robot.driveTrain.driveToCrosshair();
    if (gyro) {
      targetQuad = Robot.driveTrain.checkScoringQuadrant();
      Robot.log.writeLog(false, "DriveWithVision", "Init", "Gyro,true,Quadrant,"+targetQuad);
    } else {
      Robot.log.writeLog(false, "DriveWithVision", "Init", "Gyro,false");
    }

    priorVisionSpeed = MAX_SPEED;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double xOffsetAdjustmentFactor = 1.5; // Should be tested to be perfect; 2 seems to go out of frame too quickly. Must be greater than 1. Was 1.5
    //double xOffsetAdjustmentFactor = 2.0 + Robot.oi.leftJoystick.getY(); // xAdjustment based on distance


    //double minDistanceToTarget = 13;
    distance = Robot.vision.distance; // Distance formula should work now; need to modulate speed based on dist
    area = Robot.vision.areaFromCamera;
    xVal = Robot.vision.horizOffset; // Alpha offset
    yVal = Robot.vision.vertOffset;
    skew = Robot.vision.skew;
    double finalAngle;

    if (targetQuad != 0) {
      SmartDashboard.putNumber("Measured Angle", xVal);
      double alphaT = xVal + Robot.driveTrain.getGyroRotation() - Robot.driveTrain.getTargetAngle(targetQuad); // true angle for measuring x displacement
      SmartDashboard.putNumber("Adjusted angle to target ", alphaT);
      double alphaA = Math.toDegrees(Math.atan(xOffsetAdjustmentFactor * Math.tan(Math.toRadians(alphaT)))); // Adjusted angle for x displacement
      SmartDashboard.putNumber("False displacement angle", alphaA);
      finalAngle = alphaA + Robot.driveTrain.getTargetAngle(targetQuad) - Robot.driveTrain.getGyroRotation();
      if (finalAngle > 180) finalAngle -= 360; // Should fix problems in quadrant 3.5 regarding angle overflow
      if (finalAngle < -180) finalAngle += 360; // Relative angle
      SmartDashboard.putNumber("Final Angle", finalAngle);
    } else {
      // Drive directly towards target
      finalAngle = xVal;

      // If we are farther than 40in from the target, use the target skew to 
      // follow an S-curve path to approach the target from a perpendicular line
      if (distance>43) finalAngle -= skew*distance * 0.0;  // tuned to 0.025  for distance > 43
    }

    double gainConstant = distance * 0.00005 + 0.008;  // tuned to 0.008

    //double lJoystickAdjust = Math.abs(Robot.oi.leftJoystick.getY());
    //double lJoystickAdjust = 0.7 * Math.sqrt(lJoystickRaw);
    //double lJoystickAdjust = 0.55 / (1 + Math.exp(-10 * (lJoystickRaw - 0.35)));
    // double lJoystickAdjust = 0.50 / (1 + Math.exp(-8 * (lJoystickRaw - 0.4))); // Slightly longer acceleration curve than previous sigmoid

    // Prior code
    // double lJoystickRaw = Math.abs(Robot.oi.leftJoystick.getY());
    // double lJoystickAdjust = lJoystickRaw * 0.8;

    // Decease speed in last 10 inches
    visionSpeed = (distance > stopDistance + 15) ? MAX_SPEED : (MAX_SPEED - MIN_SPEED) * (distance-stopDistance)/15.0 + MIN_SPEED;

    // Check for bad values from vision.  If so, then just go forward at prior speed and we will correct on next cycle.
    if ( distance > 150 || distance < stopDistance - 2) {
      visionSpeed = priorVisionSpeed;
      finalAngle = 0;
    }

    // Don't allow speed to increase
    if (priorVisionSpeed < visionSpeed) visionSpeed = priorVisionSpeed;
    priorVisionSpeed = visionSpeed;

    SmartDashboard.putNumber("Vision Speed", visionSpeed);
    lPercentOutput = visionSpeed + (gainConstant * finalAngle);
    rPercentOutput = visionSpeed - (gainConstant * finalAngle);

    /* Untested auto-turn stuff */
    // if (lEncStopped && lPercentOutput != 0) rPercentOutput = 1.0; // The goal here is to slam the right side so that we still line up to the wall
    // if (rEncStopped && rPercentOutput != 0) lPercentOutput = 1.0; 
    // if (lPercentOutput == 1.0 || rPercentOutput == 1.0) System.out.println("STOP DETECTED, INITIATING EVASIVE MANEUVERS"); 

    if (area != 0 && Robot.vision.distance > stopDistance) Robot.driveTrain.tankDrive(lPercentOutput, rPercentOutput); // area goes to zero before the front hits the wall
    else Robot.driveTrain.stop();

    updateLog();
  }

  private void updateLog() {
    Robot.log.writeLog(false, "DriveWithVision", "update", "Crosshair Horiz Offset," + xVal + ",Vert Offset," + yVal
     + ",Target Area," + area + ",Target Skew," + skew + ",gyro," + Robot.driveTrain.getGyroRotation() + ",Inches from Target," + distance
     + ",DistUseArea," + Robot.vision.distanceUsingArea + ",DistUseCorner," + Robot.vision.distanceUsingCorners
     + ",Base power," + visionSpeed + ",Left Percent," + lPercentOutput + ",Right Percent," + rPercentOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Robot.driveTrain.areEncodersStopped(5.0);

    // We get some false readings in the first few cycles (likely due to robot vibration from wrist/elevator move), so 
    // keep going for at least 1 second
    return Robot.vision.distance <= stopDistance  && timeSinceInitialized()>1.0;

    // return endOnLine && Robot.lineFollowing.isLinePresent() && Robot.vision.distanceFromTarget() < 40; // Stops when a line is detected by the line followers within a reasonable expected distance
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
    Robot.vision.setPipe(2);
    Robot.vision.setLedMode(1);
    Robot.driveTrain.setDriveModeCoast(false);
    Robot.log.writeLog("DriveWithVision", "end", "");
    // Robot.leds.setColor(LedHandler.Color.OFF);   // Robot Periodic will turn off LEDs
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
