/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.*;
import frc.robot.triggers.*;
import frc.robot.utilities.RobotPreferences.ElevatorPosition;
import frc.robot.utilities.RobotPreferences.WristAngle;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

  public Joystick leftJoystick = new Joystick(0);
  public Joystick rightJoystick = new Joystick(1);
  public Joystick coPanel = new Joystick(2);
  public Joystick xBoxController = new Joystick(3);

  public OI() {
    Button[] left = new Button[12];
    Button[] right = new Button[12];
    Button[] coP = new Button[20];
    Button[] xbB = new Button[11];
    Trigger xbUp = new POVTrigger(xBoxController, 0);
    Trigger xbRight = new POVTrigger(xBoxController, 90);
    Trigger xbDown = new POVTrigger(xBoxController, 180);
    Trigger xbLeft = new POVTrigger(xBoxController, 270);
    Trigger xbLT = new AxisTrigger(xBoxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xBoxController, 3, 0.9);

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    for (int i = 1; i < xbB.length; i++) {
      xbB[i] = new JoystickButton(xBoxController, i);
    }

    for (int i = 0; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // XBox controller buttons/triggers
    xbB[1].whenPressed(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchLow)); // A
    xbB[2].whenPressed(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchMid)); // B
    xbB[3].whenPressed(new ElevatorWristStow()); // X
    xbB[4].whenPressed(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchHigh)); // Y
    // xbB[5].whenPressed(new RearHatchSet(false)); // LB
    xbB[5].whenActive(new HatchSet(false)); // LB
    xbB[5].whenInactive(new HatchSet(true)); // LB
    xbB[6].whenPressed(new ElevatorWristMoveAndPrepare(ElevatorPosition.cargoShipCargo)); // RB
    xbB[7].whenPressed(new StopAllMotors()); // Back
    xbB[8].whenPressed(new CargoRearHatchStop()); // Start
    xbB[9].whenPressed(new ElevatorWithXBox()); // LStick
    xbB[10].whenPressed(new WristWithXBox()); // RStick
    xbUp.whenActive(new CargoIntakeFromLoad()); // DPadUp
    // xbRight.whenActive(new HatchSet(true)); // DPadRight
    // xbRight.whenActive(new HatchFrontAndRearSet(true)); // DPadRight press
    // xbRight.whenInactive(new RearHatchSetPercentOutput(0.6, 1)); // DPadRight release
    xbDown.whenActive(new CargoIntakeFromGround()); // DPadDown
    // xbLeft.whenActive(new HatchSet(false)); // DPadLeft
    // xbLeft.whenActive(new HatchFrontAndRearSet(false)); // DPadLeft press
    // xbLeft.whenInactive(new RearHatchSetPercentOutput(-0.6, 1.5)); // DPadLeft release
    // xbLeft.whenActive(new HatchSet(false)); // DPad Left Press
    // xbLeft.whenInactive(new HatchSet(true)); // DPad Left Release
    xbLT.whenActive(new CargoOuttake(-1.0)); // LT
    xbRT.whenActive(new CargoOuttake(-0.8)); // RT

    // Joystick buttons
    left[1].whenPressed(new Shift(false)); // low gear
    right[1].whenPressed(new Shift(true)); // high gear
    left[2].whenPressed(new DriveAssist()); // Turn on vision pipeline and move elevator low
    // left[2].whileHeld(new DriveWithVision(false, false)); // drive with visionf
    // left[2].whenReleased(new DriveWithJoysticks());
    left[2].whenReleased(new ElevatorWristMoveAndPrepare(ElevatorPosition.hatchLow));
    right[2].whileHeld(new DriveStraightJoystick()); // drive straight with right joystick
    right[2].whenReleased(new DriveWithJoysticks()); // drive with joysticks
    // left[3].whenPressed(new Command());
    // right[3].whenPressed(new Command());
    left[4].whenPressed(new VisionChangePipeline(0)); // set pipeline for vision
    // right[4].whenPressed(new Command());
    left[5].whenPressed(new VisionChangePipeline(2)); // set pipeline for drive feed
    // right[5].whenPressed(new Command());

    // Copanel buttons
    coP[1].whenPressed(new ClimbArmSetAngle(Robot.robotPrefs.climbStart)); // top row, first button, UP
    coP[2].whenPressed(new ClimbPrepSequence()); // top row, first button, DOWN
    coP[3].whileHeld(new ClimbArmSetPercentOutput(0.5)); // top row, second button, UP
    coP[4].whileHeld(new ClimbArmSetPercentOutput(-0.5)); // top row, second button, DOWN
    coP[5].whenPressed(new ClimbVacuumTurnOn(true)); // top row, third button, UP
    coP[6].whenPressed(new ClimbVacuumTurnOn(false)); // top row, third button, DOWN
    coP[7].whileHeld(new CargoIntake()); // mid row, fourth button, UP or DOWN
    coP[8].whenPressed(new ClimbSequence()); // BIG RED BUTTON
    coP[9].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.hatchHigh)); // mid row, first button, UP
    coP[10].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.hatchMid)); // mid row, first button, DOWN
    // coP[11].whenPressed(new RearHatchSet(true)); // mid row, second button, UP
    // coP[12].whenPressed(new RearHatchSet(false)); // mid row, second button, DOWN
    coP[13].whenPressed(new WristSetPercentOutput(0.2)); // mid row, third button, UP
    coP[14].whenPressed(new WristSetPercentOutput(-0.2)); // mid row, third button, DOWN
    coP[15].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.cargoShipCargo)); // third row, first button, UP
    coP[16].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.hatchLow)); // third row, first button, DOWN

    // Buttons for shifter
    SmartDashboard.putData("Shift High", new Shift(true));
    SmartDashboard.putData("Shift Low", new Shift(false));

    // Buttons for controlling the elevator
    SmartDashboard.putData("Elevator Up", new ElevatorSetPercentOutput(0.4)); // For testing limit switch and encoder
    SmartDashboard.putData("Elevator Down", new ElevatorSetPercentOutput(-0.2)); // For testing limit switch and encoder
    SmartDashboard.putData("Test", new ElevatorMoveToLevel(50.0));
    SmartDashboard.putData("Move Elevator to Bottom", new ElevatorMoveToLevel(ElevatorPosition.bottom)); // Move to encoder's zero position
    SmartDashboard.putData("Move Elevator to WristStow", new ElevatorMoveToLevel(ElevatorPosition.wristStow)); // Move to level that wrist can be stowed safely
    SmartDashboard.putData("Move Elevator to High", new ElevatorMoveToLevel(ElevatorPosition.hatchHigh)); // Move to high position (test wrist interlock)
    SmartDashboard.putData("Elevator Cal if Low", new ElevatorCalibrateIfAtLowerLimit());

    // Buttons for controlling the climber
    SmartDashboard.putData("Climb Up", new ClimbArmSetPercentOutput(0.8));
    SmartDashboard.putData("Climb Down", new ClimbArmSetPercentOutput(-0.8));
    SmartDashboard.putData("Climb move to start", new ClimbArmStow());  // For testing
    SmartDashboard.putData("Climb move to vacuum", new ClimbArmSetAngle(Robot.robotPrefs.climbVacuumAngle));  // For testing
    SmartDashboard.putData("Climb lift robot", new ClimbArmSetAngle(Robot.robotPrefs.climbLiftAngle));  // For testing
    SmartDashboard.putData("Climb Vacuum On", new ClimbVacuumTurnOn(true));
    SmartDashboard.putData("Climb Vacuum Off", new ClimbVacuumTurnOn(false));
    SmartDashboard.putData("ClimbMoveUntilVacuum", new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    SmartDashboard.putData("ClimbSequence", new ClimbSequence());

    // Test Buttons
    SmartDashboard.putData("Turn to 90", new TurnWithGyro(90.0, false));
    SmartDashboard.putData("Turn to -90", new TurnWithGyro(-90.0, false));
    SmartDashboard.putData("Turn to 0", new TurnWithGyro(0.0, false));
    SmartDashboard.putData("Turn +90", new TurnWithGyro(90.0, true));
    SmartDashboard.putData("Turn -90", new TurnWithGyro(-90.0, true));
    SmartDashboard.putData("Turn +10", new TurnWithGyro(10.0, true));
    SmartDashboard.putData("Turn -10", new TurnWithGyro(-10.0, true));
    
    // Buttons for the Cargo rollers
    SmartDashboard.putData("Cargo Intake", new CargoIntake());
    SmartDashboard.putData("Cargo Outtake", new CargoOuttake(-0.8));

    // Buttons for the rear hatch intake
    SmartDashboard.putData("Rear Hatch Intake", new RearHatchSetPercentOutput(0.6, 5));
    SmartDashboard.putData("Rear Hatch Outtake", new RearHatchSetPercentOutput(-0.6, 3));
    SmartDashboard.putData("Rear Hatch Retract", new RearHatchSet(false));
    SmartDashboard.putData("Rear Hatch Extend", new RearHatchSet(true));

    // Buttons for controlling the wrist
    SmartDashboard.putData("Wrist recalibrate", new WristEncoderFail());
    SmartDashboard.putData("Wrist Up", new WristSetPercentOutput(0.2));  // For testing
    SmartDashboard.putData("Wrist Down", new WristSetPercentOutput(-0.2));  // For testing
    SmartDashboard.putData("Wrist pos stow", new WristMoveToAngle(WristAngle.stowed));
    SmartDashboard.putData("Wrist vision", new WristMoveToAngle(WristAngle.vision));
    SmartDashboard.putData("Wrist pos down", new WristMoveToAngle(WristAngle.down));
    SmartDashboard.putData("Wrist pos straight", new WristMoveToAngle(WristAngle.straight));
    SmartDashboard.putData("Wrist pos up", new WristMoveToAngle(WristAngle.up));
    SmartDashboard.putData("Wrist +10 deg", new WristChangeAngle(10.0));
    SmartDashboard.putData("Wrist -10 deg", new WristChangeAngle(-10.0));

    // Buttons for controlling FileLogging
    SmartDashboard.putData("Log 1 InitialTesting", new FileLogSetLevel(1));
    SmartDashboard.putData("Log 2 PitTesting", new FileLogSetLevel(2));
    SmartDashboard.putData("Log 3 Competition", new FileLogSetLevel(3));
    Robot.log.setLogLevel(3);   // Also puts log level indicator on ShuffleBoard

    // These aren't useful; they don't update when the robot is running
    SmartDashboard.putBoolean("Left LineFollower", Robot.lineFollowing.isLinePresent(1));
    SmartDashboard.putBoolean("Middle LineFollower", Robot.lineFollowing.isLinePresent(2));
    SmartDashboard.putBoolean("Right LineFollower", Robot.lineFollowing.isLinePresent(3));

    SmartDashboard.putData("Clear Sticky Faults", new ClearStickyFaults());
    Robot.robotPrefs.showStickyFaults();
    SmartDashboard.putData("Disc Grab", new HatchSet(true));
    SmartDashboard.putData("Disc Release", new HatchSet(false));
    SmartDashboard.putString("Disc Position", "Null");
    SmartDashboard.putData("DriveStraight 100 in", new DriveStraightDistanceProfile(100, 0, 80, 65));
    SmartDashboard.putData("DriveToCargoShip", new DriveToCargoShip());

    SmartDashboard.putData("Pathfinder Test", new DrivePathfinder("R2ShipFront", true, true));

/*
    SmartDashboard.putData("LEDSet Purple", new LEDSet(0));
    SmartDashboard.putData("LEDSet Red", new LEDSet(1));
    SmartDashboard.putData("LEDSet Blue", new LEDSet(2));
    SmartDashboard.putData("LEDSet Off", new LEDSet(3));
*/
  }

  /**
	 * Sets the Xbox controller rumble power.
	 * 
	 * @param percentRumble value 0 to 1
	 */
	public void setXBoxRumble(double percentRumble) {
		xBoxController.setRumble(RumbleType.kLeftRumble, percentRumble);
    xBoxController.setRumble(RumbleType.kRightRumble, percentRumble);
	}
}
