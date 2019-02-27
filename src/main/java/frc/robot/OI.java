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
    Button[] coP = new Button[15];
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
    xbB[5].whenPressed(new CargoStop()); // LB
    xbB[6].whenPressed(new ElevatorWristMoveAndPrepare(ElevatorPosition.cargoShipCargo)); // RB
    xbB[7].whenPressed(new StopAllMotors()); // Back
    xbB[8].whenPressed(new CargoIntake()); // Start
    xbB[9].whenPressed(new ElevatorWithXBox()); // LStick
    xbB[10].whenPressed(new WristWithXBox()); // RStick
    xbUp.whenActive(new CargoIntakeFromLoad()); // DPadUp
    xbRight.whenActive(new HatchSet(true)); // DPadRight
    xbDown.whenActive(new CargoIntakeFromGround()); // DPadDown
    xbLeft.whenActive(new HatchSet(false)); // DPadLeft
    xbLT.whenActive(new CargoOuttake()); // LT
    xbRT.whenActive(new CargoOuttake()); // RT

    // Joystick buttons
    left[1].whenPressed(new Shift(true)); // high gear
    right[1].whenPressed(new Shift(false)); // low gear
    left[2].whileHeld(new DriveAssist()); // drive with vision assist
    left[2].whenReleased(new DriveWithJoysticks());
    right[2].whileHeld(new DriveStraightJoystick()); // drive straight with right joystick
    right[2].whenReleased(new DriveWithJoysticks()); // drive with joysticks
    left[3].whenPressed(new HatchScoreAndIntake()); // toggle hatch, back up
    // right[3].whenPressed(new Command());
    left[4].whenPressed(new VisionChangePipeline(0)); // set pipeline for vision
    right[4].whenPressed(new DriveSetDirection(false)); // set direction reverse
    left[5].whenPressed(new VisionChangePipeline(2)); // set pipeline for drive feed
    right[5].whenPressed(new DriveSetDirection(true)); // set direction forward

    // Copanel buttons
    coP[1].whenPressed(new ClimbArmSetAngle(Robot.robotPrefs.climbStart));
    coP[2].whenPressed(new ClimbArmSetAngle(Robot.robotPrefs.climbStart));
    coP[3].whenPressed(new ClimbArmSetPercentOutput(0.3));
    coP[4].whenPressed(new ClimbArmSetPercentOutput(-0.3));
    coP[5].whenPressed(new ClimbVacuumTurnOn(true));
    coP[6].whenPressed(new ClimbVacuumTurnOn(false));
    coP[8].whenPressed(new ClimbSequence());
    coP[9].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.hatchHigh));
    coP[10].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.hatchMid));
    coP[11].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.cargoShipCargo));
    coP[12].whenPressed(new ElevatorMoveToLevel(ElevatorPosition.hatchLow));

    // Buttons for controlling the elevator
    SmartDashboard.putData("Elevator Up", new ElevatorRaise()); // For testing limit switch and encoder
    SmartDashboard.putData("Elevator Down", new ElevatorLower()); // For testing limit switch and encoder
    SmartDashboard.putData("Move Elevator to Bottom", new ElevatorMoveToLevel(ElevatorPosition.bottom)); // Move to encoder's zero position
    SmartDashboard.putData("Move Elevator to WristStow", new ElevatorMoveToLevel(ElevatorPosition.wristStow)); // Move to level that wrist can be stowed safely
    SmartDashboard.putData("Move Elevator to High", new ElevatorMoveToLevel(ElevatorPosition.hatchHigh)); // Move to high position (test wrist interlock)
    SmartDashboard.putData("Zero Elev Enc (w/ Limit)", new ElevatorMoveToBottomThenZeroEncoder());

    // Buttons for controlling the climber
    SmartDashboard.putData("Climb Up", new ClimbArmSetPercentOutput(0.3));  // For testing
    SmartDashboard.putData("Climb Down", new ClimbArmSetPercentOutput(-0.3));  // For testing
    SmartDashboard.putData("Climb move to start", new ClimbArmStow());  // For testing
    SmartDashboard.putData("Climb move to vacuum", new ClimbArmSetAngle(Robot.robotPrefs.climbVacuumAngle));  // For testing
    SmartDashboard.putData("Climb lift robot", new ClimbArmSetAngle(Robot.robotPrefs.climbLiftAngle));  // For testing
    SmartDashboard.putData("Climb Vacuum On", new ClimbVacuumTurnOn(true));
    SmartDashboard.putData("Climb Vacuum Off", new ClimbVacuumTurnOn(false));
    SmartDashboard.putData("Climb Set Reference", new ClimbMoveToLimitThenCalibrate());
    SmartDashboard.putData("ClimbMoveUntilVacuum", new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    SmartDashboard.putData("ClimbSequence", new ClimbSequence());

    // Buttons for the Cargo rollers
    SmartDashboard.putData("Cargo Intake", new CargoIntake());
    SmartDashboard.putData("Cargo Outtake", new CargoOuttake());

    // Buttons for controlling the wrist
    SmartDashboard.putData("Wrist recalibrate", new WristEncoderFail());
    SmartDashboard.putData("Wrist Up", new WristSetPercentOutput(0.2));  // For testing
    SmartDashboard.putData("Wrist Down", new WristSetPercentOutput(-0.2));  // For testing
    SmartDashboard.putData("Wrist pos stow", new WristMoveToAngle(WristAngle.stowed));
    SmartDashboard.putData("Wrist pos down", new WristMoveToAngle(WristAngle.down));
    SmartDashboard.putData("Wrist pos straight", new WristMoveToAngle(WristAngle.straight));
    SmartDashboard.putData("Wrist pos up", new WristMoveToAngle(WristAngle.up));

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
    // SmartDashboard.putData("Turn To Line", new TurnToLine());
    SmartDashboard.putData("Disc Toggle", new HatchToggle());
    SmartDashboard.putData("Disc Grab", new HatchSet(true));
    SmartDashboard.putData("Disc Release", new HatchSet(false));
    SmartDashboard.putString("Disc Position", "Null");
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
