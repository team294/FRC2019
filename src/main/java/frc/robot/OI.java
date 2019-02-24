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
import frc.robot.utilities.RobotPreferences;
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
    // xbB[1].whenPressed(new ElevatorMoveAndPrepare(RobotPreferences.ElevatorPosition.hatchLow)); // A
    xbB[1].whenPressed(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.hatchLow)); // A
    // xbB[2].whenPressed(new ElevatorMoveAndPrepare(RobotPreferences.ElevatorPosition.hatchMid)); // B
    xbB[2].whenPressed(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.hatchMid)); // B
    xbB[3].whenPressed(new ElevatorWristStow()); // X
    // xbB[4].whenPressed(new ElevatorMoveAndPrepare(RobotPreferences.ElevatorPosition.hatchHigh)); // Y
    xbB[4].whenPressed(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.hatchHigh)); // Y
    xbB[5].whenPressed(new CargoStop()); // LB
    // xbB[6].whenPressed(new ElevatorMoveAndPrepare(RobotPreferences.ElevatorPosition.cargoShipCargo)); // RB
    xbB[6].whenPressed(new ElevatorMoveToLevel(RobotPreferences.ElevatorPosition.cargoShipCargo)); // RB
    xbB[7].whenPressed(new StopAllMotors()); // Back
    // xbB[8].whenPressed(new Command()); // Start
    xbB[9].whenPressed(new ElevatorWithXBox()); // LStick
    xbB[10].whenPressed(new WristWithXBox()); // RStick
    xbUp.whenActive(new CargoIntakeFromLoad()); // DPadUp
    xbRight.whenActive(new HatchSet(true)); // DPadRight
    xbDown.whenActive(new CargoIntakeFromGround()); // DPadDown
    xbLeft.whenActive(new HatchSet(false)); // DPadLeft
    xbLT.whenActive(new CargoOuttake()); // LT
    xbRT.whenActive(new CargoOuttake()); // RT

    // Joystick buttons
    left[1].whenPressed(new Shift(true));
    right[1].whenPressed(new Shift(false));
    left[2].whenPressed(new DriveAssist());
    // right[2].whenPressed(new Command());
    right[2].whenReleased(new DriveWithJoysticks());
    // left[3].whenPressed(new Command());
    // right[3].whileHeld(new DriveStraight(Robot.oi.rightJoystick.getY(), 0)); // TODO fix drivestraight
    left[4].whenPressed(new HatchScoreAndIntake());
    right[4].whenPressed(new DriveSetDirection(false));
    left[5].whenPressed(new HatchScoreAndIntake());
    right[5].whenPressed(new DriveSetDirection(true));

    // left[1].whenPressed(new Shift(true));
    // right[1].whenPressed(new Shift(false));
    // left[2].whenPressed(new DriveAssist());
    // right[2].whenPressed(new DriveAssist()); // Hatch grab
    // left[3].whenPressed(new DriveWithVision(false, true)); // No line followers, but gyro correction
    // right[3].whenPressed(new DriveWithVision(false, false)); // No line followers, no gyro
    // left[4].whenPressed(new DriveWithLineFollowing(true));
    // right[4].whenPressed(new DriveWithLineFollowing(false));
    // left[5].whenPressed(new DriveWithLineFollowing(true));
    // right[5].whenPressed(new DriveWithLineFollowing(false));
    // left[2].whenReleased(new DriveWithJoysticks());
    // right[2].whenReleased(new DriveWithJoysticks());
    // left[3].whenReleased(new DriveWithJoysticks());
    // right[3].whenReleased(new DriveWithJoysticks());
    // left[4].whenReleased(new DriveWithJoysticks());
    // right[4].whenReleased(new DriveWithJoysticks());
    // left[5].whenReleased(new DriveWithJoysticks());
    // right[5].whenReleased(new DriveWithJoysticks());

    // Copanel buttons
    coP[1].whenPressed(new ClimbArmSetAngle(Robot.robotPrefs.climbStart));
    coP[2].whenPressed(new ClimbArmSetAngle(Robot.robotPrefs.climbStart));
    coP[3].whenPressed(new ClimbArmSetPercentOutput(0.3));  // TODO determine manual control percent
    coP[4].whenPressed(new ClimbArmSetPercentOutput(-0.3));  // TODO determine manual control percent
    // coP[5].whenPressed(new Command());  // TODO add turning on vacuum
    // coP[6].whenPressed(new Command());  // TODO add turning off vacuum
    coP[8].whenPressed(new ClimbSequence());

    // Buttons for controlling the elevator
    SmartDashboard.putData("Elevator Up", new ElevatorRaise()); // For testing limit switch and encoder
    SmartDashboard.putData("Elevator Down", new ElevatorLower()); // For testing limit switch and encoder
    SmartDashboard.putData("Move Elevator to Bottom", new ElevatorMoveToLevel(ElevatorPosition.bottom)); // Move to encoder's zero position
    SmartDashboard.putData("Move Elevator to WristStow", new ElevatorMoveToLevel(ElevatorPosition.wristStow)); // Move to level that wrist can be stowed safely
    SmartDashboard.putData("Move Elevator to High", new ElevatorMoveToLevel(ElevatorPosition.hatchHigh)); // Move to high position (test wrist interlock)
    SmartDashboard.putData("Zero Elev Enc (w/ Limit)", new ElevatorMoveToBottomThenZeroEncoder());

    // Buttons for controlling the climber
    SmartDashboard.putData("Climb Up", new ClimbArmSetPercentOutput(0.2));  // For testing
    SmartDashboard.putData("Climb Down", new ClimbArmSetPercentOutput(-0.2));  // For testing
    SmartDashboard.putData("Climb move to start", new ClimbArmStow());  // For testing
    SmartDashboard.putData("Climb move to vacuum", new ClimbArmSetAngle(Robot.robotPrefs.climbVacuumAngle));  // For testing
    SmartDashboard.putData("Climb lift robot", new ClimbArmSetAngle(Robot.robotPrefs.climbLiftAngle));  // For testing
    SmartDashboard.putData("Climb Vacuum On", new ClimbVacuumTurnOn(true));
    SmartDashboard.putData("Climb Vacuum Off", new ClimbVacuumTurnOn(false));
    SmartDashboard.putData("Climb Set Reference", new ClimbMoveToLimitThenCalibrate());
    SmartDashboard.putData("ClimbMoveUntilVacuum", new ClimbMoveUntilVacuum(Robot.robotPrefs.climbVacuumAngle));
    SmartDashboard.putData("ClimbSequnce", new ClimbSequence());

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
    //SmartDashboard.putData("Turn To Line", new TurnToLine());
    SmartDashboard.putData("Disc Toggle", new HatchToggle());
    SmartDashboard.putData("Disc Grab", new HatchSet(true));
    SmartDashboard.putData("Disc Release", new HatchSet(false));
    SmartDashboard.putString("Disc Position", "Null");
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
