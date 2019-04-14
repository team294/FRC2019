package frc.robot.utilities;

import frc.robot.Robot;
import frc.robot.commands.DrivePathfinder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelection {
	public Command autonomousCommand;

	// Auto path selections
	public static final int AUTO_PLANS = 3;

	public enum AutoPlan {
		CargoShipFront, RocketFront, RocketBack;
		// do not change the order of these
	}

	// Starting positions
	public enum StartingPosition {
		Left1, Left2, Middle, Right1, Right2;
	}

	public static int[][] startingAutoPrograms = {
			{ 0, 1, 2, 3, 4 }, // Plan 0, CargoShipFront
			{ 5, 6, 7, 8, 9 }, // Plan 1, RocketFront
			{ 10, 11, 12, 13, 14 }, // Plan 2, RocketBack
	};

	public AutoSelection() {
		super();
	}
	
	public void selectPath() {
		int programSelected;
		int selectedPlan = 0;
		AutoPlan autoPlan;
		// Button startButton = new JoystickButton(Robot.oi.leftJoystick, 1);
		Button startButton = new JoystickButton(Robot.oi.coPanel, 7);

		Timer timeSinceAutoStart = new Timer();
		timeSinceAutoStart.start();
		
		StartingPosition startPosition = Robot.oi.readStartPosition();
		autoPlan = Robot.oi.readAutoPlan();

		switch (autoPlan) {
			case CargoShipFront:
				selectedPlan = 0;
				break;
			case RocketFront:
				selectedPlan = 1;
				break;
			case RocketBack:
				selectedPlan = 2;
		}
		
		if (startPosition == StartingPosition.Left1) {
			programSelected = startingAutoPrograms[selectedPlan][0];
		} else if (startPosition == StartingPosition.Left2) {
			programSelected = startingAutoPrograms[selectedPlan][1];
		} else if (startPosition == StartingPosition.Middle) {
			programSelected = startingAutoPrograms[selectedPlan][2];
		} else if (startPosition == StartingPosition.Right1) {
			programSelected = startingAutoPrograms[selectedPlan][3];
		} else {
			programSelected = startingAutoPrograms[selectedPlan][4];
		}

		if(startButton.get()) {
			switch (programSelected) {
				case 1:
					autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left1ShipF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
					break;
				case 2:
					autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left2ShipF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
					break;
				case 3:
					autonomousCommand = new DrivePathfinder("Middle1ShipF", true, true); //new DrivePathfinder("Middle1ShipF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
					break;
				case 4:
					autonomousCommand = new DrivePathfinder("Right1ShipF", true, true); //new DrivePathfinder("Right1ShipF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
					break;
				case 5:
					autonomousCommand = new DrivePathfinder("Right2ShipF", true, true); //new DrivePathfinder("Right2ShipF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
					break;
				case 6:
					autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left1RocketF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 2 (Rocket Front)", "startPos = " + startPosition.name());
					break;
				case 7:
					autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left2RocketF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 2 (Rocket Front)", "startPos = " + startPosition.name());
					break;
				case 8:
					autonomousCommand = new DrivePathfinder("Middle1RocketF", true, true); //new DrivePathfinder("Middle1RocketF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 2 (Rocket Front)", "startPos = " + startPosition.name());
					break;
				case 9:
					autonomousCommand = new DrivePathfinder("Right1RocketF", true, true); //new DrivePathfinder("Right1RocketF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 2 (Rocket Front)", "startPos = " + startPosition.name());
					break;
				case 10:
					autonomousCommand = new DrivePathfinder("Right2RocketF", true, true); //new DrivePathfinder("Right2RocketF", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 2 (Rocket Front)", "startPos = " + startPosition.name());
					break;
				case 11:
					autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left1RocketB", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 3 (Rocket Back)", "startPos = " + startPosition.name());
					break;
				case 12:
					autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left2RocketB", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 3 (Rocket Back)", "startPos = " + startPosition.name());
					break;
				case 13:
					autonomousCommand = new DrivePathfinder("Middle1RocketB", true, true); //new DrivePathfinder("Middle1RocketB", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 3 (Rocket Back)", "startPos = " + startPosition.name());
					break;
				case 14:
					autonomousCommand = new DrivePathfinder("Right1RocketB", true, true); //new DrivePathfinder("Right1RocketB", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 3 (Rocket Back)", "startPos = " + startPosition.name());
					break;
				case 15:
					autonomousCommand = new DrivePathfinder("Right2RocketB", true, true); //new DrivePathfinder("Right2RocketB", true, true);
					Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 3 (Rocket Back)", "startPos = " + startPosition.name());
					break;
			}
			SmartDashboard.putString("Auto path", autonomousCommand.getName());
			SmartDashboard.putString("Auto plan selected", autoPlan.name());
			SmartDashboard.putString("Auto start position", startPosition.name());	
		}


		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();

		if (color == DriverStation.Alliance.Blue) {
			SmartDashboard.putBoolean("Alliance Color", true);
		} else {
			SmartDashboard.putBoolean("Alliance Color", false);
		}
	}
}
