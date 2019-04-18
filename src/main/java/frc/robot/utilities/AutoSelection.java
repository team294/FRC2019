package frc.robot.utilities;

import frc.robot.Robot;
import frc.robot.commands.DrivePathfinder;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelection {
	private Command autonomousCommand;
	private int priorProgramSelected = -1;

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
		String strProgram = "";
		
		StartingPosition startPosition = Robot.oi.readStartPosition();
		autoPlan = Robot.oi.readAutoPlan();

		if (autoPlan == null || startPosition == null) return;

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

		// Don't generate a new command if the selection has not changed
		if (programSelected == priorProgramSelected) return;

		priorProgramSelected = programSelected;
		SmartDashboard.putString("Auto path", "loading path file...");
		Robot.log.writeLogEcho("AutoSelection", "Load path init", "Plan," + autoPlan.name() + ",startPos," + startPosition.name() + ",program selected," + programSelected);

		switch (programSelected) {
			// case 0:
			// 	autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left1Ship", true, true);
			// 	strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
			// 	// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
			// 	break;
			case 1:
				autonomousCommand = new DrivePathfinder("Left2CargoF", true, true); //new DrivePathfinder("Left2ShipF", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			case 2:
				autonomousCommand = new DrivePathfinder("Middle1CargoF", true, true); //new DrivePathfinder("Middle1ShipF", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			case 3:
				autonomousCommand = new DrivePathfinder("Right1Cargo", true, true); //new DrivePathfinder("Right1ShipF", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			case 4:
				autonomousCommand = new DrivePathfinder("Right2CargoF", true, true); //new DrivePathfinder("Right2ShipF", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			// case 5:
			// 	autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left1RocketF", true, true);
			// 	strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
			// 	// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
			// 	break;
			case 6:
				autonomousCommand = new DrivePathfinder("Left2RocketF", true, true); //new DrivePathfinder("Left2RocketF", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			// case 7:
			// 	autonomousCommand = new DrivePathfinder("Middle1RocketF", true, true); //new DrivePathfinder("Middle1RocketF", true, true);
			// 	strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
			// 	// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
			// 	break;
			case 8:
				autonomousCommand = new DrivePathfinder("Right1RocketF", true, true); //new DrivePathfinder("Right1RocketF", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			case 9:
				autonomousCommand = new DrivePathfinder("Right2RocketF", true, true); //new DrivePathfinder("Right2RocketF", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			// case 10:
			// 	autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left1RocketB", true, true);
			// 	strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
			// 	// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
			// 	break;
			// case 11:
			// 	autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Left2RocketB", true, true);
			// 	strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
			// 	// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
			// 	break;
			// case 12:
			// 	autonomousCommand = new DrivePathfinder("Middle1RocketB", true, true); //new DrivePathfinder("Middle1RocketB", true, true);
			// 	strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
			// 	// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
			// 	break;
			case 13:
				autonomousCommand = new DrivePathfinder("Right1RocketB", true, true); //new DrivePathfinder("Right1RocketB", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			case 14:
				autonomousCommand = new DrivePathfinder("Right2RocketB", true, true); //new DrivePathfinder("Right2RocketB", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				// Robot.log.writeLogEcho("AutoSelection", "Ran Auto Path 1 (CargoShip Front)", "startPos = " + startPosition.name());
				break;
			default:
				autonomousCommand = new DrivePathfinder("Straight150", true, true); //new DrivePathfinder("Right2RocketB", true, true);
				strProgram = ((DrivePathfinder)autonomousCommand).getPathName();
				break;
		}
		
		Robot.log.writeLogEcho("AutoSelection", "Load path", "Plan," + autoPlan.name() + ",startPos," + startPosition.name() + ",File name," + strProgram);

		SmartDashboard.putString("Auto path", strProgram);
		SmartDashboard.putString("Auto plan selected", autoPlan.name());
		SmartDashboard.putString("Auto start position", startPosition.name());	
		
	}

	public void startAutoCommand() {
		Robot.log.writeLogEcho("AutoSelection", "Start Command", "Init");
		if (autonomousCommand != null) {
			Robot.log.writeLogEcho("AutoSelection", "Start Command", "Command Name," + autonomousCommand.getName());
			autonomousCommand.start();
		}
	}
}
