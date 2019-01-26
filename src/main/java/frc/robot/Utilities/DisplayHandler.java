/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class DisplayHandler {

    private NetworkTableEntry xEntry;
    private NetworkTableEntry yEntry;
    private NetworkTableEntry driverControl;

    public DisplayHandler() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("display");

        xEntry = table.getEntry("x"); // X and Y used for exmaples only at this time
        yEntry = table.getEntry("y");

        driverControl = table.getEntry("driverControl");


    }

    public void updateX(double x) {
        xEntry.setDouble(x);
    }

    public void updateY(double y) {
        yEntry.setDouble(y);
    }

    public void updateControl(boolean driver) {
        driverControl.setBoolean(driver);
    }
}
