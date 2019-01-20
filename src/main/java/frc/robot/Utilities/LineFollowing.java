package frc.robot.Utilities;

import frc.robot.Robot;
import frc.robot.RobotMap;

import java.lang.Math.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class LineFollowing {
    public final DigitalInput lineFollowerLeft = new DigitalInput(RobotMap.lineFollowerLeft);
    public final DigitalInput lineFollowerCenter = new DigitalInput(RobotMap.lineFollowerCenter);
    public final DigitalInput lineFollowerRight = new DigitalInput(RobotMap.lineFollowerRight);

    public LineFollowing() {
    }
    /**
     * Returns true if object is found
     * @param follower int between 1 and 3 inclusive that tells which lineFollower to get data from
     **/
    public boolean isLinePresent(int follower) {
        if (follower == 1) return !(lineFollowerLeft.get());
        else if (follower == 2) return !(lineFollowerCenter.get());
        else {
            return !(lineFollowerRight.get());
        }
    }

    public void updateLineFollowerLog() { //TODO update data later
        Robot.log.writeLog("LineFollower", "Update Variables", "Some data");
    }
}
