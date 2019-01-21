package frc.robot.Utilities;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    public void displayLineSensors(){
        SmartDashboard.putBoolean("Left LineFollower", Robot.lineFollowing.isLinePresent(1));
        SmartDashboard.putBoolean("Middle LineFollower", Robot.lineFollowing.isLinePresent(2));
        SmartDashboard.putBoolean("Right LineFollower", Robot.lineFollowing.isLinePresent(3));
    }

    public int lineNumber() {

        //@param returns lineNumber
        /*
        0 1 2
        l c r Output
        0 0 0 0
        1 1 0 1
        1 0 0 2
        0 1 1 -1
        0 0 1 -2
        */
        
        int lineNumber = 0;

        if (Robot.lineFollowing.isLinePresent(1) == true && Robot.lineFollowing.isLinePresent(2) == true && Robot.lineFollowing.isLinePresent(3) == false) {
        lineNumber = 1; // 1 1 0 : Turn left slight?
        } else if (Robot.lineFollowing.isLinePresent(1) == false && Robot.lineFollowing.isLinePresent(2) == true && Robot.lineFollowing.isLinePresent(3) == true) {
        lineNumber = -1; // 0 1 1 : Turn right slight?
        } else if (Robot.lineFollowing.isLinePresent(1) == false && Robot.lineFollowing.isLinePresent(2) == false && Robot.lineFollowing.isLinePresent(3) == true) {
        lineNumber = -2; // 0 0 1 : Turn left? 
        } else if (Robot.lineFollowing.isLinePresent(1) == true && Robot.lineFollowing.isLinePresent(2) == false && Robot.lineFollowing.isLinePresent(3) == false) {
        lineNumber = 2; // 1 0 0 : Turn right?
        } else if (Robot.lineFollowing.isLinePresent(1) == false && Robot.lineFollowing.isLinePresent(2) == true && Robot.lineFollowing.isLinePresent(3) == false) {
        lineNumber = 0; // 0 1 0 : Straight
        } else {
            lineNumber = 3; // 0 0 0 ; 1 1 1 ; 1 0 1 : Stop
        }
        return lineNumber;
    }

    public void updateLineFollowerLog() { //TODO update data later
        Robot.log.writeLog("LineFollower", "Update Variables", "Some data");
    }


}
