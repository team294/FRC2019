package frc.robot.Utilities;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;

public class LineFollowing {
    private final DigitalInput lineFollower1 = new DigitalInput(RobotMap.lineFollower1);
    private final DigitalInput lineFollower2 = new DigitalInput(RobotMap.lineFollower2);
    private final DigitalInput lineFollower3 = new DigitalInput(RobotMap.lineFollower3);

    public LineFollowing() {
    }
    /**
     * Returns true if object is found
     * @param follower int between 1 and 3 inclusive that tells which lineFollower to get data from
     **/
    public boolean isLinePresent(int follower) {
        if (follower == 1) return lineFollower1.get();
        else if (follower == 2) return lineFollower2.get();
        else {
            return lineFollower3.get();
        }
    }

    public void updateLineFollowerLog() { //TODO update data later
        Robot.log.writeLog("LineFollower", "Update Variables", "Some data");
    }
}
