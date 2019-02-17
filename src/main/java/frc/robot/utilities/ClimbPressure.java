package frc.robot.utilities;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbPressure{

    public final DigitalInput vacuumSwitch = new DigitalInput(3);

    public ClimbPressure(){

    }
    /*
    * Returns true if pressure is low enough to initiate climb
    */
    public boolean vacuumPresent(){
        return vacuumSwitch.get();
    }

    public void displayVacuum(){
        SmartDashboard.putBoolean("Vacuum Switch", Robot.climbPressure.vacuumPresent());
    }
}