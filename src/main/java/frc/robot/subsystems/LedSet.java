/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LedSet extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final Relay ledRelay;

    public LedSet(){
        ledRelay = new Relay(RobotMap.ledRelay);
    }

    /**
     * turn on LED lights
     */
    public void setOff(){
        ledRelay.set(Relay.Value.kOn);
    }

    public void setRed(){
        ledRelay.set(Relay.Value.kReverse);
    }
    
    public void setBlue(){
        ledRelay.set(Relay.Value.kForward);
    }

    public void setPurple(){
        ledRelay.set(Relay.Value.kOff);
    }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
