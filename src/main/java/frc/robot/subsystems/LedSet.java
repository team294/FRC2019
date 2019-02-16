/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Date;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LedSet extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final Relay ledRelay;

  private long currentTime = System.currentTimeMillis();
  private long initialTime = System.currentTimeMillis();
  private boolean lightRed = false;
  private int colorValue = 0;
  private final Solenoid PneumaticLedsBlue = new Solenoid(RobotMap.pneumaticLedsBlue);
  private final Solenoid PneumaticLedsRed = new Solenoid(RobotMap.pneumaticLedsRed);
  private final Solenoid PneumaticLedsGreen = new Solenoid(RobotMap.pneumaticLedsGreen);
    public LedSet(){
        ledRelay = new Relay(RobotMap.ledRelay);
    }

    /**
     * turn on LED lights
     */

    public void setBlue(){
        PneumaticLedsBlue.set(true);
        System.out.println("Blue");
    }
    
    public void setGreen(){
      
      PneumaticLedsGreen.set(true);
      System.out.println("Green");
    }

    public void setOff(){
      PneumaticLedsGreen.set(false);
      PneumaticLedsBlue.set(false);
      PneumaticLedsRed.set(false);
    }
    public void setRed(){
      PneumaticLedsRed.set(true);
      System.out.println("Red");
    }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
