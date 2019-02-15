/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Date;
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

  private long currentTime = System.currentTimeMillis();
  private long initialTime = System.currentTimeMillis();
  private boolean lightRed = false;
  private int colorValue = 0;

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
        ledRelay.set(Relay.Value.kForward);
    }
    
    public void setGreen(){
        ledRelay.set(Relay.Value.kReverse);
    }

    public void setYellow(){
        ledRelay.set(Relay.Value.kOff);
    }

    

    public void setBlink(int pColorValue){
        currentTime = System.currentTimeMillis();

        //switches the boolean between true and false every 1800 seconds
        //resets the time after every switch
        if(currentTime > initialTime + 800){
          if(lightRed == true){
            lightRed = false;
          } else {
            lightRed = true;
          }
          currentTime = System.currentTimeMillis();
          initialTime = System.currentTimeMillis();
        }

        if(lightRed == true){
          System.out.println("Setting off in blink");
            setOff();
        } else {
          if(colorValue == 1){
            setGreen();
          } else {
            setRed();
          }
          
        }
    }
    

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
