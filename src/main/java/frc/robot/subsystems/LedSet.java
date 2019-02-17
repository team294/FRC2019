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
  private boolean ifBlink;
  private boolean blinkOn;
  private final Solenoid PneumaticLedsBlue = new Solenoid(RobotMap.pneumaticLedsBlue);
  private final Solenoid PneumaticLedsRed = new Solenoid(RobotMap.pneumaticLedsRed);
  private final Solenoid PneumaticLedsGreen = new Solenoid(RobotMap.pneumaticLedsGreen);

  

    public LedSet(){
        ledRelay = new Relay(RobotMap.ledRelay);
    }

    //LED SET METHOD

    public void LEDSet1(int colorValue) {
      this.colorValue = colorValue;
      if (colorValue<0 || colorValue>3) {
        this.colorValue = 0;
      }
      ifBlink = false;

      if(colorValue == 0){
        setOff();
      } else if(colorValue == 1){
        setOff();
        setGreen();
      } else if (colorValue == 2){
        setOff();
        setBlue();
      } else if(colorValue == 3){
        setOff();
        setRed();
      }else{
      }

    }
    public void LEDSet1(int colorValue, boolean ifBlink) {
      this.colorValue = colorValue;
      this.ifBlink = ifBlink;
      if (colorValue<0 || colorValue>3) {
        this.colorValue = 0;
      } if (colorValue==0 && ifBlink==true) {
        this.ifBlink = false;
      }
      
      if(!ifBlink) {
        if(colorValue == 0){
          setOff();
        } else if(colorValue == 1){
          setOff();
          setGreen();
        } else if (colorValue == 2){
          setOff();
          setBlue();
        } else if(colorValue == 3){
          setOff();
          setRed();
        }else{
        }
      }
      
    
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

  @Override
	public void periodic() {
    if (ifBlink) {
      currentTime = System.currentTimeMillis();
        if(currentTime > initialTime + 500){
          if(blinkOn == true){
            blinkOn = false;
          } else {
            blinkOn = true;
          }
          currentTime = System.currentTimeMillis();
          initialTime = System.currentTimeMillis();
        }
        if(blinkOn == true){
          setOff();
        } else {
          if(colorValue == 1){
            //Robot.leds.setOff();
            setGreen();
          } else if (colorValue == 2){
            //Robot.leds.setOff();
            setBlue();
          } else if(colorValue == 3){
            //Robot.leds.setOff();
            setRed();
          }else{
          }
        }
      }
  
    }
  }
