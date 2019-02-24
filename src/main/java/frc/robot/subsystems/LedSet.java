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

  private long currentTime = System.currentTimeMillis();
  private long initialTime = System.currentTimeMillis();
  private boolean lightRed = false;
  private int colorValue = 0;
  private int strobeCount = 0;
  private boolean ifBlink;
  private boolean ifStrobe = false;
  private boolean blinkOn;
  private boolean ifPoPo = false;
  private final Solenoid PneumaticLedsBlue = new Solenoid(RobotMap.pneumaticLedsBlue);
  private final Solenoid PneumaticLedsRed = new Solenoid(RobotMap.pneumaticLedsRed);
  private final Solenoid PneumaticLedsGreen = new Solenoid(RobotMap.pneumaticLedsGreen);

  

    public LedSet(){
    }

    //LED SET METHOD

    public void LEDSet(int colorValue) {
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
    /*
    0 = Off
    1 = Green
    2 = Blue
    3 = Red
    4 = Purple
    5 = Sky Blue
    6 = Lime
    7 = White (Don't use for more than a second) */

    public void LEDSet(int colorValue, boolean ifBlink) {
      this.colorValue = colorValue;
      this.ifBlink = ifBlink;
      if (colorValue<0 || colorValue>7) {
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
        } else if(colorValue == 4){
          setOff();
          setPurple();
        } else if(colorValue == 5){
          setOff();
          setSkyBlue();
        } else if(colorValue == 6){
          setOff();
          setLime();
        } else if(colorValue == 7){
          setOff();
          setWhite();
        }else{
        }
      }
    }

    public void disco() {
      ifStrobe = true;
    }

    /**
     * turn on LED lights
     */

    public void setBlue(){
        PneumaticLedsBlue.set(true);
        System.out.println("Blue");
    }

    public void weeWoo() {
      ifPoPo = true;
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
    // Dont use too long
    public void setPurple(){
      PneumaticLedsRed.set(true);
      PneumaticLedsBlue.set(true);
      System.out.println("Purple");
    }
    // Dont use too long
    public void setLime(){
      PneumaticLedsRed.set(true);
      PneumaticLedsGreen.set(true);
      System.out.println("Lime");
    }
    // Dont use too long
    public void setSkyBlue(){
      PneumaticLedsGreen.set(true);
      PneumaticLedsBlue.set(true);
      System.out.println("SkyBlue");
    }
    // <!WARNING!> <!WARNING!> <!WARNING!> <!WARNING!>
    public void setWhite(){
      PneumaticLedsRed.set(true);
      PneumaticLedsBlue.set(true);
      PneumaticLedsGreen.set(true);
      System.out.println("White");
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
            setGreen();
          } else if (colorValue == 2){
            setBlue();
          } else if(colorValue == 3){
            setRed();
          } else if(colorValue == 4){
            setPurple();
          } else if(colorValue == 5){
            setSkyBlue();
          } else if(colorValue == 6){
            setLime();
          } else if(colorValue == 7){
            setWhite();
          }else{
          }
        }
      }
      if(ifStrobe) {
        currentTime = System.currentTimeMillis();
        if(currentTime > initialTime + 69){
          if(strobeCount == 0){
            strobeCount = 1;
          } else if (strobeCount == 1){
            strobeCount = 2;
          } else if (strobeCount == 2){
            strobeCount = 3;
          }  else if (strobeCount == 3){
            strobeCount = 4;
          } else if (strobeCount == 4){
            strobeCount = 5;
          } else if (strobeCount == 5){
            strobeCount = 0;
          } else {
          }
          currentTime = System.currentTimeMillis();
          initialTime = System.currentTimeMillis();
        } 
        if(strobeCount == 0){
          setOff();
          setRed();
        } else if (strobeCount == 1){
          setOff();
          setGreen();
        } else if(strobeCount == 2){
          setOff();
          setBlue();
        }else if(strobeCount == 3){
          setOff();
          setPurple();
        }else if(strobeCount == 4){
          setOff();
          setLime();
        }
        else if(strobeCount == 5){
          setOff();
          setSkyBlue();
        }
        else{
        }
      }
      if (ifPoPo) {
        if(blinkOn == true){
          blinkOn = false;
        } else {
          blinkOn = true;
        }
        if(blinkOn == true){
          setOff();
          setRed();
        } else {
          setOff();
          setBlue();
        }
      }
    }
  }
