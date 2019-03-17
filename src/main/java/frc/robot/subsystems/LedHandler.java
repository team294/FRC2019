/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class LedHandler extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private long currentTime = System.currentTimeMillis();
  private long initialTime = System.currentTimeMillis();

  private Color color = Color.OFF;
  private int blinkWait = 100; // Wait time in ms between blinks
  private boolean isBlink = false; 
  private boolean blinkOn = false; // Iterator for on/off cycles of blinks

  private final Solenoid blue = new Solenoid(RobotMap.pneumaticLedsBlue);
  private final Solenoid red = new Solenoid(RobotMap.pneumaticLedsRed);
  private final Solenoid green = new Solenoid(RobotMap.pneumaticLedsGreen);

  /**
   * Set the color of the LEDs
   * @param color only LedHandler.Color enum values accepted (NOT Java.awt.Color)
   * @param blink true to blink on/off, false for solid color. 
   * Time between blinks can be set with setBlinkWait()
   */
  public void setColor(Color color, boolean blink) {
    if (this.color != color || (isBlink && !blink)) { // avoid a bit of flicker when setting the same color repeatedly
      this.color = color;
      if (!blink) setDefaultColor();
    }
    isBlink = blink;
  }

  /**
   * Sets the solid color of the LEDs (for blinking, use setColor(color, blink))
   * @param color LedHandler.Color enum value
   */
  public void setColor(Color color) {
    setColor(color, false);
  }

  // Sets the LEDs to whatever the saved state already is
  // For use in periodic
  private void setDefaultColor() {

    setOff();

    switch (color) {
      case MAGENTA: blue.set(true);
      case RED: red.set(true);
        break;
      case CYAN: green.set(true);
      case BLUE: blue.set(true);
        break;
      case GREEN: green.set(true);
        break;
      default: // already set LEDs off
        break;
    }
  }

  /**
   * Sets the default wait time between blinks
   * @param waitTime time in ms (default is 100)
   */
  public void setBlinkWait(int waitTime) {
    blinkWait = waitTime;
  }

  /**
   * Sets all the LEDs off
   */
  public void setOff(){
    green.set(false);
    blue.set(false);
    red.set(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
	public void periodic() {
    if (isBlink) {
      currentTime = System.currentTimeMillis();

      // If it's past the wait time, switch whether the LEDs are on or off
      if (currentTime > initialTime + blinkWait) {
        initialTime = currentTime;

        // if on cycle, set the current color on. Otherwise, turn off LEDs
        if (blinkOn) {
          setOff();
        } else {
          setDefaultColor();
        }

        blinkOn = !blinkOn;
      }
    }
  }

  public enum Color {
    RED, BLUE, GREEN, CYAN, MAGENTA, OFF
  }
}