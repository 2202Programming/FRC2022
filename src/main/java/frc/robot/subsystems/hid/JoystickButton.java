// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class JoystickButton extends Trigger {
  private final GenericHID m_joystick;
  private final int m_buttonNumber;
  BooleanSupplier foo;
  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickButton(GenericHID joystick, int buttonNumber) {
    super( () -> get(joystick, buttonNumber) );
    m_joystick = joystick;    
    m_buttonNumber = buttonNumber;
  }


  /**
   * Gets the value of the joystick button.
   * (old way was for deprecated Button class )
   * @return The value of the joystick button
   */
  @Deprecated
  public boolean get() {
    return m_joystick.getRawButton(m_buttonNumber);
  }

  // new way - Trigger BooleanSupplier
  public static boolean get(GenericHID js, int buttonNum) {
    return js.getRawButton(buttonNum);
  }

}
