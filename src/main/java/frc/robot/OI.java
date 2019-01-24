/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.climber.OpenPole;
import frc.robot.triggers.ClimbingPoleDown;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //public static final ClimbingPoleDown CLIMBING_POLE_DOWN = new ClimbingPoleDown();
  public static final Joystick CLIMBER_JOYSTICK = new Joystick(1);
  public static final JoystickButton CLIMBER_BUTTON = new JoystickButton(CLIMBER_JOYSTICK, 3);

  public OI() {
    //CLIMBING_POLE_DOWN.whenActive(new OpenPole());
    CLIMBER_BUTTON.whileHeld(new OpenPole());
  }
}
