/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.IntakeCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public OI() {
    Joystick Joystick1 = new Joystick(0);
    JoystickButton intake_speed = new JoystickButton(Joystick1, 1);
    JoystickButton intake_position = new JoystickButton(Joystick1, 3);


    intake_speed.whileActive(new IntakeCommand());
    intake_position.whileActive(new IntakeCommand());

  
  }

}
 