/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.awt.Button;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.commands.IntakePID;
import frc.robot.commands.IntakePull;
import frc.robot.commands.IntakePush;
import frc.robot.commands.ResatEncoder;
import frc.robot.commands.SolonoidIntake;
import frc.robot.commands.TrigerResat;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Joystick JOYSTICKMOTORS = new Joystick(0);
    JoystickButton INTAKE_PULL = new JoystickButton(JOYSTICKMOTORS, 1);
    JoystickButton INTAKE_PUSH = new JoystickButton(JOYSTICKMOTORS, 4);
    JoystickButton INTAKE_PID = new JoystickButton(JOYSTICKMOTORS, 2);
    JoystickButton INTAKE_SOLONOID = new JoystickButton(JOYSTICKMOTORS, 3);
    TrigerResat TrigerResat = new TrigerResat();
  public OI() {
    INTAKE_PUSH.whileActive(new IntakePush(-1));
    INTAKE_PULL.whileActive(new IntakePull(1));
    INTAKE_PID.whenPressed(new IntakePID(1, 0.5));
    INTAKE_SOLONOID.whenPressed(new SolonoidIntake());
    TrigerResat.whenActive(new ResatEncoder());
  }

}
 