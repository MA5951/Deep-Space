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
import frc.robot.commands.IntakeMovment;
import frc.robot.commands.IntakePID;
import frc.robot.commands.IntakePull;
import frc.robot.commands.IntakePush;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.StopIntakeMovement;
import frc.robot.commands.PistonCommandGroup;
import frc.robot.triggers.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  //TODO
    Joystick JOYSTICKMOTORS = new Joystick(0);
    JoystickButton INTAKE_PULL = new JoystickButton(JOYSTICKMOTORS, 1);
    JoystickButton INTAKE_PUSH = new JoystickButton(JOYSTICKMOTORS, 4);
    JoystickButton INTAKE_PID = new JoystickButton(JOYSTICKMOTORS, 2);
    JoystickButton INTAKE_UP = new JoystickButton(JOYSTICKMOTORS, 2);
    JoystickButton INTAKE_DOWN = new JoystickButton(JOYSTICKMOTORS, 2);
    JoystickButton INTAKE_SOLONOID = new JoystickButton(JOYSTICKMOTORS, 3);

    TriggerStopIntake triggerStopIntake = new TriggerStopIntake();
    TriggerReset TriggerReset = new TriggerReset();
 
    public OI() {
      
    INTAKE_PUSH.whileActive(new IntakePush(-1));
    INTAKE_PULL.whileActive(new IntakePull(1));
    INTAKE_PID.whenPressed(new IntakePID(1, 0.5));
    INTAKE_SOLONOID.whenPressed(new PistonCommandGroup());
    TriggerReset.whenActive(new ResetEncoder());
    triggerStopIntake.whenActive(new StopIntakeMovement());
    INTAKE_UP.whenPressed(new IntakeMovment(0.5));
    INTAKE_DOWN.whenPressed(new IntakeMovment(-0.3));
  }

}
 