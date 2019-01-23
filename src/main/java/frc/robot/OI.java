/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.IntakeMovement;
import frc.robot.commands.IntakePID;
import frc.robot.commands.IntakePull;
import frc.robot.commands.IntakePush;
import frc.robot.commands.PistonCommandGroup;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.StopIntakeMovement;
import frc.robot.triggers.TriggerReset;
import frc.robot.triggers.TriggerStopIntake;

public class OI {

  // Joystick
  
  Joystick JOYSTICK_ONE = new Joystick(RobotMap.JOYSTICK_ONE);

  // Joystick buttons
  JoystickButton INTAKE_PULL = new JoystickButton(JOYSTICK_ONE, 1);
  JoystickButton INTAKE_PUSH = new JoystickButton(JOYSTICK_ONE, 4);
  JoystickButton INTAKE_PID = new JoystickButton(JOYSTICK_ONE, 2);
  JoystickButton INTAKE_UP = new JoystickButton(JOYSTICK_ONE, 2);
  JoystickButton INTAKE_DOWN = new JoystickButton(JOYSTICK_ONE, 2);
  JoystickButton INTAKE_SOLONOID = new JoystickButton(JOYSTICK_ONE, 3);

  // Triggers
  TriggerStopIntake triggerStopIntake = new TriggerStopIntake();
  TriggerReset TriggerReset = new TriggerReset();

  /**
   * Initialize all the intake components and set all joystick buttons.
   */
  public OI() {

    INTAKE_PUSH.whileActive(new IntakePush(-1));
    INTAKE_PULL.whileActive(new IntakePull(1));
    INTAKE_PID.whenPressed(new IntakePID(1, 0.5));
    INTAKE_SOLONOID.whenPressed(new PistonCommandGroup());
    TriggerReset.whenActive(new ResetEncoder());
    triggerStopIntake.whenActive(new StopIntakeMovement());
    INTAKE_UP.whenPressed(new IntakeMovement(0.5));
    INTAKE_DOWN.whenPressed(new IntakeMovement(-0.3));
  }

}
