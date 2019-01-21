/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.elevator.ElevatorJoystickControl;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.elevator.ElevatorUp;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public static final Joystick OPERATOR_STICK = new Joystick(RobotMap.JOYSTICK_MOTORS);

  private JoystickButton elevatorUp = new JoystickButton(OPERATOR_STICK, 1);
  private JoystickButton elevatorJoystickControl = new JoystickButton(OPERATOR_STICK, 3);
  private JoystickButton elevatorDown = new JoystickButton(OPERATOR_STICK, 2);
  private JoystickButton elevatorPIDUp = new JoystickButton(OPERATOR_STICK, 4);
  private JoystickButton elevatorPIDDown = new JoystickButton(OPERATOR_STICK, 5);

  public OI() {
    elevatorPIDDown.whenPressed(new ElevatorPID(40, 0.5));
    elevatorPIDUp.whenPressed(new ElevatorPID(50, 0.5));
    elevatorJoystickControl.whileHeld(new ElevatorJoystickControl());
    elevatorUp.whenPressed(new ElevatorUp());
    elevatorDown.whenPressed(new ElevatorDown());
  }
}
