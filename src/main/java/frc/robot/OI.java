/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.elevator.ElevatorControlSpeedY;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.elevator.ElevatorUp;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public static final Joystick OPERATOR_STICK = new Joystick(RobotMap.JOYSTICK_MOTORS);

  public static final JoystickButton ELEVATOR_UP = new JoystickButton(OPERATOR_STICK, 1);
  public static final JoystickButton ELEVATOR_CONTROL_JOYSTICK = new JoystickButton(OPERATOR_STICK, 3);
  public static final JoystickButton ELEVATOR_DOWN = new JoystickButton(OPERATOR_STICK, 2);
  public static final JoystickButton ELEVATOR_PID_UP = new JoystickButton(OPERATOR_STICK, 4);
  public static final JoystickButton ELEVATOR_PID_DOWN = new JoystickButton(OPERATOR_STICK, 5);

  public OI() {
    ELEVATOR_PID_DOWN.whenPressed(new ElevatorPID(40, 0.5));
    ELEVATOR_PID_UP.whenPressed(new ElevatorPID(50, 0.5));
    ELEVATOR_CONTROL_JOYSTICK.whileHeld(new ElevatorControlSpeedY());
    ELEVATOR_UP.whenPressed(new ElevatorUp());
    ELEVATOR_DOWN.whenPressed(new ElevatorDown());
  }
}
