/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

/**
 * The ports of the chassis components
 */
public class RobotMap {

  public static final int JOYSTICK_OPERATOR = 0;
  public static final int JOYSTICK_DRIVER_LEFT = 1;
  public static final int JOYSTICK_DRIVER_RIGHT = 2;

  // Elevator
  // TODO
  public static final int ELEVATOR_TALON = 1;

  public static final int ELEVATOR_ENCODER_A = 2;
  public static final int ELEVATOR_ENCODER_B = 3;

  public static final int ELEVATOR_SWITCH_DOWN_LEFT = 4;
  public static final int ELEVATOR_SWITCH_DOWN_RIGHT = 5;
  public static final int ELEVATOR_SWITCH_UP_LEFT = 7;
  public static final int ELEVATOR_SWITCH_UP_RIGHT = 6;

  // Chassis
  // TODO
  public static final int CHASSIS_LEFT_FRONT = 1;
  public static final int CHASSIS_LEFT_REAR = 1;

  public static final int CHASSIS_RIGHT_FRONT = 1;
  public static final int CHASSIS_RIGHT_REAR = 1;

  public static final int CHASSIS_RIGHT_ENCODER_A = 1;
  public static final int CHASSIS_RIGHT_ENCODER_B = 1;

  public static final int CHASSIS_LEFT_ENCODER_A = 1;
  public static final int CHASSIS_LEFT_ENCODER_B = 1;

  // Rider
  // TODO
  public static final int RIDER_ANGLE_MOTOR = 1;
  public static final int RIDER_INTAKE_MOTOR = 1;

  public static final int RIDER_ENCODER_A = 1;
  public static final int RIDER_ENCODER_B = 1;

  public static final int RIDER_ANGLE_LIMIT_SWITCH = 0;
  public static final int RIDER_INTAKE_LIMIT_SWITCH = 0;
}
