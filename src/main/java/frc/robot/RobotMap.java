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
public class RobotMap {

  public static final int JOYSTICK_OPERATOR = 0;
  public static final int JOYSTICK_DRIVER_LEFT = 1;
  public static final int JOYSTICK_DRIVER_RIGHT = 2;

  // Elevator
  // TODO
  public static final int ELEVATOR_TALON = 888;

  public static final int ELEVATOR_ENCODER_A = 887;
  public static final int ELEVATOR_ENCODER_B = 886;

  public static final int ELEVATOR_SWITCH_DOWN_LEFT = 885;
  public static final int ELEVATOR_SWITCH_DOWN_RIGHT = 884;
  public static final int ELEVATOR_SWITCH_UP_LEFT = 883;
  public static final int ELEVATOR_SWITCH_UP_RIGHT = 882;

  // Chassis
  public static final int CHASSIS_LEFT_FRONT = 1;
  public static final int CHASSIS_LEFT_REAR = 2;

  public static final int CHASSIS_RIGHT_FRONT = 3;
  public static final int CHASSIS_RIGHT_REAR = 4;

  public static final int CHASSIS_RIGHT_ENCODER_A = 0;
  public static final int CHASSIS_RIGHT_ENCODER_B = 1;

  public static final int CHASSIS_LEFT_ENCODER_A = 2;
  public static final int CHASSIS_LEFT_ENCODER_B = 3;

  // Rider
  // TODO
  public static final int RIDER_ANGLE_MOTOR = 999;
  public static final int RIDER_INTAKE_MOTOR = 998;

  public static final int RIDER_ENCODER_A = 997;
  public static final int RIDER_ENCODER_B = 996;

  public static final int RIDER_ANGLE_LIMIT_SWITCH = 995;
  public static final int RIDER_INTAKE_LIMIT_SWITCH = 994;

  // Intake
  public static final int INTAKE_MOTORS_WHEELS = 7;

  public static final int INTAKE_MOTORS_ANGLE_A = 5;
  public static final int INTAKE_MOTORS_ANGLE_B = 6;

  public static final int PCM = 5987;

  public static final int INTAKE_PISTON_RIGHT_FORWARD = 2;
  public static final int INTAKE_PISTON_RIGHT_BACKWARD = 2;
  public static final int INTAKE_PISTON_LEFT_FORWARD = 0;
  public static final int INTAKE_PISTON_LEFT_BACKWARD = 1;

  public static final int INTAKE_ENCODER_A = 4;
  public static final int INTAKE_ENCODER_B = 5;

  public static final int INTAKE_LIMIT_SWITCH_UP = 6;
  public static final int INTAKE_LIMIT_SWITCH_DOWN = 7;
}
