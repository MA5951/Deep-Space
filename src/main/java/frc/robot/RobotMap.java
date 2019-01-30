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

  // Joysticks
  public static final int JOYSTICK_OPERATOR = 0;
  public static final int JOYSTICK_DRIVER_LEFT = 1;
  public static final int JOYSTICK_DRIVER_RIGHT = 2;

  
  // Motors
  public static final int CHASSIS_LEFT_FRONT = 1;
  public static final int CHASSIS_LEFT_REAR = 2;
  public static final int CHASSIS_RIGHT_FRONT = 3;
  public static final int CHASSIS_RIGHT_REAR = 4;

  public static final int INTAKE_MOTORS_ANGLE_A = 5;
  public static final int INTAKE_MOTORS_ANGLE_B = 6;
  public static final int INTAKE_MOTORS_WHEELS = 7;

  public static final int RIDER_ANGLE_MOTOR = 8;
  public static final int RIDER_INTAKE_MOTOR = 9;
  
  public static final int ELEVATOR_TALON = 10;


  // Sensors
  public static final int CHASSIS_RIGHT_ENCODER_A = 0;
  public static final int CHASSIS_RIGHT_ENCODER_B = 1;
  public static final int CHASSIS_LEFT_ENCODER_A = 2;
  public static final int CHASSIS_LEFT_ENCODER_B = 3;

  public static final int INTAKE_ENCODER_A = 4;
  public static final int INTAKE_ENCODER_B = 5;
  public static final int INTAKE_LIMIT_SWITCH_UP = 6;
  public static final int INTAKE_LIMIT_SWITCH_DOWN = 7;

  public static final int ELEVATOR_ENCODER_A = 8;
  public static final int ELEVATOR_ENCODER_B = 9;
  public static final int ELEVATOR_SWITCH_DOWN_LEFT = 10;
  public static final int ELEVATOR_SWITCH_DOWN_RIGHT = 11;
  public static final int ELEVATOR_SWITCH_UP_LEFT = 12;
  public static final int ELEVATOR_SWITCH_UP_RIGHT = 13;

  public static final int RIDER_ENCODER_A = 14;
  public static final int RIDER_ENCODER_B = 15;
  public static final int RIDER_ANGLE_LIMIT_SWITCH = 16;
  public static final int RIDER_INTAKE_LIMIT_SWITCH = 17;
  public static final int RIDER_PROXIMITY_SENSOR = 18;

  // Pneumatic
  public static final int PCM = 0;

  public static final int INTAKE_PISTON_FORWARD = 0;
  public static final int INTAKE_PISTON_BACKWARD = 1;

}
