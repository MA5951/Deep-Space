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
  public static final int CHASSIS_LEFT_FRONT = 3;
  public static final int CHASSIS_LEFT_REAR = 4;
  public static final int CHASSIS_RIGHT_FRONT = 1;
  public static final int CHASSIS_RIGHT_REAR = 2;

  public static final int INTAKE_MOTORS_ANGLE_LEFT_A = 5;
  public static final int INTAKE_MOTORS_ANGLE_RIGHT_B = 6;
  public static final int INTAKE_MOTORS_WHEELS = 7;

  public static final int RIDER_ANGLE_MOTOR = 10; // need change to 8 instad 10 or change the ID in the tuner 
  public static final int RIDER_INTAKE_MOTOR = 9;

  public static final int ELEVATOR_SPARK = 0;

  // Sensors (RoboRIO DIO's)
    
  public static final int INTAKE_ENCODER_A = 1;
  public static final int INTAKE_ENCODER_B =2;

  public static final int RIDER_ENCODER_A = 8;
  public static final int RIDER_ENCODER_B = 9;

  public static final int ELEVATOR_ENCODER_A =4;
  public static final int ELEVATOR_ENCODER_B =5;
  
  public static final int ELEVATOR_LIMITSWITCH_DOWN = 3;
  public static final int ELEVATOR_LIMITSWITCH_UP = 0;
  public static final int RIDER_LIMITSWITCH_BALL = 7;

  // Pneumatic
  public static final int PCM = 0;

  public static final int INTAKE_PISTON_FORWARD = 0;
  public static final int CLIMBIR_PISTON_FORWARD = 1;

 
}
