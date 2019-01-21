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
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  public static final int JOYSTIC_MOTORS=1;
     //Motors for mamuta (rider)
public static final int ANGLE_MOTOR = 1;
public static final int INTAKE_MOTOR = 1;

//Encoders for mamuta (rider)
public static final int ENCODER_PORT_RIDER_ONE = 1;
public static final int ENCODER_PORT_RIDER_TWO = 1;

//JoySticks
public static final int JOYSTICK = 1;

//??
public static final int ANGLE_MOTOR_JOYSTICK_BUTTON_INWARD = 1;
public static final int ANGLE_MOTOR_JOYSTICK_BUTTON_OUTWARD = 2;

//??
public static final int INTAKE_MOTOR_JOYSTICK_BUTTON_IN = 3;
public static final int INTAKE_MOTOR_JOYSTICK_BUTTON_OUT = 4;

//LimitSwitches for the mamuta (rider)
public static final int RIDER_LIMIT_SWITCH_ANGLE_PORT = 0;
public static final int RIDER_LIMIT_SWITCH_INTAKE_PORT=0;
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  //Motors for mamuta (rider)

}

