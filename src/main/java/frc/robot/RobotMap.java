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
public static final int INTAKE_MOTORS_WILDES = 1;
public static final int  INTAKE_MOTORS_ANGLE_LEFT = 2;
public static final int  INTAKE_MOTORS_ANGLE_RIGHT = 3;
public static final int  RELAY_SOLONID_LEFT=4;
public static final int  RELAY_SOLONID_RIGHT=5;
public static final int ENCODER_A_INTAKE=2;
public static final int ENCODER_B_INTAKE=1;
public static final int LINIT_SWHICH_UP=1;
public static final int LINIT_SWHICH_DOWN=2;
public static final int Joystick1= 1;
}