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
    // Intake wheels port
    // TODO
    public static final int INTAKE_MOTORS_WHEELS = 1;

    //Angle motors ports
    //TODO
    public static final int INTAKE_MOTORS_ANGLE_A = 2;
    public static final int INTAKE_MOTORS_ANGLE_B = 3;

    //Pistons ports
    //TODO
    public static final int RELAY_PISTON_LEFT = 4;
    public static final int RELAY_PISTON_RIGHT = 5;

    //Intake encoders ports
    //TODO
    public static final int ENCODER_A_INTAKE = 2;
    public static final int ENCODER_B_INTAKE = 1;

    //limit switches ports
    //TODO
    public static final int LIMIT_SWITCH_UP = 1;
    public static final int LIMIT_SWITCH_DOWN = 2;

    //joystick port
    //TODO
    public static final int JOYSTICK_ONE = 1;
}