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

    //TalonSRX ports
    //TODO
    public static final int LEFT_MOTOR_ONE = 1;
    public static final int LEFT_MOTOR_TWO = 1;
    public static final int LEFT_MOTOR_THREE = 1;
    public static final int RIGHT_MOTOR_ONE = 1;
    public static final int RIGHT_MOTOR_TWO = 1;
    public static final int RIGHT_MOTOR_THREE = 1;

    // Encoders ports
    //TODO
    public static final int ENCODER_RIGHT_A = 1;
    public static final int ENCODER_RIGHT_B = 1;
    public static final int ENCODER_LEFT_A = 1;
    public static final int ENCODER_LEFT_B = 1;

    //Joysticks ports
    //TODO
    public static final int JOYSTICK_CHASSIS_LEFT_MOTOR =1;
    public static final int JOYSTICK_CHASSIS_RIGHT_MOTOR =1;

    
}
