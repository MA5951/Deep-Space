/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Rider.AngleInwards;
import frc.robot.commands.Rider.AngleOutward;
import frc.robot.commands.Rider.IntakeIn;
import frc.robot.commands.Rider.IntakeOut;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public static final Joystick JOYSTICK_OPERATOR = new Joystick(RobotMap.JOYSTICK);

  public static final Button ANGLE_BUTTON_INWARD = new JoystickButton(JOYSTICK_OPERATOR, RobotMap.ANGLE_MOTOR_JOYSTICK_BUTTON_INWARD);
  public static final Button ANGLE_BUTTON_OUTWARD = new JoystickButton(JOYSTICK_OPERATOR, RobotMap.ANGLE_MOTOR_JOYSTICK_BUTTON_OUTWARD);

  public static final Button INTAKE_BUTTON_IN = new JoystickButton(JOYSTICK_OPERATOR, RobotMap.INTAKE_MOTOR_JOYSTICK_BUTTON_IN);
  public static final Button INTAKE_BUTTON_OUT = new JoystickButton(JOYSTICK_OPERATOR, RobotMap.INTAKE_MOTOR_JOYSTICK_BUTTON_OUT);
  
 

public OI() {

   //ANGLE_BUTTON_INWARD.whileActive(new AngleInwards());
   //ANGLE_BUTTON_OUTWARD.whileActive(new AngleOutward());

   INTAKE_BUTTON_IN.whileActive(new IntakeIn());
   INTAKE_BUTTON_OUT.whileActive(new IntakeOut());



  }
}
