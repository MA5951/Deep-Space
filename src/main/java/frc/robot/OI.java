/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.rider.AngleRider;
import frc.robot.commands.rider.IntakeRiderPull;
import frc.robot.commands.rider.IntakeRiderPush;
import frc.robot.commands.rider.RiderPID;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public static Joystick joystickMotors=new Joystick(RobotMap.JOYSTIC_MOTORS);
  public static final JoystickButton MAMUTA_CONTROL_ANGLE_DOWN=new JoystickButton(joystickMotors, 6);
  public static final JoystickButton MAMUTA_CONTROL_ANGLE_UP=new JoystickButton(joystickMotors, 10);
  public static final JoystickButton MAMUTA_PID=new JoystickButton(joystickMotors, 7);
  public static final JoystickButton MAMUTA_INTAKE_PULL=new JoystickButton(joystickMotors, 8);
  public static final JoystickButton MAMUTA_INTAKE_IN=new JoystickButton(joystickMotors, 9);

public OI(){
  MAMUTA_INTAKE_IN.whenPressed(new IntakeRiderPull());
  MAMUTA_INTAKE_PULL.whenPressed(new IntakeRiderPush());
  MAMUTA_PID.whenPressed(new RiderPID(500, 0.5));
  MAMUTA_CONTROL_ANGLE_DOWN.whenPressed(new AngleRider(1, -1000));
  MAMUTA_CONTROL_ANGLE_UP.whenPressed(new AngleRider(-1, 1000));
  
 

}
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
