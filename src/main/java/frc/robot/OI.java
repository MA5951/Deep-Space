/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.intake.IntakeMovement;
import frc.robot.commands.intake.PistonCommandGroup;
import frc.robot.commands.intake.PullBall;
import frc.robot.commands.intake.PushBall;
import frc.robot.commands.rider.RiderIntake;
import frc.robot.commands.rider.RiderOuttake;
import frc.robot.util.JoystickUtil.XBOX;

//import frc.robot.triggers.TriggerReset;
//import frc.robot.util.JoystickUtil.XBOX;
//import frc.robot.triggers.LimitSwitchDownTrigger;
//import frc.robot.triggers.POVTrigger;

/**
 * Maps commands to buttons/POVs/triggers
 */
public class OI {
  public static final XboxController OPERATOR_STICK = new XboxController(RobotMap.JOYSTICK_OPERATOR);
  public static final Joystick LEFT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_LEFT);
  public static final Joystick RIGHT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_RIGHT);

  // private JoystickButton elevatorPIDUp = new JoystickButton(OPERATOR_STICK,
  // XBOX.B);
  // private JoystickButton elevatorPIDDown = new JoystickButton(OPERATOR_STICK,
  // XBOX.X);

  // private JoystickButton riderPID = new JoystickButton(OPERATOR_STICK,
  // XBOX.START);
  private JoystickButton riderOuttake = new JoystickButton(OPERATOR_STICK, XBOX.LB); // x
  private JoystickButton riderIntake = new JoystickButton(OPERATOR_STICK, XBOX.RB); // x

  // Joystick buttons
  private JoystickButton intakePullBall = new JoystickButton(OPERATOR_STICK, 1);
  private JoystickButton intakePushBall = new JoystickButton(OPERATOR_STICK, 4);
  // private JoystickButton intakePushBall = new JoystickButton(OPERATOR_STICK,
  // 3);
  // private JoystickButton intakePID = new JoystickButton(OPERATOR_STICK, 5);
  private JoystickButton moveIntakeUp = new JoystickButton(OPERATOR_STICK, 6);
  private JoystickButton moveIntakeDown = new JoystickButton(OPERATOR_STICK, 5);
  private JoystickButton intakeSolenoid = new JoystickButton(OPERATOR_STICK, 8);

  // private LimitSwitchDownTrigger triggerStopIntake = new
  // LimitSwitchDownTrigger();
  // private TriggerReset resetIntake = new TriggerReset();

  /**
   * Initialize all the intake components and set all joystick buttons.
   */
  public OI() {
    moveIntakeDown.whileHeld(new IntakeMovement(-0.5));
    moveIntakeUp.whileHeld(new IntakeMovement(0.5));
    intakeSolenoid.whenPressed(new PistonCommandGroup());

    intakePullBall.whileHeld(new PullBall());
    intakePushBall.whileHeld(new PushBall());

    riderIntake.whileHeld(new RiderIntake());
    riderOuttake.whileHeld(new RiderOuttake());

  }
}
