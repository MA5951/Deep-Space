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
import frc.robot.commands.climber.ClosePole;
import frc.robot.commands.climber.OpenPole;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.elevator.ResetElevatorEncoder;
import frc.robot.commands.intake.IntakeMovement;
import frc.robot.commands.intake.IntakePID;
import frc.robot.commands.intake.PistonCommandGroup;
import frc.robot.commands.intake.PullBall;
import frc.robot.commands.intake.PushBall;
import frc.robot.commands.intake.ResetIntakeEncoder;
import frc.robot.commands.rider.ResetRiderEncoder;
import frc.robot.commands.rider.RiderIntake;
import frc.robot.commands.rider.RiderOuttake;
import frc.robot.commands.rider.RiderPID;
import frc.robot.triggers.POVTrigger;
import frc.robot.triggers.ResetElevatorEncoderTrigger;
import frc.robot.triggers.ResetIntakeEncoderTrigger;
import frc.robot.triggers.ResetRiderEncoderTrigger;
import frc.robot.util.JoystickUtil.JOYSTICK;
import frc.robot.util.JoystickUtil.XBOX;

//import frc.robot.triggers.TriggerReset;
//import frc.robot.util.JoystickUtil.XBOX;
//import frc.robot.triggers.POVTrigger;

/**
 * Maps commands to buttons/POVs/triggers
 */
public class OI {
  public static final XboxController OPERATOR_STICK = new XboxController(RobotMap.JOYSTICK_OPERATOR);
  public static final Joystick LEFT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_LEFT);
  public static final Joystick RIGHT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_RIGHT);

   // Joystick buttons and triggers
  private POVTrigger elevatorPIDUp = new POVTrigger(OPERATOR_STICK, XBOX.POV_UP);
  private POVTrigger elevatorPIDDown = new POVTrigger(OPERATOR_STICK, XBOX.POV_DOWN);

  private POVTrigger riderPIDRight = new POVTrigger(OPERATOR_STICK, XBOX.POV_RIGHT);
  private POVTrigger riderPIDLeft = new POVTrigger(OPERATOR_STICK, XBOX.POV_LEFT);
  private JoystickButton riderOuttake = new JoystickButton(OPERATOR_STICK, XBOX.A);
  private JoystickButton riderIntake = new JoystickButton(OPERATOR_STICK, XBOX.Y);

  private JoystickButton intakePullBall = new JoystickButton(OPERATOR_STICK, XBOX.A);
  private JoystickButton intakePushBall = new JoystickButton(OPERATOR_STICK, XBOX.Y);
  private JoystickButton intakePID = new JoystickButton(OPERATOR_STICK, XBOX.X);
  private JoystickButton moveIntakeUp = new JoystickButton(OPERATOR_STICK, XBOX.RB);
  private JoystickButton moveIntakeDown = new JoystickButton(OPERATOR_STICK, XBOX.LB);
  private JoystickButton intakeSolenoid = new JoystickButton(OPERATOR_STICK, XBOX.START);

  private JoystickButton climberOpenPole = new JoystickButton(RIGHT_DRIVER_STICK, 3);
  private JoystickButton climberStopPole = new JoystickButton(RIGHT_DRIVER_STICK, 5);

  private ResetElevatorEncoderTrigger resetElevatorEncoder = new ResetElevatorEncoderTrigger();
  private ResetIntakeEncoderTrigger resetIntakeEncoder = new ResetIntakeEncoderTrigger();
  private ResetRiderEncoderTrigger resetRiderEncoder = new ResetRiderEncoderTrigger();

  // private TriggerReset resetIntake = new TriggerReset();

  /**
   * Initialize all the intake components and set all joystick buttons.
   */
  public OI() {
    moveIntakeDown.whileHeld(new IntakeMovement(-0.5));
    moveIntakeUp.whileHeld(new IntakeMovement(0.5));
    intakeSolenoid.whenPressed(new PistonCommandGroup());

    intakePID.whileHeld(new IntakePID(0)); //TODO set setpoint
    intakePullBall.whileHeld(new PullBall());
    intakePushBall.whileHeld(new PushBall());

    riderPIDRight.whileActive(new RiderPID(0)); //TODO set setpoint
    riderPIDLeft.whileActive(new RiderPID(-0)); //TODO set setpoint
    riderIntake.whileHeld(new RiderIntake());
    riderOuttake.whileHeld(new RiderOuttake());

    climberOpenPole.whileHeld(new OpenPole());
    climberStopPole.whileHeld(new ClosePole());

    elevatorPIDUp.whileActive(new ElevatorPID(0)); //TODO set setpoint
    elevatorPIDDown.whileActive(new ElevatorPID(-0)); //TODO set setpoint

    resetElevatorEncoder.whenActive(new ResetElevatorEncoder());
    resetIntakeEncoder.whenActive(new ResetIntakeEncoder());
    resetRiderEncoder.whenActive(new ResetRiderEncoder());

    
  }

}
