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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.AutoFrontCargoCommand;
import frc.robot.commands.auto.AutomaticFrontCargo;
import frc.robot.commands.auto.AutomaticMoveToPanel;
import frc.robot.commands.auto.AutomaticTakeBallCommand;
import frc.robot.commands.auto.ChangeCamera;
import frc.robot.commands.auto.MoveToHatchPanelPosition;
import frc.robot.commands.auto.ReturnToDefault;
import frc.robot.commands.auto.ReturnToDefaultCommand;
import frc.robot.commands.auto.Rocket1;
import frc.robot.commands.auto.Rocket1Command;
import frc.robot.commands.auto.StopMotors;
import frc.robot.commands.climber.climberDown;
import frc.robot.commands.climber.climberUp;
import frc.robot.commands.elevator.ResetElevatorEncoder;
import frc.robot.commands.intake.IntakeMovement;
import frc.robot.commands.intake.PistonForward;
import frc.robot.commands.intake.PistonOff;
import frc.robot.commands.intake.PullBall;
import frc.robot.commands.intake.PushBall;
import frc.robot.commands.intake.ResetIntakeEncoder;
import frc.robot.commands.rider.ResetRiderEncoder;
import frc.robot.commands.rider.TeleopRiderIntakeControl;
import frc.robot.triggers.IntakPushTrigger;
import frc.robot.triggers.IntakePullTrigger;
import frc.robot.triggers.POVTrigger;
import frc.robot.triggers.PreventFall;
import frc.robot.triggers.ResetElevatorEncoderTrigger;
import frc.robot.triggers.ResetIntakeEncoderTrigger;
import frc.robot.triggers.ResetRiderEncoderTrigger;
import frc.robot.util.JoystickUtil.XBOX;

/**
 * Maps commands to buttons/POVs/triggers
 */
public class OI {
  public static final XboxController OPERATOR_STICK = new XboxController(RobotMap.JOYSTICK_OPERATOR);
  public static final Joystick LEFT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_LEFT);
  public static final Joystick RIGHT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_RIGHT);

  private POVTrigger autoRocket1 = new POVTrigger(OPERATOR_STICK, XBOX.POV_DOWN);
  private POVTrigger climberXbox = new POVTrigger(OPERATOR_STICK, XBOX.POV_RIGHT);
  private POVTrigger autoFrontCargo = new POVTrigger(OPERATOR_STICK, XBOX.POV_UP);
  private POVTrigger PIDVisonTarget = new POVTrigger(OPERATOR_STICK, XBOX.POV_LEFT);

  private JoystickButton camera1 = new JoystickButton(RIGHT_DRIVER_STICK, 2);
  //private POVTrigger camera2 = new POVTrigger(RIGHT_DRIVER_STICK, XBOX.POV_RIGHT);
  //private POVTrigger camera3 = new POVTrigger(RIGHT_DRIVER_STICK, XBOX.POV_UP);

  private JoystickButton gotoDefault = new JoystickButton(OPERATOR_STICK, XBOX.B);
  private JoystickButton riderOuttake = new JoystickButton(OPERATOR_STICK, XBOX.START);

  private JoystickButton autoHatchPanel = new JoystickButton(OPERATOR_STICK, XBOX.Y);
  private JoystickButton autoIntake = new JoystickButton(OPERATOR_STICK, XBOX.A);
  private JoystickButton StopMotorsJoyStickRight = new JoystickButton(OPERATOR_STICK, XBOX.STICK_RIGHT);
  private JoystickButton StopMotorsJoyStickLeft = new JoystickButton(OPERATOR_STICK, XBOX.STICK_LEFT);
  private JoystickButton moveIntakeUp = new JoystickButton(OPERATOR_STICK, XBOX.RB);
  private JoystickButton moveIntakeDown = new JoystickButton(OPERATOR_STICK, XBOX.LB);
  private JoystickButton intakeSolenoid = new JoystickButton(OPERATOR_STICK, XBOX.X);
  // private JoystickButton climber = new JoystickButton (RIGHT_DRIVER_STICK , 4);

  private ResetElevatorEncoderTrigger resetElevatorEncoder = new ResetElevatorEncoderTrigger();
  private ResetIntakeEncoderTrigger resetIntakeEncoder = new ResetIntakeEncoderTrigger();
  private ResetRiderEncoderTrigger resetRiderEncoder = new ResetRiderEncoderTrigger();
  private IntakePullTrigger IntakePullTrigger = new IntakePullTrigger();
  private IntakPushTrigger IntakePushTrigger = new IntakPushTrigger();
  // private PreventFall PreventFall = new PreventFall();

  public OI() {
    StopMotorsJoyStickRight.whileActive(new StopMotors());
    StopMotorsJoyStickLeft.whileActive(new StopMotors());
    moveIntakeDown.whileHeld(new IntakeMovement(0.5));
    moveIntakeUp.whileHeld(new IntakeMovement(-0.5));
    intakeSolenoid.whileHeld(new PistonForward());
    intakeSolenoid.whenReleased(new PistonOff());
    autoIntake.whileActive(new AutomaticTakeBallCommand());
    autoHatchPanel.whenActive(new MoveToHatchPanelPosition());
    gotoDefault.whileActive(new ReturnToDefaultCommand());
    autoRocket1.whileActive(new Rocket1Command());
    autoFrontCargo.whileActive(new AutoFrontCargoCommand());
   // PIDVisonTarget.whileActive(new AutomaticMoveToPanel());

    camera1.whenActive(new ChangeCamera());
  

    IntakePullTrigger.whileActive(new PullBall());
    IntakePushTrigger.whileActive(new PushBall());
    riderOuttake.whileHeld(new TeleopRiderIntakeControl(-1));

    // climber.whileHeld(new climberUp());
    // climber.whenReleased(new climberDown());

    climberXbox.whileActive(new climberUp());
    climberXbox.whenInactive(new climberDown());

    resetElevatorEncoder.whenActive(new ResetElevatorEncoder());
    resetIntakeEncoder.whenActive(new ResetIntakeEncoder());
    resetRiderEncoder.whenActive(new ResetRiderEncoder());
    // PreventFall.whenActive(new IntakePID(-900, 0.1));
  }

}
