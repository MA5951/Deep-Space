/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.AllAutomaticIntake;
import frc.robot.commands.auto.AutomaticFrontCargo;
import frc.robot.commands.auto.AutomaticTakeBall;
import frc.robot.commands.auto.AutomaticTakeHatchPanel;
import frc.robot.commands.auto.MoveToHatchPanelPosition;
import frc.robot.commands.auto.MoveBall;
import frc.robot.commands.auto.ReturnToDefault;
import frc.robot.commands.auto.Rocket1;
import frc.robot.commands.auto.StopMotors;
import frc.robot.commands.climber.ClosePole;
import frc.robot.commands.climber.OpenPole;
import frc.robot.commands.elevator.ElevatorEncoderControlMOtors;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.elevator.ElevatorUp;
import frc.robot.commands.elevator.ResetElevatorEncoder;
import frc.robot.commands.intake.IntakeMoveBall;
import frc.robot.commands.intake.IntakeMovement;
import frc.robot.commands.intake.IntakePull;
import frc.robot.commands.intake.IntakePush;
import frc.robot.commands.intake.PistonCommandGroup;
import frc.robot.commands.intake.PistonForward;
import frc.robot.commands.intake.PistonOff;
import frc.robot.commands.intake.PushBall;
import frc.robot.commands.intake.ResetIntakeEncoder;
import frc.robot.commands.intake.pullBall;
import frc.robot.commands.rider.DisablePID;
import frc.robot.commands.rider.HoldRiderPID;
import frc.robot.commands.rider.MoveAngle;
import frc.robot.commands.rider.ResetRiderEncoder;
import frc.robot.commands.rider.RiderIntake;
import frc.robot.commands.rider.RiderOuttake;
import frc.robot.commands.rider.RiderPID;
import frc.robot.commands.rider.TeleopRiderOuttake;
import frc.robot.commands.rider.setPIDF;
import frc.robot.triggers.DisablesPIDTrigger;
import frc.robot.triggers.IntakPushTrigger;
import frc.robot.triggers.IntakePullTrigger;
import frc.robot.triggers.POVTrigger;
import frc.robot.triggers.ResetElevatorEncoderTrigger;
import frc.robot.triggers.ResetIntakeEncoderTrigger;
import frc.robot.triggers.ResetRiderEncoderTrigger;
import frc.robot.triggers.enablePIDTrigger;
import frc.robot.triggers.setNegativeF;
import frc.robot.triggers.setPositiveF;
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

  private POVTrigger autoRocket1 = new POVTrigger(OPERATOR_STICK, XBOX.POV_DOWN);
  private POVTrigger autoTakeHatchPanel = new POVTrigger(OPERATOR_STICK, XBOX.POV_RIGHT);
  private POVTrigger autoFrontCargo = new POVTrigger(OPERATOR_STICK, XBOX.POV_UP);

  private JoystickButton gotoDefault = new JoystickButton(OPERATOR_STICK, XBOX.B);
  private JoystickButton riderOuttake = new JoystickButton(OPERATOR_STICK, XBOX.START);

  private JoystickButton autoHatchPanel = new JoystickButton(OPERATOR_STICK, XBOX.Y);
  private JoystickButton autoIntake = new JoystickButton(OPERATOR_STICK, XBOX.A);
  private JoystickButton StopMotors = new JoystickButton(OPERATOR_STICK, XBOX.STICK_LEFT);
  private JoystickButton moveIntakeUp = new JoystickButton(OPERATOR_STICK, XBOX.RB);
  private JoystickButton moveIntakeDown = new JoystickButton(OPERATOR_STICK, XBOX.LB);
  private JoystickButton intakeSolenoid = new JoystickButton(OPERATOR_STICK, XBOX.X);
  private JoystickButton CLimber = new JoystickButton(OPERATOR_STICK, XBOX.BACK);
  

  private ResetElevatorEncoderTrigger resetElevatorEncoder = new ResetElevatorEncoderTrigger();
  private ResetIntakeEncoderTrigger resetIntakeEncoder = new ResetIntakeEncoderTrigger();
  private ResetRiderEncoderTrigger resetRiderEncoder = new ResetRiderEncoderTrigger();
  private IntakePullTrigger IntakePullTrigger = new IntakePullTrigger(); 
  private IntakPushTrigger IntakePushTrigger= new IntakPushTrigger();
  


  public OI() {
    StopMotors.whileHeld(new StopMotors());
    moveIntakeDown.whileHeld(new IntakeMovement(-0.5));
    moveIntakeUp.whileHeld(new IntakeMovement(0.5));
    intakeSolenoid.whileHeld(new PistonForward());
    intakeSolenoid.whenReleased(new PistonOff());

    autoIntake.whileHeld(new AutomaticTakeBall());
    autoHatchPanel.whileHeld(new MoveToHatchPanelPosition());
    gotoDefault.whileHeld(new ReturnToDefault());
    autoRocket1.whileActive(new Rocket1());
    autoTakeHatchPanel.whileActive(new AutomaticTakeHatchPanel());
    autoFrontCargo.whenActive(new AutomaticFrontCargo());

    riderOuttake.whileHeld(new TeleopRiderOuttake());
    CLimber.whileActive(new RiderPID(-700, 0.5, 15));
    // intakePullBall.whileHeld(new RiderIntake());
    // intakePullBall.whileHeld(new IntakeMoveBall(-0.5d));
    // intakePullBall.whenPressed(new RiderPID(-680 , 0.5));
    //intakePullBall.whenPressed(new ElevatorUp());


    IntakePullTrigger.whileActive(new pullBall());

    IntakePushTrigger.whenActive(new PushBall());

    riderOuttake.whileHeld(new TeleopRiderOuttake());
    
    resetElevatorEncoder.whenActive(new ResetElevatorEncoder());
    resetIntakeEncoder.whenActive(new ResetIntakeEncoder());
    resetRiderEncoder.whenActive(new ResetRiderEncoder());
 
  }

}
