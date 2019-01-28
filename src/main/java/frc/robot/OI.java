/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.intake.*;

import frc.robot.triggers.TriggerReset;
import frc.robot.triggers.StopIntake;

/**
 * Maps commands to buttons/POVs/triggers
 */
public class OI {
  public static final Joystick OPERATOR_STICK = new Joystick(RobotMap.JOYSTICK_OPERATOR);
  public static final Joystick LEFT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_LEFT);
  public static final Joystick RIGHT_DRIVER_STICK = new Joystick(RobotMap.JOYSTICK_DRIVER_RIGHT);
  /*
  private JoystickButton elevatorUp = new JoystickButton(OPERATOR_STICK, XBOX.A);
  private JoystickButton elevatorDown = new JoystickButton(OPERATOR_STICK, XBOX.Y);
  private JoystickButton elevatorPIDUp = new JoystickButton(OPERATOR_STICK, XBOX.B);
  private JoystickButton elevatorPIDDown = new JoystickButton(OPERATOR_STICK, XBOX.X);
  
  private POVTrigger riderAngleDown = new POVTrigger(XBOX.POV_DOWN, OPERATOR_STICK);
  private POVTrigger riderAngleUp = new POVTrigger(XBOX.POV_UP, OPERATOR_STICK);
  private JoystickButton riderPID = new JoystickButton(OPERATOR_STICK, XBOX.START);
  private JoystickButton riderOuttake = new JoystickButton(OPERATOR_STICK, XBOX.LB);
  private JoystickButton riderIntake = new JoystickButton(OPERATOR_STICK, XBOX.RB);
  */
  
  // Joystick buttons
  private JoystickButton intakePull = new JoystickButton(OPERATOR_STICK, 1);
  private JoystickButton intakePush = new JoystickButton(OPERATOR_STICK, 2);
  private JoystickButton intakePID = new JoystickButton(OPERATOR_STICK, 5);
  private JoystickButton intakeUp = new JoystickButton(OPERATOR_STICK, 3);
  private JoystickButton intakeDown = new JoystickButton(OPERATOR_STICK, 4);
  private JoystickButton intakeSolenoid = new JoystickButton(OPERATOR_STICK, 6);

  private StopIntake triggerStopIntake = new StopIntake();
  private TriggerReset resetIntake = new TriggerReset();

  /**
   * Initialize all the intake components and set all joystick buttons.
   */
  public OI() {

    //TODO
    /* 
    elevatorPIDDown.whenPressed(new ElevatorPID(40, 0.5));
    elevatorPIDUp.whenPressed(new ElevatorPID(50, 0.5));
    elevatorUp.whenPressed(new ElevatorUp());
    elevatorDown.whenPressed(new ElevatorDown());

    
    riderIntake.whenPressed(new RiderIntake());
    riderOuttake.whenPressed(new RiderOuttake());
    riderPID.whenPressed(new RiderPID(500, 0.5));
    riderAngleDown.whenActive(new AngleRider(1, 1000, 10));
    riderAngleUp.whenActive(new AngleRider(-1, 1000, 10));
    */


    // CHECK BEFORE EVERY RUN
    intakePush.whileHeld(new IntakeMoveBall(-1));
   // intakePull.whileHeld(new IntakePull(1));
    intakePID.whileHeld(new IntakePID(1, 0.5)); // TODO Comment out this
    intakeSolenoid.whileHeld(new PistonCommandGroup());
    resetIntake.whenActive(new ResetEncoder()); 
   // triggerStopIntake.whileActive(new StopIntakeMovement()); TODO I think using default command + whileheld is better. 
    intakeUp.whileHeld(new IntakeMovement(0.5));
    intakeDown.whileHeld(new IntakeMovement(-0.3));
  }
}
