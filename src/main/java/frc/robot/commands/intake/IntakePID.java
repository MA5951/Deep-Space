/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

@Deprecated
public class IntakePID extends Command {

  private Intake intake = Intake.getInstance();
  private double setpoint;
  private double lastTimeOnTarget;
  private double waiteTime;
  /**
   * Creates new {IntakePID} command.
   * 
   * @param setpoint  The given destination.
   * @param tolerance The given range.
   */
  public IntakePID(double setpoint , double waiteTime ) {
    this.waiteTime=waiteTime;
    this.setpoint = setpoint;

    requires(intake);
  }

  /**
   * Call the {setSetpointPID} function, call the {setTolerancePID} function and
   * enables the intake PIDController.
   */
  @Override
  protected void initialize() {

    intake.setSetpoint(setpoint);
    intake.enablePID(true);
  }

  @Override
  protected void execute() {
  }

  /**
   * Check whether intake PIDController is on target.
   */
  @Override
  protected boolean isFinished() {
    if(!intake.isPIDOnTarget()){
      lastTimeOnTarget=Timer.getFPGATimestamp();
    }
    return intake.isPIDOnTarget() &&  Timer.getFPGATimestamp()-lastTimeOnTarget > waiteTime;
  }
  

  /**
   * Disable the intake PIDContoller and disable the {intakeMovmentControl} motor
   * if {isFinished} function return true.
   */
  @Override
  protected void end() {
    intake.enablePID(false);
    intake.intakeAngleControl(0);
    
  }

  /**
   * Disable the intake PIDController and disable the {intakeMovmentControl} motor
   * if {end} function
   */
  @Override
  protected void interrupted() {
    intake.enablePID(false);
    intake.intakeAngleControl(0);
  }
}
