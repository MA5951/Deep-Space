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


public class IntakePID extends Command {

  private Intake intake = Intake.getInstance();
  private double setpoint;
  //private double tolerance;
  private double lastTimeOnTarget;
  private double waitTime;

  /**
   * Creates new {IntakePID} command.
   * 
   * @param setpoint The given destination.
   */
  public IntakePID(double setpoint, double waitTime) {
    this.waitTime = waitTime;
    this.setpoint = setpoint;
    //this.tolerance = tolerance;

    requires(intake);
  }

  /**
   * Call the {setSetpointPID} function, call the {setTolerancePID} function and
   * enables the intake PIDController.
   */
  @Override
  protected void initialize() {
    intake.enablePID(true);
    intake.setSetpoint(setpoint);
  }

  @Override
  protected void execute() {
  }

  /**
   * Check whether intake PIDController is on target.
   */
  @Override
  protected boolean isFinished() {
    if (!intake.isPIDOnTarget()) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    }
    return 
    intake.isPIDOnTarget() && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
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
