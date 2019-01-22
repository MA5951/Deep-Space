/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePID extends Command {

  private Intake intake = Intake.getInstance();
  private double tolerance;
  private double setpoint;

  /**
   * 
   * @param setpoint  destination to robot for PID
   * @param tolerance tolerance
   */
  public IntakePID(double setpoint, double tolerance) {

    this.tolerance = tolerance;
    this.setpoint = setpoint;

    requires(intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    intake.setSetpointPID(setpoint);
    intake.setTolerancePID(tolerance);
    intake.enablePID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return intake.isOnTargetPID();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.disablePID();
    intake.intakeMovmentControl(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.disablePID();
    intake.intakeMovmentControl(0);
  }
}
