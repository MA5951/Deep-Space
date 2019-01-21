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
  Intake dodoSubsystem= Intake.getInstance();
  private double Tolerance;
  private double setSetpoint;
  public IntakePID(double setSetpoint,double Tolerance) {
    this.Tolerance=Tolerance;
    this.setSetpoint=setSetpoint;
    dodoSubsystem.setSetpointPID(setSetpoint);
    dodoSubsystem.tolerancePID(Tolerance);
 requires(dodoSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dodoSubsystem.enablePID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return dodoSubsystem.isOnTargetPID()||dodoSubsystem.isLImitSwhichOnDown();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    dodoSubsystem.disablePID();
    dodoSubsystem.dodoUpAndDonwLeftControl(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    dodoSubsystem.dodoUpAndDonwLeftControl(0);
  }
}
