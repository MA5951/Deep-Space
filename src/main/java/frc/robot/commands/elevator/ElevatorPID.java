/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPID extends Command {
  double setpoint;
  double absoluteTolerance;
  Elevator elevator;

  public ElevatorPID(double setSetpoint, double setAbsoluteTolerance) {
    this.setpoint = setSetpoint;
    this.absoluteTolerance = setAbsoluteTolerance; // TODO No need for tolerance as perameter
    elevator = Elevator.getInstance();
    requires(elevator);
    elevator.setAbsoluteTolerance(setAbsoluteTolerance);
    elevator.setSetPoint(setSetpoint);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    elevator.enablePID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elevator.isOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.disablePID();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    elevator.disablePID();
  }
}
