/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPID extends Command {
  private double setPoint;
  private Elevator elevator;
  private double lastTimeOnTarget;
  private double waitTime;

  public ElevatorPID(double setPoint, double waitTime) {
    this.setPoint = setPoint;
    this.waitTime = waitTime;

    elevator = Elevator.getInstance();
    requires(elevator);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    elevator.enablePID(true);
    elevator.setSetPoint(setPoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (!elevator.isPIDOnTarget()) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    }
    return elevator.isPIDOnTarget() && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.enablePID(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    elevator.enablePID(false);
  }
}
