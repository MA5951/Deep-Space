/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeMovement extends Command {

  private Intake intake = Intake.getInstance();
  private double speedUpAndDown;

  public IntakeMovement(double speedUpAndDown) {

    this.speedUpAndDown = speedUpAndDown;
    requires(intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    intake.intakeMovmentControl(speedUpAndDown);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (speedUpAndDown > 0 && intake.isLimitSwitchUp()) || (speedUpAndDown > 0 && intake.isLimitSwitchDown());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.intakeMovmentControl(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.intakeMovmentControl(0);
  }
}
