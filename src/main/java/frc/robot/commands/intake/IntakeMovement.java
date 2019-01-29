/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.subsystems.Intake;

public class IntakeMovement extends Command {

  private Intake intake = Intake.getInstance();
  Double speed;

  /**
   * Creates new {IntakeMovement} command.
   * 
   * @param speedUpAndDown the given power of {intakeMovmentControl} motor.
   */
  public IntakeMovement(Double speed) {
    this.speed = speed;

    requires(intake);
  }

  /**
   * Give power to the {intakeMovmentControl} motor.
   */
  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {
    intake.intakeAngleControl(speed);
  }

  /**
   * Check whether limit switch is pressed.
   */
  @Override
  protected boolean isFinished() {
    return (speed > 0 && intake.isLimitSwitchUpPressed() || speed < 0 && intake.isLimitSwitchDownPressed());
  }

  /**
   * Disable the {intakeMovmentControl} function if {isFinished} function return
   * true.
   */
  @Override
  protected void end() {
    intake.intakeAngleControl(0);
  }

  /**
   * Disable the {intakeMovmentControl} function if {end} function was
   * interrupted.
   */
  @Override
  protected void interrupted() {
    intake.intakeAngleControl(0);
  }
}
