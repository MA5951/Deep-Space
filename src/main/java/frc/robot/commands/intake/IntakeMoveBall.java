/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeMoveBall extends Command {

  private Intake intake = Intake.getInstance();

  float speed;

  /**
   * Creates new {IntakePush} command.
   * 
   * @param speed The given speed of the {intakeControl}.
   */
  public IntakeMoveBall(float speed) {
    this.speed = speed;
    requires(intake);
  }

  @Override
  protected void initialize() {

  }

  /**
   * Give speed to {intakeControl}.
   */
  @Override
  protected void execute() {
    intake.intakeBallControl(speed);
  }

  /**
   * Makes the command run forever
   */
  @Override
  protected boolean isFinished() {
    return false;
  }

  /**
   * Diables the {intakeControl} if {isFinished} function return true.
   */
  @Override
  protected void end() {
    intake.intakeBallControl(0);
  }

  /**
   * Diables the {intakeControl} if {end} function was iterrupted.
   */
  @Override
  protected void interrupted() {
    intake.intakeBallControl(0);
  }
}
