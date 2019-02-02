/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

/**
 * Intakes from the Rider.
 */
public class RiderIntake extends Command {
  private Rider rider = Rider.getInstance();

  public RiderIntake() {
    requires(rider);
  }

  /**
   * Give power to the intake motors
   */
  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {
    rider.controlIntakeMotor(-1);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  /**
   * disable the intake motor if isFinished is function is true
   */
  @Override
  protected void end() {
    rider.controlIntakeMotor(0);
  }

  /**
   * disable the intake motor if a function was interrupted
   */
  @Override
  protected void interrupted() {
    rider.controlIntakeMotor(0);
  }
}
