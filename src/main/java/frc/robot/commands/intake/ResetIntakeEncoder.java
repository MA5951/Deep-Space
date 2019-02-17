/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class ResetIntakeEncoder extends InstantCommand {

  private Intake intake = Intake.getInstance();

  /**
   * Requires the Intake subsystem
   */
  public ResetIntakeEncoder() {
    requires(intake);
  }

  /**
   * Reset the encoder
   */
  @Override
  protected void initialize() {
    intake.resetEncoder();
  }

}
