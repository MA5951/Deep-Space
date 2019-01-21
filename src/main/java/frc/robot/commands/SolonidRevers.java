/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;
/**
 * Add your docs here.
 */

public class SolonidRevers extends InstantCommand {

  Intake intakeSubsystem = Intake.getInstance();

  /**
   * Add your docs here.
   */
  public SolonidRevers() {
 requires(intakeSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    intakeSubsystem.RelayControlRevers();
  }

}
