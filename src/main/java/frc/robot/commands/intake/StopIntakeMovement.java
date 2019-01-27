/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NO NEED FOR THIS, USE IN END OR INTERRUPTED
@Deprecated
public class StopIntakeMovement extends InstantCommand {

  private Intake intake = Intake.getInstance();

  /**
   * Requires the Intake subsystem.
   */
  public StopIntakeMovement() {
    requires(intake);
  }

  /**
   * Disables the {intakeMovmentControl}.
   */
  @Override
  protected void initialize() {
    intake.intakeMovmentControl(0);
  }

}
