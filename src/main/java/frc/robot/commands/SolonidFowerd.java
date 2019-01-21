package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;
/**
 * Add your docs here.
 */

public class SolonidFowerd extends InstantCommand {

  Intake intakeSubsystem = Intake.getInstance();

  /**
   * Add your docs here.
   */
  public SolonidFowerd() {
 requires(intakeSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    intakeSubsystem.RelayControlFowerd();
  }

}
