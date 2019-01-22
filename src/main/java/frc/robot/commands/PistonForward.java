package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Add your docs here.
 */

public class PistonForward extends InstantCommand {

  private Intake intake = Intake.getInstance();

  /**
   * Add your docs here.
   */
  public PistonForward() {
    requires(intake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    intake.RelayControlFowerd();
  }

}
