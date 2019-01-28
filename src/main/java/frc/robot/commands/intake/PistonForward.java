package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class PistonForward extends InstantCommand {

  private Intake intake = Intake.getInstance();

  /**
   * Requires the Intake Chassis.
   */
  public PistonForward() {
    requires(intake);
  }

  /**
   * Run the {RelayControlFowerd} function.
   */
  @Override
  protected void initialize() {
    intake.PistonControlForward();
  }

}
