
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Add your docs here.
 */
public class LimitSwitchOverride extends InstantCommand {
  Intake intake = Intake.getInstance();

  public static boolean enable = true;

  public LimitSwitchOverride() {
    super();
    enable = true;
    requires(intake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    enable = !enable;
    intake.limitswitchOverride(enable);
  }

}