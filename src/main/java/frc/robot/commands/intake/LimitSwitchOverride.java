package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Intake;

/**
 * Add your docs here.
 */
public class LimitSwitchOverride extends InstantCommand {
  Intake intake = Intake.getInstance();

  private static boolean enable = true;
  public static boolean robot = enable;

  public LimitSwitchOverride() {
    super();
    enable = true;
    robot = enable;
    requires(intake);

  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    enable = !enable;
    robot = enable;
    intake.limitswitchOverride(enable);
  }

}