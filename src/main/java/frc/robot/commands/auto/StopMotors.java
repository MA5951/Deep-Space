/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorControl;
import frc.robot.subsystems.Rider;

/**
 * Add your docs here.
 */
public class StopMotors extends InstantCommand {
  /**
   * Add your docs here.
   */
  public StopMotors() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Rider.getInstance());
    requires(Elevator.getInstance());
    requires(Intake.getInstance());
    requires(OperatorControl.getInstance());
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
  }
}
