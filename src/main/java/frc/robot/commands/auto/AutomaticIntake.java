/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

public class AutomaticIntake extends Command {
  private Intake intake = Intake.getInstance();
  private double minDistance;
  private double maxDistance;
  private double speed;

  public AutomaticIntake(double maxDistance, double minDistance, double speed) {
    this.minDistance = minDistance;
    this.maxDistance = maxDistance;
    this.speed = Math.abs(speed);
    requires(intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (intake.getEncoder() > ((maxDistance + minDistance) / 2)) {
      intake.intakeAngleControl(-speed);
    } else {
      intake.intakeAngleControl(speed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return intake.isEncoderInDistanceRangeIntake(maxDistance, minDistance);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.intakeAngleControl(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.intakeAngleControl(0);
  }
}
