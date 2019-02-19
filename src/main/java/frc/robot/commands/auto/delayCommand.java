/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.OperatorControl;

public class delayCommand extends Command {
  private double time;

  public delayCommand(double time) {
    requires(OperatorControl.getInstance());
    this.time = time;
  }

  @Override
  protected void initialize() {
    this.setTimeout(time);
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return this.isTimedOut();
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
