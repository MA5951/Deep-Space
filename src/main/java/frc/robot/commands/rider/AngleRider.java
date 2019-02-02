/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Rider;

public class AngleRider extends Command {
  double speed;

  private Rider rider = Rider.getInstance();

  public AngleRider() {

    requires(rider);
  }

  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {
    speed = OI.OPERATOR_STICK.getRawAxis(5);
    rider.controlAngleMotor(speed);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  /**
   * Reset the encoder and disable the angle motor if isFinished function is true
   */
  @Override
  protected void end() {
    rider.controlAngleMotor(0);
  }

  /**
   * Disable the angle motor if end function was interrupted
   */
  @Override
  protected void interrupted() {
    rider.controlAngleMotor(0);
  }
}
