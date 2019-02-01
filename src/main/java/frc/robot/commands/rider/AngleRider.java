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
  double speed; // TODO no need for member

  private Rider rider = Rider.getInstance();

  public AngleRider() {

    requires(rider);
  }

  /**
   * Give power to the angle motor
   */
  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {
    speed = OI.OPERATOR_STICK.getRawAxis(5);
    rider.controlAngleMotor(speed);
  }

  /**
   * If the current angle is the desire one, the getCurrentAngle will disable
   */
  @Override
  protected boolean isFinished() { // TODO default command is always false, implement SAFETY in execute and interrupted
    return false; // speed < 0 && rider.isLimitSwitchAngleDownPressed() || speed > 0 && rider.isLimitSwitchAngleUpPressed();
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
