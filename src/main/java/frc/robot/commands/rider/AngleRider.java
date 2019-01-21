/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

public class AngleRider extends Command {
  private double speed;
  private double angle;
  private Rider rider = Rider.getInstance();
  public AngleRider(double speed, double angle) {
    this.angle = angle;
    this.speed = speed;
    requires(rider);
  }

  /**
   * Give power to the angle motor
   */
  @Override
  protected void initialize() {
    rider.controlAngleMoter(speed);
  }


  @Override
  protected void execute() {
  }

  /**
   * If the current angle is the desire one, the getCurrentAngle will disable 
   */
  @Override
  protected boolean isFinished() {
    return rider. getCurrentAngle(angle);
  }

  /**
   * Reset the encoder and disable the angle motor if isFinished function is true
   */
  @Override
  protected void end() {
    rider.encoderReset();
    rider.controlAngleMoter(0);
  }

  /**
   * Disable the angle motor if end function was interrupted
   */
  @Override
  protected void interrupted() {
    rider.controlAngleMoter(0);
  }
}
