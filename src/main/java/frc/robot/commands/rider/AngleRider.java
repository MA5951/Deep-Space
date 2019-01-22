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

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    rider.controlAngleMoter(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return rider. ifInTheCurrentAngle(angle);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rider.controlAngleMoter(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    rider.controlAngleMoter(0);
  }
}
