/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

public class MoveAngle extends Command {
  private Rider rider = Rider.getInstance();
  private double speed;
  private double minDistance;
  private double maxDistance;

  public MoveAngle(double maxDistance,double minDistance, double speed) {
    this.maxDistance = maxDistance;
    this.minDistance = minDistance;
    this.speed = Math.abs(speed);
     requires(rider);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (rider.getEncoder() > ((maxDistance + minDistance) / 2)) {
      rider.controlAngleMotor(speed);
    } else {
      rider.controlAngleMotor(-speed);
    }
    if (!rider.getBallLimitswitch()) {

      rider.controlIntakeMotor(0.1);
    } else {
      rider.controlIntakeMotor(0);
    }
  }



  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return rider.isEncoderInDistanceRangeRider(maxDistance, minDistance);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rider.controlAngleMotor(0);
    rider.controlIntakeMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    rider.controlAngleMotor(0);
    rider.controlIntakeMotor(0);
  }
}
