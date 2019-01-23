/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

/**
 * Gets the Rider to a specific angle using PID.
 */
public class RiderPID extends Command {
  private double setPoint;
  private double tolorance;
  private Rider rider = Rider.getInstance();

  public RiderPID(double setPoint, double tolorance) {
    this.setPoint = setPoint;
    this.tolorance = tolorance;
    requires(rider);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rider.setSetPoint(setPoint);
    rider.setPIDTolerance(tolorance);
    rider.enablePID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return rider.isOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rider.disablePID();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    rider.disablePID();
  }
}
