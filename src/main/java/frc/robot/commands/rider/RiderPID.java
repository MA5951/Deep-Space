/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

public class RiderPID extends Command {
  double setPoint;
  double TOLORANCE;
  private Rider rider = Rider.getInstance();
  public RiderPID(double setPoint,double TOLORANCE) {
    this.setPoint=setPoint;
    this.TOLORANCE=TOLORANCE;    
 requires(rider);
 rider.setPointRider(setPoint);
 rider.RiderPIDTolerance(TOLORANCE);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rider.PIDRiderEnable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return rider.isRiderOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rider.PIDDisable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
