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

  /**
   * Enable the PID controller
   */
  @Override
  protected void initialize() {
    rider.PIDRiderEnable();
  }

  
  @Override
  protected void execute() {
  }

  /**
   * Check if Rider reached the desire destination 
   */
  @Override
  protected boolean isFinished() {
    return rider.isRiderOnTarget();
  }

  /**
   * disable the PID controller if isFinished function is true
   */
  @Override
  protected void end() {
    rider.PIDDisable();
  }

  
  @Override
  protected void interrupted() {
  }
}
