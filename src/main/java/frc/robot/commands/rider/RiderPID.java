/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

/**
 * Gets the Rider to a specific angle using PID.
 */
public class RiderPID extends Command {
  private double setPoint;
  private double lastTimeOnTarget;
  private double tolerance;
  private double waitTime;

  private Rider rider = Rider.getInstance();

  public RiderPID(double setPoint, double waitTime, double tolerance) {
    this.setPoint = setPoint;
    this.waitTime = waitTime;
    this.tolerance = tolerance;
    requires(rider);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rider.enablePID(true);
    rider.setSetPoint(setPoint);
    //setTimeout(10); //
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(rider.getBallLimitswitch()){
      rider.controlIntakeMotor(-0.35);
    }else{
      rider.controlIntakeMotor(0);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (!rider.isPIDOnTarget(setPoint, tolerance)) {
      lastTimeOnTarget = Timer.getFPGATimestamp();
    }
    return rider.isPIDOnTarget(setPoint, tolerance) && Timer.getFPGATimestamp() - lastTimeOnTarget > waitTime; // || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rider.controlIntakeMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    rider.controlIntakeMotor(0);
  }
}