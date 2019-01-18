/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

public class AngleOutward extends Command {
  
  private Rider rider = Rider.getInstance();
  private double angleOutward;

  public AngleOutward(double angleOutward) {
    this.angleOutward = angleOutward;
    // Use requires() here to declare subsystem dependencies
    requires(rider);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(angleOutward != rider.getCurrentAngle()){
      rider.angleOutward();
      } 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return angleOutward == rider.getCurrentAngle();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    rider.stopAngleMotor();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
