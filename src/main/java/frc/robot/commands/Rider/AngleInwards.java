/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Rider;

public class AngleInwards extends Command {

  private Rider rider = Rider.getInstance();
  private double angleInward;
  //Tolorance for the encoder (need to defined agter config)
  private int toloranceMax = 360,toloranceMin =0;

  public AngleInwards(double angleInward) {

    this.angleInward = angleInward;
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
    rider.angleInward();   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (!(toloranceMin < angleInward) && !(angleInward < toloranceMax));
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
