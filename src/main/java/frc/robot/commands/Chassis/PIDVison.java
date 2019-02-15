/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class PIDVison extends Command {
  private double speed;
  Chassis chassis = Chassis.getInstance();
  public PIDVison() {
    requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    speed = chassis.PIDVison(0); //TODO
    chassis.driveWestCoast(speed*0.5, speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return chassis.isOnTargetPIDVison();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    chassis.driveWestCoast(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    chassis.driveWestCoast(0, 0);
  }
}
