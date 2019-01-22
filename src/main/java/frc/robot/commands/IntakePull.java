/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePull extends Command {

  //using rider subsystem
  //TODO check if good idea.
   private Rider  rider  = Rider.getInstance();

  private Intake intake= Intake.getInstance();
  
  private double speed;
  public IntakePull (double speed) {
    this.speed=speed;
   requires(intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intake.intakeControl(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return rider.isLimitSwitchAnglePressed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.intakeControl(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    intake.intakeControl(0);
  }
}
