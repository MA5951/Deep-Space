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

public class IntakeRiderPush extends Command {
  private Rider rider = Rider.getInstance();
  
  public IntakeRiderPush() {
    requires(rider);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    rider.controlIntakeMoter(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !rider.isLimitSwitchAnglePressed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rider.controlIntakeMoter(1);
    Timer.delay(0.5);
    rider.controlIntakeMoter(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    rider.controlIntakeMoter(0);
  }
}
