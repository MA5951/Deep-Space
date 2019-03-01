/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Rider;

public class TeleopRiderIntakeControl extends Command {
  Rider rider = Rider.getInstance();
  private double speed;
  public TeleopRiderIntakeControl(double speed) {
    this.speed=speed;
    requires(rider);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    rider.controlIntakeMotor(speed);
    if(!rider.getBallLimitswitch()&&speed>0||rider.getBallLimitswitch()&&speed<0){
      OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 1);
      OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 1);
}else{
  OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
  OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
}
}


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rider.controlIntakeMotor(0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    rider.controlIntakeMotor(0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kLeftRumble, 0);
    OI.OPERATOR_STICK.setRumble(RumbleType.kRightRumble, 0);
  }
}
