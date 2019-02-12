/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Rider;

public class AngleRider extends Command {
  double speed;

  private Rider rider = Rider.getInstance();

  private boolean firstPID_Run = true;

  public AngleRider() {

    requires(rider);
  }

  @Override
  protected void initialize() {
    
  }

  

  @Override
  protected void execute() {
    if(OI.OPERATOR_STICK.getRawAxis(5)<0.1|| OI.OPERATOR_STICK.getRawAxis(5)>-0.1){
      if (firstPID_Run) {
        rider.setSetPoint(rider.getEncoder());
        rider.enablePID(true);
        firstPID_Run = false;
      } 
     
    }
    if(OI.OPERATOR_STICK.getRawAxis(5)>0.1||OI.OPERATOR_STICK.getRawAxis(5)<-0.1){
      rider.enablePID(false);
      firstPID_Run = true;
      speed = OI.OPERATOR_STICK.getRawAxis(5);
      rider.controlAngleMotor(speed *-0.8);
      if (rider.getBallLimitswitch()) {
        rider.controlIntakeMotor(0.35);
    } else {
      rider.controlIntakeMotor(0);
    }
    }
 
}

  @Override
  protected boolean isFinished() {
    return false;
  }

  /**
   * Reset the encoder and disable the angle motor if isFinished function is true
   */
  @Override
  protected void end() {
    rider.controlAngleMotor(0);
  }

  /**
   * Disable the angle motor if end function was interrupted
   */
  @Override
  protected void interrupted() {
    rider.controlAngleMotor(0);
  }
}
