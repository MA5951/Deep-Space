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

  /**
   * Set the intake motor to the max power
   */
  @Override
  protected void initialize() {
    rider.controlIntakeMotor(1);
  }

  
  @Override
  protected void execute() {
  }

  /**
   * If the limit switch is pressed, the isLimitSwitchAnglePressed will disable  
   */
  @Override
  protected boolean isFinished() {
    return !rider.isLimitSwitchAnglePressed();
  }

  /**
   * set the intake motor to the max power , set Timer to 0.5 and disable the the intake motor if isFinished is function is true
   */
  @Override
  protected void end() {
    rider.controlIntakeMotor(1);
    Timer.delay(0.5);
    rider.controlIntakeMotor(0);

    
  }

  /**
   * disable the intake motor if end function was interrupted
   */
  @Override
  protected void interrupted() {
    rider.controlIntakeMotor(0);
  }
}
