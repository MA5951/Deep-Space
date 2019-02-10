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
 * Outtakes from the Rider.
 */
public class RiderOuttake extends Command {
  private Rider rider = Rider.getInstance();

  public RiderOuttake() {
    requires(rider);
  }

  /**
   * Set the intake motor to the max power
   */
  @Override
  protected void initialize() {

  }

  @Override
  protected void execute() {
    rider.controlIntakeMotor(-1);
  }

  /**
   * If the limit switch is pressed, the isLimitSwitchAnglePressed will disable
   */
  @Override
  protected boolean isFinished() {
    
    return rider.getBallLimitswitch();
  }

  /**
   * set the intake motor to the max power , set Timer to 0.5 and disable the the
   * intake motor if isFinished is function is true
   */
  @Override
  protected void end() {
    
    rider.controlIntakeMotor(-1);
    Timer.delay(0.5); // TODO : check if this works. If not, use a wait command and a command group.
    rider.controlIntakeMotor(0);
    rider.controlAngleMotor(0);
  }

  /**
   * disable the intake motor if a function was interrupted
   */
  @Override
  protected void interrupted() {
    rider.controlIntakeMotor(0);
  }
}
