/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Chassis;

public class TankDrive extends Command {

  private Chassis chassis;
private boolean limit;
  /**
   * Create an object of chassis (create a new instance of chassis)
   */
  public TankDrive() {
    chassis = Chassis.getInstance();
    requires(chassis);

  }

  @Override
  protected void initialize() {

  }

  /**
   * Control the chassis with joysticks
   */
  @Override
  protected void execute() {
    if (!limitspped.limit) {
      chassis.driveWestCoast(OI.LEFT_DRIVER_STICK.getY()*0.9, OI.RIGHT_DRIVER_STICK.getY()*0.9);
    } else {
      chassis.driveWestCoast(OI.LEFT_DRIVER_STICK.getY()*0.65, OI.RIGHT_DRIVER_STICK.getY()*0.65);
    }
  }

  /**
   * Makes the execute function run forever
   */
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }

}
