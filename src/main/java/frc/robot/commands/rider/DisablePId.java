/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.rider;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Rider;

/**
 * Add your docs here.
 */
public class DisablePID extends InstantCommand {
  Rider rider = Rider.getInstance();
  /**
   * Add your docs here.
   */
  public DisablePID() {

    super();
    requires(rider) ;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    rider.enablePID(false);
  }


}
