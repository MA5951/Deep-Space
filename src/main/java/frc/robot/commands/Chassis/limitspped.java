/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.camera;

/**
 * Add your docs here.
 */
public class limitspped extends InstantCommand {
  public static boolean limit = false;
  /**
   * Add your docs here.
   */
  public limitspped() {
    super();
     requires(camera.getInstance());
     limit= false;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    limit= !limit;
  }

}
