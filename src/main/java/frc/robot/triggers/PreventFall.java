/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.subsystems.Chassis;

/**
 * Add your docs here.
 */
public class PreventFall extends Trigger {
  @Override
  public boolean get() {
    return Chassis.getInstance().isRobotFall(0, 0); //TODO
  }
}
