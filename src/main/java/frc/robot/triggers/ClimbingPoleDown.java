/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * A trigger that start's when the intake is in the correct height. The trigger
 * will pull the back pole out.
 */
public class ClimbingPoleDown extends Trigger {

  @Override
  public boolean get() {
    return false;
    // return Intake.getInstance().getHeight() >= Climber.START_CLIMB_HEIGHT;
    // TODO
  }
}
