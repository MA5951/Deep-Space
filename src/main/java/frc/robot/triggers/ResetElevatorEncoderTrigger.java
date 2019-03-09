/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.subsystems.Elevator;

/**
 * Reset the encoder if limit switch is pressed.
 */
public class ResetElevatorEncoderTrigger extends Trigger {
  @Override
  public boolean get() {
    return Elevator.getInstance().isElevatorLimitswitchUpPressed() && Elevator.getInstance().isPID_Disabled();
  }
}
