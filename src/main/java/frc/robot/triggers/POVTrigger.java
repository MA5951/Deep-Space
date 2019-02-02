/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * A class for making a trigger out of a POV. When making an object, simply pass
 * the POV angle, the controller and if needed the POV number.
 */
public class POVTrigger extends Trigger {
  private int angle;
  private GenericHID controller;
  private int POVNum;

  public POVTrigger(int angle, GenericHID controller) {
    this(angle, controller, 0);
  }

  public POVTrigger(int angle, GenericHID controller, int POVNum) {
    this.angle = angle;
    this.controller = controller;
    this.POVNum = POVNum;
  }

  @Override
  public boolean get() {
    return angle == controller.getPOV(POVNum);
  }
}
