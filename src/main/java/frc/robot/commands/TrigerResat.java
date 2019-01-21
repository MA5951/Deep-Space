/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.subsystems.Intake;
/**
 * Add your docs here.
 */
public class TrigerResat extends Trigger {
  Intake dodoSubsystem= Intake.getInstance();
  @Override
  public boolean get() {
    return dodoSubsystem.isLImitSwhichOnUp();
  
  }
