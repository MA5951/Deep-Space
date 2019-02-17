/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorPID;

import frc.robot.commands.rider.RiderPID;

public class Rocket1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Rocket1() {
 

     addSequential(new RiderPID(0, 1, 20));
     addSequential(new AutomaticIntake(-150,-200,1));
     addSequential(new ElevatorPID(-3435, 0.2));
     addSequential(new RumbleJoystick(500));

  }
}
