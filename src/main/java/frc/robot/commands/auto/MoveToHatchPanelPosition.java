/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.commands.intake.IntakePID;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rider;

public class MoveToHatchPanelPosition extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MoveToHatchPanelPosition() {
    addSequential(
        // TODO Check elevator encoder value
        new ConditionalCommand(new ElevatorPID(100, 0.1), null) {
          @Override
          protected boolean condition() {
            return Rider.getInstance().getEncoder() < 4944 && Elevator.getInstance().getElevatorEncoder() > 847;
          }
        });
    // addSequential(new ElevatorPID(0, 0.2));
    // addSequential(new RiderPID(0, 0.1, 15);
    addSequential(new IntakePID(-520, 0.1));
    addSequential(new RumbleJoystick(500));
    // addParallel(new WaitCommand(10)); //chack if work
  }
}
