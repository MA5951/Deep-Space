/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  private Solenoid climbeSolenoid;
  private static Climber Climber;

  private Climber() {
    climbeSolenoid = new Solenoid(RobotMap.CLIMBIR_PISTON_FORWARD);
  }

  public void kforwardeClamber() {
    climbeSolenoid.set(true);
  }

  public void kReverseClimber() {
    climbeSolenoid.set(false);

  }

  public void climberSmartDashBoardValues() {
    SmartDashboard.putBoolean("ClimberPistons", climbeSolenoid.get());

  }

  public static Climber getInstance() {
    if (Climber == null) {
      Climber = new Climber();
    }
    return Climber;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
