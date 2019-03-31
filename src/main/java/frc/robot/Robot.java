/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rider;

public class Robot extends TimedRobot {
  public static OI m_oi;

  @Override
  public void robotInit() {
    m_oi = new OI();
    Chassis.getInstance();
    Intake.getInstance();
    Rider.getInstance();
    Elevator.getInstance();
    SmartDashboard.updateValues();
  
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
   
    
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    Rider.getInstance().setSetPoint(Rider.getInstance().getEncoder());
    Elevator.getInstance().setSetPoint(Elevator.getInstance().getElevatorEncoder());
  }
  @Override
  public void autonomousInit() {
    Scheduler.getInstance().run();

  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    Intake.getInstance().intakeSmartdashboardValue();
    Elevator.getInstance().elevatorSmartdashboardValue();
    Rider.getInstance().riderSmartdashboardValue();
    Climber.getInstance().climberSmartDashBoardValues();
    SmartDashboard.updateValues();
  }

  @Override
  public void testPeriodic() {
  }
}
