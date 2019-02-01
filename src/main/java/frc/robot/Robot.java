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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rider;

public class Robot extends TimedRobot { // TODO do not use encoder.get(), use encoder.getDistance() on ALL subsystems
  public static OI m_oi;

  @Override
  public void robotInit() {
    m_oi = new OI();
    Chassis.getInstance();
    
    
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
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // TODO All names should start with lowercase
    // TODO Add javadoc
    Chassis.getInstance().ChassisSmartdashboardValue();
    Intake.getInstance().IntakeSmartdashboardValue();
    Elevator.getInstance().ElevatorSmartdashboardValue(); // TODO Getting instance with no substantial subsystem will throw error. 
    Rider.getInstance().RiderSmartdashboardValue();
    SmartDashboard.updateValues();
  }


  @Override
  public void testPeriodic() {
  }
}
