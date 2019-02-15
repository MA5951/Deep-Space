/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  public static NetworkTableEntry xEntry;
  public static NetworkTableEntry yEntry; 
  public static double x;
  public static double y;
  @Override
  public void robotInit() {
   NetworkTableInstance inst = NetworkTableInstance.getDefault();
   NetworkTable  table =inst.getTable("datatable");
   xEntry = table.getEntry("X");  
   yEntry = table.getEntry("Y");

    m_oi = new OI();
    Chassis.getInstance();
    Intake.getInstance();
    Rider.getInstance();
    Elevator.getInstance();
    Climber.getInstance();

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();
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
    xEntry.setDouble(x);
    yEntry.setDouble(y);
    x+=0.05;
    y+=1;
    Scheduler.getInstance().run();
    Chassis.getInstance().chassisSmartdashboardValue();
    Intake.getInstance().intakeSmartdashboardValue();
    Elevator.getInstance().elevatorSmartdashboardValue();
    Rider.getInstance().riderSmartdashboardValue();
    SmartDashboard.updateValues();
  }

  @Override
  public void testPeriodic() {
  }
}
