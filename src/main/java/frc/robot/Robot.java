/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Chassis;
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

    //CameraServer.getInstance().startAutomaticCapture();
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
    Rider.getInstance().setSetPoint(Rider.getInstance().getEncoder());
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
    Chassis.getInstance().chassisSmartdashboardValue();
    Intake.getInstance().intakeSmartdashboardValue();
    Elevator.getInstance().elevatorSmartdashboardValue();
    Rider.getInstance().riderSmartdashboardValue();
    SmartDashboard.updateValues();
    if (OI.LEFT_DRIVER_STICK.getRawAxis(3) > 0.5) {
      SmartDashboard.putBoolean("forwardBack", true);
}else if(OI.LEFT_DRIVER_STICK.getRawAxis(3) < -0.5){
  SmartDashboard.putBoolean("forwardBack", false);
    } 

   
  }

  @Override
  public void testPeriodic() {
  }
}
