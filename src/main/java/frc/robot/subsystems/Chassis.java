/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Chassis.TankDrive;

/**
 * The Chassis subsystem
 */
public class Chassis extends Subsystem {
  public double setPoint;
  private final double TOLERANCEPIDVISON = 0;
  private final double KPVISON = 0;
  private static Chassis c_Instance = new Chassis();

  private WPI_TalonSRX leftFrontMotor;
  private WPI_TalonSRX leftRearMotor;

  private WPI_TalonSRX rightFrontMotor;
  private WPI_TalonSRX rightRearMotor;

  private AHRS navx;

  private PIDController navxController;
  public static final double KP_NAVX = 0.0;
  public static final double KI_NAVX = 0.0;
  public static final double KD_NAVX = 0.0;
  private static final double NAVX_TOLERANCE = 0.5;

  /**
   * Initializes all Chassis components
   */
  private Chassis() {

    navx = new AHRS(Port.kMXP);
    rightFrontMotor = new WPI_TalonSRX(RobotMap.CHASSIS_RIGHT_FRONT);
    rightRearMotor = new WPI_TalonSRX(RobotMap.CHASSIS_RIGHT_REAR);

    leftFrontMotor = new WPI_TalonSRX(RobotMap.CHASSIS_LEFT_FRONT);
    leftRearMotor = new WPI_TalonSRX(RobotMap.CHASSIS_LEFT_REAR);

    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);

    navx.setPIDSourceType(PIDSourceType.kDisplacement);

    rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());
    leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());

    navxController = new PIDController(KP_NAVX, KI_NAVX, KD_NAVX, navx, rightFrontMotor);

    navxController.setAbsoluteTolerance(NAVX_TOLERANCE);

  }

  public double PIDVison(double setPoint) {
    this.setPoint = setPoint;
    return (setPoint - Robot.xEntry.getDouble(Robot.x)) * KPVISON;
  }

  public boolean isOnTargetPIDVison() {
    return setPoint - Robot.xEntry.getDouble(Robot.x) == setPoint - TOLERANCEPIDVISON
        || setPoint - Robot.xEntry.getDouble(Robot.x) == setPoint + TOLERANCEPIDVISON;
  }

  public void chassisSmartdashboardValue() {
    SmartDashboard.putNumber("Right Chassis Motors", rightFrontMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Chassis Motors", leftFrontMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Chassis Navx", navx.getAngle());
  }

  /**
   * Give power to the motors
   * 
   * @param speedLeft  Speed of the left side chassis
   * @param speedRight Speed of the right side chassis
   */
  public void driveWestCoast(double speedLeft, double speedRight) {
    rightFrontMotor.set(ControlMode.PercentOutput, speedRight);
    leftFrontMotor.set(ControlMode.PercentOutput, speedLeft);
  }

  /**
   * setLeftSide is for the navx PID, set the speed For both left and right side
   * 
   * @param speedLeft
   */
  public void setLeftSide(double speedLeft) {
    leftFrontMotor.set(speedLeft);
  }

  /**
   * Check whether right NavxPIDController is on target
   * 
   * @return Indication if left controler is on target
   */
  public boolean isNavxPIDOnTarget() {
    return navxController.onTarget();
  }

  /**
   * Set the PIDController NAVX destination (set point)
   * 
   * @param setPoint The given destination (set point)
   */
  public void setSetPointNavx(double setPoint) {
    navxController.setSetpoint(setPoint);
  }

  /**
   * Enables the NAVX PIDController. Doesn't allow navx and encoder pid to be
   * enabled simultaneously.
   * 
   * @param enable Whether to enable or disable controller
   */
  public void enableChassisNavxPID(boolean enable) {

    if (enable) {
      navxController.enable();
    } else {
      navxController.disable();
    }
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDrive());
  }

  /**
   * Makes sure that only one instance runs at a time
   * 
   * @return The instance
   */
  public static Chassis getInstance() {
    if (c_Instance == null)
      c_Instance = new Chassis();
    return c_Instance;
  }
}