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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Chassis.TankDrive;
import frc.robot.commands.Chassis.limitspped;

/**
 * The Chassis subsystem
 */
public class Chassis extends Subsystem {
  public double setPoint;
  private static Chassis c_Instance = new Chassis();

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;

  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private AHRS navx;

  boolean limit = false;

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
    rightFrontMotor = new CANSparkMax(RobotMap.CHASSIS_RIGHT_FRONT , MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(RobotMap.CHASSIS_RIGHT_REAR , MotorType.kBrushless);

    leftFrontMotor = new CANSparkMax(RobotMap.CHASSIS_LEFT_FRONT , MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(RobotMap.CHASSIS_LEFT_REAR , MotorType.kBrushless);
    rightFrontMotor.setSmartCurrentLimit(60);
    rightRearMotor.setSmartCurrentLimit(60);
    leftFrontMotor.setSmartCurrentLimit(60);
    leftRearMotor.setSmartCurrentLimit(60);
  
    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);

    navx.setPIDSourceType(PIDSourceType.kDisplacement);

    rightRearMotor.follow(rightFrontMotor);
    leftRearMotor.follow(leftFrontMotor);

    navxController = new PIDController(KP_NAVX, KI_NAVX, KD_NAVX, navx, rightFrontMotor);

    navxController.setAbsoluteTolerance(NAVX_TOLERANCE);
    limitspped.limit=false;

  }

 

  public void chassisSmartdashboardValue() {
    SmartDashboard.putNumber("Right Chassis Motors", rightFrontMotor.get());
    SmartDashboard.putNumber("Left Chassis Motors", leftFrontMotor.get());
    SmartDashboard.putNumber("Chassis Navx", navx.getAngle());
  }

  
  public double getAngle() {
    return navx.getAngle() % 180;
  }

  public double getAcceleration() {
    return navx.getRawAccelZ();
  }
  /**
   * Give power to the motors
   * 
   * @param speedLeft  Speed of the left side chassis
   * @param speedRight Speed of the right side chassis
   */
  public void driveWestCoast(double speedLeft, double speedRight) {
    rightFrontMotor.set(speedRight);
    leftFrontMotor.set(speedLeft);
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