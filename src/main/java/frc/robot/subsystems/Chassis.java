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
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Chassis.TankDrive;

/**
 * The Chassis subsystem
 */
public class Chassis extends Subsystem {

  private static Chassis c_Instance;

  private WPI_TalonSRX leftFrontMotor;
  private WPI_TalonSRX leftRearMotor;

  private WPI_TalonSRX rightFrontMotor;
  private WPI_TalonSRX rightRearMotor;

  private Encoder encoderRight;
  private Encoder encoderLeft;

  private AHRS navX;

  private PIDController leftChassisEncoderPID;
  private PIDController rightChassisEncoderPID;

  private PIDController navXController;

  // TODO
  public static final double KP_ENCODER = 0.0;
  public static final double KI_ENCODER = 0.0;
  public static final double KD_ENCODER = 0.0;
  private static final double ENCODER_TOLERANCE = 0.5;

  public static final double KP_NAVX = 0.0;
  public static final double KI_NAVX = 0.0;
  public static final double KD_NAVX = 0.0;
  private static final double NAVX_TOLERANCE = 0.5;
  private static final double DISTANCE_PER_PULSE = 0.01;

  /**
   * Initializes all Chassis components
   */
  private Chassis() {
    rightFrontMotor = new WPI_TalonSRX(RobotMap.CHASSIS_RIGHT_FRONT);
    rightRearMotor = new WPI_TalonSRX(RobotMap.CHASSIS_RIGHT_REAR);

    leftFrontMotor = new WPI_TalonSRX(RobotMap.CHASSIS_LEFT_FRONT);
    leftRearMotor = new WPI_TalonSRX(RobotMap.CHASSIS_LEFT_REAR);

    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);

    navX = new AHRS(Port.kMXP);
    navX.setPIDSourceType(PIDSourceType.kDisplacement);

    encoderLeft = new Encoder(RobotMap.CHASSIS_LEFT_ENCODER_A, RobotMap.CHASSIS_LEFT_ENCODER_B, false,
        EncodingType.k4X);
    encoderRight = new Encoder(RobotMap.CHASSIS_RIGHT_ENCODER_A, RobotMap.CHASSIS_RIGHT_ENCODER_B, false,
        EncodingType.k4X);

    encoderLeft.reset();
    encoderRight.reset();

    encoderLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderRight.setDistancePerPulse(DISTANCE_PER_PULSE);

    encoderLeft.setPIDSourceType(PIDSourceType.kDisplacement);
    encoderRight.setPIDSourceType(PIDSourceType.kDisplacement);

    rightFrontMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());
    rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());

    leftFrontMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());
    leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());

    rightChassisEncoderPID = new PIDController(KP_ENCODER, KI_ENCODER, KD_ENCODER, encoderRight, rightFrontMotor);
    leftChassisEncoderPID = new PIDController(KP_ENCODER, KI_ENCODER, KD_ENCODER, encoderLeft, leftFrontMotor);

    rightChassisEncoderPID.setAbsoluteTolerance(ENCODER_TOLERANCE);
    leftChassisEncoderPID.setAbsoluteTolerance(ENCODER_TOLERANCE);

    navXController = new PIDController(KP_NAVX, KI_NAVX, KD_NAVX, navX, rightFrontMotor);

    navXController.setAbsoluteTolerance(ENCODER_TOLERANCE);

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
   * setLeftSide is for the PID, set the speed For both left and right side
   * @param speedLeft
   */
  public void setLeftSide(double speedLeft) {
    leftFrontMotor.set(speedLeft);
  }

  /**
   * Check whether right EncoderPIDController is on target
   * 
   * @return Indication if right controller is on target
   */
  public boolean isRightEncoderPIDOnTarget() {
    return rightChassisEncoderPID.onTarget();
  }

  /**
   * Check whether left EncoderPIDController is on target
   * 
   * @return Indication if left controler is on target
   */
  public boolean isLeftEncoderPIDOnTarget() {

    return leftChassisEncoderPID.onTarget();
  }

  /**
   * Check whether right NavxPIDController is on target
   * 
   * @return Indication if left controler is on target
   */
  public boolean isRightNavxPIDOnTarget() {

    return navXController.onTarget();
  }

  /**
   * Set the PIDController Encoder destination (set point)
   * 
   * @param setPoint The given destination (set point)
   */
  public void setSetPointEncoder(double setPoint) {
    rightChassisEncoderPID.setSetpoint(setPoint);
    leftChassisEncoderPID.setSetpoint(setPoint);
  }

  /**
   * Set the PIDController NAVX destination (set point)
   * 
   * @param setPoint The given destination (set point)
   */
  public void setSetPointNavx(double setPoint) {
    navXController.setSetpoint(setPoint);
  }

  /**
   * Enables the Encoder PIDController.
   * 
   * @param enable Whether to enable or disable controller
   */
  public void enableChassisEncoderPID(boolean enable) {
    if (enable) {
      rightChassisEncoderPID.enable();
      leftChassisEncoderPID.enable();
    } else {
      rightChassisEncoderPID.disable();
      leftChassisEncoderPID.disable();
    }
  }

  /**
   * Enables the NAVX PIDController.
   * 
   * @param enable Whether to enable or disable controller
   */
  public void enableChassisNavxPID(boolean enable) {
    if (enable) {
      navXController.enable();
    } else {
      navXController.disable();
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