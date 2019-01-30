/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

// TODO Redesign rider commands after redesigning subsystem. 
@Deprecated
public class Rider extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Rider r_Instance;

  private WPI_TalonSRX angleMotor;
  private WPI_TalonSRX intakeMotor;

  private Encoder encoderAngle;

  private DigitalInput limitSwitcAngleUp;
  private DigitalInput limitSwitcAngleDown;
  private DigitalInput ir;

  private PIDController anglePIDController;

  public static final double KP_ANGLE = 0;
  public static final double KI_ANGLE = 0;
  public static final double KD_ANGLE = 0;

  private static final double DISTANCE_PER_PULSE = 0;
  private static final double TOLERANCE = 0;

  /**
   * Initializes all Rider components
   */
  private Rider() {

    angleMotor = new WPI_TalonSRX(RobotMap.RIDER_ANGLE_MOTOR);
    intakeMotor = new WPI_TalonSRX(RobotMap.RIDER_INTAKE_MOTOR);

    encoderAngle = new Encoder(RobotMap.RIDER_ENCODER_A, RobotMap.RIDER_ENCODER_B, false, EncodingType.k4X);
    encoderAngle.setDistancePerPulse(DISTANCE_PER_PULSE);

    limitSwitcAngleUp = new DigitalInput(RobotMap.RIDER_ANGLE_LIMIT_SWITCH);
    limitSwitcAngleDown = new DigitalInput(RobotMap.RIDER_INTAKE_LIMIT_SWITCH);
    ir = new DigitalInput(RobotMap.RIDER_PROXIMITY_SENSOR);

    anglePIDController = new PIDController(KP_ANGLE, KI_ANGLE, KD_ANGLE, encoderAngle, angleMotor);
    encoderAngle.setPIDSourceType(PIDSourceType.kDisplacement);
    anglePIDController.setAbsoluteTolerance(TOLERANCE);
  }

  public void RiderSmartdashboardValue() {
    SmartDashboard.putNumber("Rider Intake Motor", intakeMotor.get());
    SmartDashboard.putNumber("Rider Angle Motor", angleMotor.get());
    SmartDashboard.putNumber("Rider Angle Encoder", encoderAngle.get());
    SmartDashboard.putBoolean("Rider Limit Switch Up", limitSwitcAngleUp.get());
    SmartDashboard.putBoolean("Rider Limit Switch Down", limitSwitcAngleDown.get());
    SmartDashboard.putBoolean("Rider Proximity Sensor", ir.get());

  }

  /**
   * Enables the angle PID
   */
  public void enablePID(boolean enable) {
    if (enable) {
      anglePIDController.enable();
    } else {
      anglePIDController.disable();
    }
  }

  /**
   * Set the desired angle for the Rider.
   * 
   * @param setPoint the destination.
   */
  public void setSetPoint(double setPoint) {
    anglePIDController.setSetpoint(setPoint);
  }

  /**
   * 
   * Return if the robot reached the desired destination
   * 
   * @return Indication if rider is on target
   */
  public boolean isOnTarget() {
    return anglePIDController.onTarget();
  }

  /**
   * Control the intake motor power
   * 
   * @param speed The given power
   */
  public void controlIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Control the angle motor power
   * 
   * @param angleSpeed The given power
   */
  public void controlAngleMotor(double angleSpeed) {
    angleMotor.set(ControlMode.PercentOutput, angleSpeed);
  }

  /**
   * Check whether {limitSwitcAngleUp} is pressed.
   * 
   * @return Indication if {limitSwitcAngleUp} is pressed.
   */
  public boolean isLimitSwitchAngleUpPressed() {
    return limitSwitcAngleUp.get();
  }

  /**
   * Check whether {limitSwitcAngleDown} is pressed.
   * 
   * @return Indication if {limitSwitcAngleDown} is pressed.
   */
  public boolean isLimitSwitchAngleDownPressed() {
    return limitSwitcAngleDown.get();
  }

  /**
   * Check whether proximity sensor is on range.
   * 
   * @return Indication if proximity sensor is on range.
   */
  public boolean getProximitySensorInRange() {
    return ir.get();
  }

  /**
   * Get the current angle.
   * 
   * @return Indication of the current angle.
   */
  public int getCurrentAngle() {
    return encoderAngle.get();
  }

  /**
   * Resets the encoder
   */
  public void resetEncoder() {
    encoderAngle.reset();
  }

  /**
   * Makes sure that only one intance runs at a time
   * 
   * @return The instance
   */
  public static Rider getInstance() {
    if (r_Instance == null)
      r_Instance = new Rider();
    return r_Instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here
  }
}
