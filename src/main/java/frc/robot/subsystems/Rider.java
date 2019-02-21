/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.rider.AngleRider;

public class Rider extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Rider r_Instance;

  private WPI_TalonSRX angleMotor;
  private WPI_VictorSPX intakeMotor;

  private Encoder encoderAngle;

  private PIDController anglePIDController;

  private DigitalInput riderLimitswitch;

  
  public static final double KP_ANGLE = 0.009;
  public static final double KI_ANGLE = 0.000002;
  public static final double KD_ANGLE = 0.0006;
  public static final double KF_ANGLE = 0.0;

  private static final double DISTANCE_PER_PULSE = 1;
  public static final double TOLERANCE = 1;

  /**
   * Initializes all Rider components
   */
  private Rider() {
    riderLimitswitch = new DigitalInput(RobotMap.RIDER_LIMITSWITCH_BALL);
    angleMotor = new WPI_TalonSRX(RobotMap.RIDER_ANGLE_MOTOR);
    intakeMotor = new WPI_VictorSPX(RobotMap.RIDER_INTAKE_MOTOR);
    angleMotor.setInverted(true);
    encoderAngle = new Encoder(RobotMap.RIDER_ENCODER_A, RobotMap.RIDER_ENCODER_B, false, EncodingType.k4X);
    encoderAngle.setDistancePerPulse(DISTANCE_PER_PULSE);
    anglePIDController = new PIDController(KP_ANGLE, KI_ANGLE, KD_ANGLE, KF_ANGLE, encoderAngle, angleMotor);
    encoderAngle.setPIDSourceType(PIDSourceType.kDisplacement);
    anglePIDController.setAbsoluteTolerance(TOLERANCE);
    anglePIDController.setOutputRange(-0.6, 0.6);
  }

  public void setF(double f) {
    // anglePIDController.setF(f);
  }

  public void riderSmartdashboardValue() {
    SmartDashboard.putNumber("Rider Intake Motor", intakeMotor.get());
    SmartDashboard.putNumber("Rider Angle Motor", angleMotor.get());
    SmartDashboard.putNumber("Rider Angle Encoder", encoderAngle.getDistance());
  }

  /**
   * Check whether the limitswitch is pressed.
   * 
   * @return Indication if the limitswitch is pressed.
   */
  public boolean isLimitswitchClosed() {
    return angleMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean isPID_Disabled() {
    return !anglePIDController.isEnabled();
  }

  /**
   * Enables the angle PID
   */
  public void enablePID(boolean enable) {
    if (enable) {
      anglePIDController.enable();
      angleMotor.overrideLimitSwitchesEnable(false);
    } else {
      anglePIDController.disable();
      angleMotor.overrideLimitSwitchesEnable(true);
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

  public boolean isEncoderInDistanceRangeRider(double maxDistance, double minDistance) {
    return encoderAngle.getDistance() <= maxDistance && encoderAngle.getDistance() >= minDistance;
  }

  /**
   * 
   * Return if the robot reached the desired destination
   * 
   * @return Indication if rider is on target
   */
  public boolean isPIDOnTarget(double ticks, double tolerance) {
    return encoderAngle.get() >= ticks - tolerance && encoderAngle.get() <= ticks + tolerance;
  }

  public boolean getBallLimitswitch() {
    return !riderLimitswitch.get();
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
    angleMotor.set(angleSpeed);
  }

  /**
   * Resets the encoder
   */
  public void resetEncoder() {
    encoderAngle.reset();
  }

  public double getEncoder() {
    return encoderAngle.getDistance();
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
    setDefaultCommand(new AngleRider());
  }
}
