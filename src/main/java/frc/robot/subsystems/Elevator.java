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
import frc.robot.commands.elevator.ElevatorJoystickControl;

public class Elevator extends Subsystem {
  private PIDController elevatorEncoderPID;
  private WPI_TalonSRX elevatorMotor;

  private Encoder encoderElevator;

  public static final double KP_ENCODER = 0;
  public static final double KI_ENCODER = 0;
  public static final double KD_ENCODER = 0;
  private static final double TOLERANCE = 0;
  private static final double DISTANCE_PER_PULSE = 0;

  private static Elevator e_Instance;

  /**
   * Initializes all the elevator components.
   */
  private Elevator() {

    encoderElevator = new Encoder(RobotMap.ELEVATOR_ENCODER_A, RobotMap.ELEVATOR_ENCODER_B, false, EncodingType.k4X);
    encoderElevator.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderElevator.setPIDSourceType(PIDSourceType.kDisplacement);

    elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_TALON);
    elevatorMotor.setInverted(true);

    elevatorEncoderPID = new PIDController(KP_ENCODER, KI_ENCODER, KD_ENCODER, encoderElevator, elevatorMotor);
    elevatorEncoderPID.setAbsoluteTolerance(TOLERANCE);
  }

  public void elevatorSmartdashboardValue() {
    SmartDashboard.putNumber("Elevator Motor", elevatorMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Elevator Encoder", encoderElevator.getDistance());
  }

  public void enablePID(boolean enable) {
    if (enable) {
      elevatorEncoderPID.enable();
    } else {
      elevatorEncoderPID.disable();
    }
  }

  public void setSetPoint(double setSetpoint) {
    elevatorEncoderPID.setSetpoint(setSetpoint);
  }

  public boolean isPIDOnTarget() {
    return elevatorEncoderPID.onTarget();
  }

  /**
   * Controls the speed of the talon
   */
  public void controlSpeed(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Specifies if the elevator encoder passed a distance within a certain range.
   * 
   * @param maxDistance The maximum distance the encoder could passed in the range
   * @param minDistance The minimum distance the encoder could pass in the range
   * @return Is the elevator encoder in the correct range in terms of distance
   */
  public boolean isEncoderInDistanceRange(double maxDistance, double minDistance) {
    return encoderElevator.getDistance() < maxDistance && encoderElevator.getDistance() > minDistance;
  }

  /**
   * Reset's the encoder
   */
  public void resetEncoder() {
    encoderElevator.reset();
  }

  /**
   * Singleton
   */
  public static Elevator getInstance() { 
    if (e_Instance == null)
      e_Instance = new Elevator();
    return e_Instance;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorJoystickControl());
  }
}
