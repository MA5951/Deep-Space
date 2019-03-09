/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

  private DigitalInput elevatorLimitswitchDown;
  private DigitalInput elevatorLimitswitchUp;

  public static final double KP_ENCODER = 0.0028;
  public static final double KI_ENCODER = 0.0009;
  public static final double KD_ENCODER = 0.0035;
  public static final double TOLERANCE = 300;
  private static final double DISTANCE_PER_PULSE = 1;

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

    elevatorLimitswitchUp = new DigitalInput(RobotMap.ELEVATOR_LIMITSWITCH_UP);
    elevatorLimitswitchDown = new DigitalInput(RobotMap.ELEVATOR_LIMITSWITCH_DOWN);

    elevatorEncoderPID = new PIDController(KP_ENCODER, KI_ENCODER, KD_ENCODER, encoderElevator, elevatorMotor);
    elevatorEncoderPID.setAbsoluteTolerance(TOLERANCE);
    elevatorEncoderPID.setOutputRange(-0.85, 0.85);
  }

  public void elevatorSmartdashboardValue() {
    if (getElevatorEncoder() <= 0) {
      SmartDashboard.putBoolean("Elevator", true);
    } else {
      SmartDashboard.putBoolean("Elevator", false);
    }
    SmartDashboard.putNumber("Elevator Encoder", encoderElevator.get());
  }
  public double getElevatorEncoder (){
    return encoderElevator.getDistance();
  }

  
  /**
   * Check whether limitswitch is pressed.
   * 
   * @return Indication if limitswitch is pressed.
   */
  public boolean isElevatorLimitswitchUpPressed() {
    return !elevatorLimitswitchUp.get();
  }

  public boolean isPID_Disabled() {
    return !elevatorEncoderPID.isEnabled();
  }
  /**
   * Check whether limitswitch is pressed.
   * 
   * @return Indication if limitswitch is pressed.
   */
  public boolean isElevatorLimitswitchDownPressed() {
    return elevatorLimitswitchDown.get();
  }

  /**
   * Enables or disables the PIDController.
   * 
   * @param enable Is the PID enabled.
   */
  public void enablePID(boolean enable) {
    if (enable) {
      elevatorEncoderPID.enable();
      elevatorMotor.overrideLimitSwitchesEnable(false);
    } else {
      elevatorEncoderPID.disable();
      elevatorMotor.overrideLimitSwitchesEnable(true);
    }
  }

  /**
   * Set the desired PID destination (setpoint).
   * 
   * @param setSetpoint The given destination (setpoint).
   */
  public void setSetPoint(double setSetpoint) {
    elevatorEncoderPID.setSetpoint(setSetpoint);
  }

  /**
   * Check whether the robot reached the desire destination.
   * 
   * @return Indication if the robot is on the PID target.
   */
  public boolean isPIDOnTarget() {
    return elevatorEncoderPID.onTarget();
  }

  /**
   * Controls the speed of the elevator motor.
   */
  public void controlSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  /**
   * Specifies if the the elevator encoder passed a distance within a certain range.
   * 
   * @param maxDistance The maximum distance the encoder could passed in the range
   * @param minDistance The minimum distance the encoder could pass in the range
   * @return Indication if the elevator encoder in the correct range in terms of the distance.
   */
  public boolean isEncoderInDistanceRangeElevator(double maxDistance, double minDistance) {
    return encoderElevator.getDistance() <= maxDistance && encoderElevator.getDistance() >= minDistance;
  }

  /**
   * Reset the encoder
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
