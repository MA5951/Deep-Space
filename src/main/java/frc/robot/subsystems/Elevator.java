/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.ElevatorJoystickControl;

public class Elevator extends Subsystem {
  private PIDController elevatorEncoderPID;
  private Spark elevatorMotor;

  private Encoder encoderElevator;

  private DigitalInput elevatorLimitswitchDown;
  private DigitalInput elevatorLimitswitch;

  // TODO
  public static final double KP_ENCODER = 0;
  public static final double KI_ENCODER = 0;
  public static final double KD_ENCODER = 0;
  private static final double TOLERANCE = 0;
  private static final double DISTANCE_PER_PULSE = 1;

  private static Elevator e_Instance;

  /**
   * Initializes all the elevator components.
   */
  private Elevator() {

    encoderElevator = new Encoder(RobotMap.ELEVATOR_ENCODER_A, RobotMap.ELEVATOR_ENCODER_B, false, EncodingType.k4X);
    encoderElevator.setDistancePerPulse(DISTANCE_PER_PULSE);
    encoderElevator.setPIDSourceType(PIDSourceType.kDisplacement);

    elevatorMotor = new Spark(RobotMap.ELEVATOR_SPARK);
    elevatorMotor.setInverted(true);

    elevatorLimitswitchDown = new DigitalInput(RobotMap.ELEVATOR_LIMITSWITCH);
    

    elevatorEncoderPID = new PIDController(KP_ENCODER, KI_ENCODER, KD_ENCODER, encoderElevator, elevatorMotor);
    elevatorEncoderPID.setAbsoluteTolerance(TOLERANCE);
  }

  public void elevatorSmartdashboardValue() {
    SmartDashboard.putNumber("Elevator Motor", elevatorMotor.get());
    SmartDashboard.putNumber("Elevator Encoder", encoderElevator.get());
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
    } else {
      elevatorEncoderPID.disable();
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
  public boolean isEncoderInDistanceRange(double maxDistance, double minDistance) {
    return encoderElevator.getDistance() < maxDistance && encoderElevator.getDistance() > minDistance;
  }

  /**
   * Reset the encoder
   */
  public void resetEncoder() {
    encoderElevator.reset();
    System.out.println("RESETTED");
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
