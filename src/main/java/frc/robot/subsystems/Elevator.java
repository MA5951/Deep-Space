/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.ElevatorJoystickControl;

@Deprecated
public class Elevator extends Subsystem {
  private PIDController pidControllerElevator;
  private WPI_TalonSRX elevatorTalon;

  private DigitalInput limitSwitchUpLeft;
  private DigitalInput limitSwitchDownRight;
  private DigitalInput limitSwitchDownLeft;
  private DigitalInput limitSwitchUpRight;

  private Encoder elevatorEncoder;

  public static final double KP_ELEVATOR = 1;
  public static final double KI_ELEVATOR = 1;
  public static final double KD_ELEVATOR = 1;

  private static Elevator e_Instance;

  private Elevator() {
    limitSwitchUpLeft = new DigitalInput(RobotMap.ELEVATOR_SWITCH_UP_LEFT);
    limitSwitchUpRight = new DigitalInput(RobotMap.ELEVATOR_SWITCH_UP_RIGHT);
    limitSwitchDownLeft = new DigitalInput(RobotMap.ELEVATOR_SWITCH_DOWN_LEFT);
    limitSwitchDownRight = new DigitalInput(RobotMap.ELEVATOR_SWITCH_DOWN_RIGHT);

    elevatorEncoder = new Encoder(RobotMap.ELEVATOR_ENCODER_B, RobotMap.ELEVATOR_ENCODER_A, false, EncodingType.k4X);
    elevatorEncoder.setDistancePerPulse(1);
    elevatorEncoder.setPIDSourceType(PIDSourceType.kDisplacement);

    elevatorTalon = new WPI_TalonSRX(RobotMap.ELEVATOR_TALON);
    pidControllerElevator = new PIDController(KP_ELEVATOR, KI_ELEVATOR, KD_ELEVATOR, elevatorEncoder, elevatorTalon);
  }

  public void enablePID() {
    pidControllerElevator.enable();
  }

  public void disablePID() {
    pidControllerElevator.disable();
  }

  public void setSetpoint(double setSetpoint) {
    pidControllerElevator.setSetpoint(setSetpoint);
  }

  public void setAbsoluteTolerance(double setAbsoluteTolerance) {
    pidControllerElevator.setAbsoluteTolerance(setAbsoluteTolerance);
  }

  public boolean isOnTarget() {
    return pidControllerElevator.onTarget();
  }

  /**
   * Controls the speed of the talon
   */
  public void controlSpeed(double speed) {
    elevatorTalon.set(ControlMode.PercentOutput, speed);
  }

  /**
   * @return Checks if at least one limit switch is pressed
   */
  public boolean isLimitSwitchDown() {
    return limitSwitchDownLeft.get() || limitSwitchDownRight.get();
  }

  /**
   * @return Checks if at least one limit switch is pressed
   */
  public boolean isLimitSwitchUp() {
    return limitSwitchUpLeft.get() || limitSwitchUpRight.get();
  }

  /**
   * Specifies if the elevator encoder passed a distance within a certain range.
   * 
   * @param maxDistance The maximum distance the encoder could passed in the range
   * @param minDistance The minimum distance the encoder could pass in the range
   * @return Is the elevator encoder in the correct range in terms of distance
   */
  public boolean isEncoderInDistanceRange(double maxDistance, double minDistance) {
    return elevatorEncoder.getDistance() < maxDistance && elevatorEncoder.getDistance() > minDistance;
  }

  /**
   * Reset's the encoder
   */
  public void resetEncoder() {
    elevatorEncoder.reset();
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
