/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * A subsystem which contains mainly 2 motors and an encoder. In the end game,
 * the robot will use these motors to climb.
 */
public class Climber extends Subsystem {

  private static Climber m_instance;

  private WPI_TalonSRX leftPoleMotor, rightPoleMotor;
  private Encoder poleEncoder;

  public static final double MAX_POLE_ENCODER_TICKS = 10000; // TODO

  private Climber() {
    leftPoleMotor = new WPI_TalonSRX(RobotMap.LEFT_CLIMB_MOTOR);
    rightPoleMotor = new WPI_TalonSRX(RobotMap.RIGHT_CLIMB_MOTOR);
    rightPoleMotor.set(ControlMode.Follower, leftPoleMotor.getDeviceID());

    poleEncoder = new Encoder(RobotMap.CLIMB_ENCODER_A_CHANNEL, RobotMap.CLIMB_ENCODER_B_CHANNEL, false,
        EncodingType.k4X);
    poleEncoder.reset();
  }

  /**
   * Singleton function which returns the instance.
   * 
   * @return The subsystem instance.
   */
  public static Climber getInstance() {
    if (m_instance == null)
      m_instance = new Climber();
    return m_instance;
  }

  /**
   * @return Has the robot has reached it's target height (and can finish
   *         climbing).
   */
  public boolean isPoleOut() {
    return poleEncoder.getRaw() >= MAX_POLE_ENCODER_TICKS;
  }

  /**
   * Has the robot use it's backwards climbing pole with maximum strength.
   */
  public void openPole() {
    leftPoleMotor.set(1);
  }

  /**
   * Stops the climbing mechanism from the back.
   */
  public void stopOpeningPole() {
    leftPoleMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
