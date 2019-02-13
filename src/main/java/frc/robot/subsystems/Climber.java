/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * A subsystem which contains mainly 2 motors and an encoder. In the end game,
 * the robot will use these motors to climb.
 */
public class Climber extends Subsystem {

  private static Climber m_instance;
DoubleSolenoid climberSolenoid;


  private Climber() {
    climberSolenoid = new DoubleSolenoid(RobotMap.CLIMBIR_PISTON_FORWARD,RobotMap.CLIMBIR_PISTON_REVERS);
  }

  public void climberSolenoidForward(){
    climberSolenoid.set(Value.kForward);
  }
  public void climberSolenoidReverse(){
    climberSolenoid.set(Value.kReverse);
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


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
