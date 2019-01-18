/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
/**
 * Add your docs here.
 */

public class Rider extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static Rider instance;

  private WPI_TalonSRX angleMotor;
  private WPI_TalonSRX intakeMotor;
  private Encoder encoder;


  public Rider(){
   angleMotor = new WPI_TalonSRX(RobotMap.ANGLE_MOTOR);
   intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR);

   encoder = new Encoder(RobotMap.ENCODER_PORT_ONE, RobotMap.ENCODER_PORT_TWO);



  }


    public void intakeButtonIn()  {
      intakeMotor.set(-0.5);
    }

    public void intakeButtonOut() {
      intakeMotor.set(0.5);
    }


    public void angleButtonInward()  {
      angleMotor.set(-0.5);
    }

    public void angleButtonOutward()  {
      angleMotor.set(0.5);
    }


    public static Rider getInstance()  {
      if(instance == null){
          instance = new Rider();
      }

      return instance;
    }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
