// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX armMotor;

  private int dashboardCounter;

  public Arm() {
    armMotor = new TalonFX(Constants.ArmConstants.CanIDs.armMotor);
    armMotor.setInverted(false);
    armMotor.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    if(dashboardCounter++ >= 5){

      dashboardCounter = 0;
    }
    // This method will be called once per scheduler run
  }
  public void setArmPosition(double position){
    armMotor.set(ControlMode.Position, position);
  }
  
  public double getArmPosition(){
    return armMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public void setArmSpeed(double speed){
    armMotor.set(ControlMode.PercentOutput, speed);
  }


}


