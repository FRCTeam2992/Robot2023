// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMotor;

  private Solenoid elevatorSolenoid;

  private int dashboardCounter = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new TalonFX(Constants.ElevatorConstants.CanIDs.elevatorMotor);
    elevatorMotor.setInverted(false);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(dashboardCounter++ >= 5){
      SmartDashboard.putNumber("Elevator Encoder", getElevatorPostion());

      dashboardCounter = 0;

      
    }
  }

  public double getElevatorPostion(){
    return elevatorMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public void setElevatorSpeed(double speed){
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }


  public void setElevatorPosition(double position){
    elevatorMotor.set(ControlMode.Position, position);
  }

  public void deployElevator(boolean toggle){
    elevatorSolenoid.set(toggle);
  }}
