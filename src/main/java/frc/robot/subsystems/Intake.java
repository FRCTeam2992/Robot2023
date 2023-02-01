// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Spindexer. */
  private TalonSRX intakeMotor;

  private Solenoid intakeSolenoid;

  private int dashboardCounter = 0;

  public enum IntakeStates {
    In,
    Out
  }

  public Intake() {
    intakeMotor = new TalonSRX(Constants.IntakeConstants.CanIDs.intakeMotor);
    intakeMotor.setInverted(false);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(dashboardCounter++ >= 5){

      dashboardCounter = 0;
    }
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void deployIntake(IntakeStates toggle){
    switch(toggle){
      case In:
        intakeSolenoid.set(true);
        break;
      case Out:
        
    }
  }
}
