// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  private TalonSRX spindexerMotor;

  private Solenoid spindexerBar;

  private int dashboardCounter = 0;

  public Spindexer() {
    spindexerMotor = new TalonSRX(23);
    spindexerMotor.setInverted(false);
    spindexerMotor.setNeutralMode(NeutralMode.Coast);

    spindexerBar = new Solenoid(PneumaticsModuleType.REVPH, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(dashboardCounter++ >= 5){

      dashboardCounter = 0;
    }
  }

  public void setSpindexerSpeed(double speed){
    spindexerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void deploySpindexerBar(boolean toggle){
    spindexerBar.set(toggle);
  }
}
