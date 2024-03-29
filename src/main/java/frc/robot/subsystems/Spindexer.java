// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  private VictorSPX spindexerMotor;

  private int dashboardCounter = 0;

  public Spindexer() {
    spindexerMotor = new VictorSPX(Constants.SpindexerConstants.DeviceIDs.spindexerMotor);
    spindexerMotor.setInverted(false);
    spindexerMotor.setNeutralMode(NeutralMode.Brake);
    spindexerMotor.configOpenloopRamp(0.25);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {
      dashboardCounter = 0;
    }
  }

  public void setSpindexerSpeed(double speed) {
    spindexerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void onDisable() {
    setSpindexerSpeed(0.0);
  }
}
