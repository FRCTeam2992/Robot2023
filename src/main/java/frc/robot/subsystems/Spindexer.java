// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  private CANSparkMax spindexerMotor;

  private int dashboardCounter = 0;

  public Spindexer() {
    spindexerMotor = new CANSparkMax(Constants.SpindexerConstants.DeviceIDs.spindexerMotor, MotorType.kBrushless);
    spindexerMotor.setInverted(false);
    spindexerMotor.setIdleMode(IdleMode.kBrake);

    setSpindexerPID();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {
      dashboardCounter = 0;
    }

  }

  public void setSpindexerSpeed(double speed) {
    spindexerMotor.set(speed);
  }

  public void onDisable() {
    setSpindexerSpeed(0.0);
  }

  public double getSpindexerEncoderPosition() {
    return spindexerMotor.getEncoder().getPosition();
  }

  public void setEncoderPosition(double position) {
    spindexerMotor.getEncoder().setPosition(position);
  }

  public void setSpindexerToPosition(double positionDegrees) {
    positionDegrees = (positionDegrees * Constants.SpindexerConstants.gearRatio) / 360.0;
    spindexerMotor.getPIDController().setReference(positionDegrees, ControlType.kPosition);
  }

  public void setSpindexerVelocity(double velocityRPM) {
    velocityRPM = (velocityRPM * Constants.SpindexerConstants.gearRatio);
    spindexerMotor.getPIDController().setReference(velocityRPM, ControlType.kVelocity);
  }

  public void setSpindexerPID() {
    spindexerMotor.getPIDController().setP(Constants.SpindexerConstants.PIDConstants.P);
    spindexerMotor.getPIDController().setI(Constants.SpindexerConstants.PIDConstants.I);
    spindexerMotor.getPIDController().setD(Constants.SpindexerConstants.PIDConstants.D);
    spindexerMotor.getPIDController().setFF(Constants.SpindexerConstants.PIDConstants.FF);
  }
}
