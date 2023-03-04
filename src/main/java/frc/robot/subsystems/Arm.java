// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX armMotor;

  private CANCoder armEncoder;

  private int dashboardCounter;

  private double targetAngleDeg = 0;

  public enum ArmPosition {
    PARALLEL_TO_ELEVATOR(45.0),
    MOVEMENT_THRESHOLD_2(15.0),
    MOVEMENT_THRESHOLD_6(27.0),
    MOVEMENT_THRESHOLD_9(90.0);

    public final double positionDegrees;

    private ArmPosition(double positionDegrees) {
      this.positionDegrees = positionDegrees;
    }
  }

  public Arm() {
    armMotor = new TalonFX(Constants.ArmConstants.DeviceIDs.armMotor);
    armMotor.setInverted(false);
    armMotor.setNeutralMode(NeutralMode.Brake);

    armEncoder = new CANCoder(Constants.ArmConstants.DeviceIDs.armEncoder);
    armEncoder.configSensorDirection(true);
    armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    // Wait for CANCoder config to take effect
    Timer.delay(0.5);

    setArmMotorEncoder();
    setPIDConstants();
  }

  @Override
  public void periodic() {
    if (dashboardCounter++ >= 5) {

      if (hasArmMotorReset()) {
        setArmMotorEncoder();
      }

      SmartDashboard.putNumber("Arm CANcoder", getArmCANCoderPositionCorrected());
      SmartDashboard.putNumber("Arm Motor Encoder Raw", getArmMotorPositionRaw());

      SmartDashboard.putNumber("Arm Motor Encoder Degrees", getArmMotorPositionDeg());

      dashboardCounter = 0;
    }
    // This method will be called once per scheduler run
  }

  public void setArmPosition(double angle) {
    targetAngleDeg = angle;
    double position = angle * Constants.ArmConstants.motorEncoderClicksPerDegree;
    armMotor.set(ControlMode.MotionMagic, position);
  }

  public double getArmMotorPositionRaw() {
    return armMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getArmMotorPositionDeg() {
    return getArmMotorPositionRaw() / Constants.ArmConstants.motorEncoderClicksPerDegree;
  }

  public void setArmSpeed(double speed) {
    if (getArmMotorPositionDeg() > Constants.ArmConstants.Limits.softStopTop) {
      speed = Math.min(0.0, speed);
    } else if (getArmMotorPositionDeg() < Constants.ArmConstants.Limits.softStopBottom) {
      speed = Math.max(0.0, speed);
    }
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getArmCANCoderPositionRaw() {
    return armEncoder.getAbsolutePosition();
  }

  public double getArmCANCoderPositionCorrected() {
    return armEncoder.getAbsolutePosition() + Constants.ArmConstants.CANCoderOffset;
  }

  public boolean atPosition() {
    return (Math.abs(targetAngleDeg - getArmMotorPositionDeg()) < Constants.ArmConstants.armAngleToleranceDeg);
  }

  public void onDisable() {
    setArmSpeed(0.0);
  }

  public void setArmMotorEncoder() {
    double value = getArmCANCoderPositionCorrected();

    // If arm stowed at top of range, we have to adjust for mechanical chain slop
    if (value > Constants.ArmConstants.ArmSlopConstants.topZoneEdge) {
      value -= Constants.ArmConstants.ArmSlopConstants.topZoneAdjustment;
    } else if (value < Constants.ArmConstants.ArmSlopConstants.bottomZoneEdge) {
      value -= Constants.ArmConstants.ArmSlopConstants.bottomZoneAdjustment;
    }

    // Convert from degrees to encoder clicks
    value *= Constants.ArmConstants.motorEncoderClicksPerDegree;

    armMotor.setSelectedSensorPosition(value);
  }

  public boolean hasArmMotorReset() {
    return armMotor.hasResetOccurred();
  }

  public void setPIDConstants() {
    armMotor.config_kP(0, Constants.ArmConstants.PIDConstants.P);
    armMotor.config_kI(0, Constants.ArmConstants.PIDConstants.I);
    armMotor.config_kD(0, Constants.ArmConstants.PIDConstants.D);
    armMotor.config_kF(0, Constants.ArmConstants.PIDConstants.FF);

    armMotor.configMotionAcceleration(Constants.ArmConstants.PIDConstants.acceleration);
    armMotor.configMotionCruiseVelocity(Constants.ArmConstants.PIDConstants.cruiseVelocity);
  }

}
