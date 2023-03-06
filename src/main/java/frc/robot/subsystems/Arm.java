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

  public enum EncoderState {
    UNKNOWN,
    CANCODER_FAILED,
    TOP_SLOP_ZONE,
    BOTTOM_SLOP_ZONE,
    CALIBRATED
  }

  private EncoderState motorEncoderConfidentCalibrated = EncoderState.UNKNOWN;

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

    initArmMotorEncoder();
    setPIDConstants();
  }

  @Override
  public void periodic() {
    if (dashboardCounter++ >= 5) {

      if (hasArmMotorReset()) {
        initArmMotorEncoder();
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

  public EncoderState motorEncoderCalibrated() {
    return motorEncoderConfidentCalibrated;
  }

  public void clearMotorEncoder() {
    motorEncoderConfidentCalibrated = EncoderState.UNKNOWN;
  }

  public boolean atPosition() {
    return (Math.abs(targetAngleDeg - getArmMotorPositionDeg()) < Constants.ArmConstants.armAngleToleranceDeg);
  }

  public void onDisable() {
    setArmSpeed(0.0);
  }

  public void initArmMotorEncoder() {
    double value = getArmCANCoderPositionCorrected();

    if (!hasArmMotorReset() && motorEncoderConfidentCalibrated == EncoderState.CALIBRATED) {
      // We previously had a good reset and no motor reset so still good
      return;
    }

    // If arm stowed at top of range, we have to adjust for mechanical chain slop
    if (Math.abs(armEncoder.getLastTimestamp() - Timer.getFPGATimestamp()) > 0.5) {
      // Not getting current Cancoder settings
      // Assume we are at Match Start position and PRAY!
      System.out.println("+++++++++++++++++> ARM ENCODER NO CURRENT READING");
      value = Constants.ArmConstants.Limits.hardStopTop - Constants.ArmConstants.ArmSlopConstants.topZoneAdjustment;
      motorEncoderConfidentCalibrated = EncoderState.CANCODER_FAILED;
    }
    if (value >= Constants.ArmConstants.ArmSlopConstants.topZoneHiEdge) {
      // Above the top slop zone -- apply adjustment
      value -= Constants.ArmConstants.ArmSlopConstants.topZoneAdjustment;
      motorEncoderConfidentCalibrated = EncoderState.CALIBRATED;
    } else if (value >= Constants.ArmConstants.ArmSlopConstants.topZoneLowEdge) {
      // In the top slop zone -- assume midpoint but note we don't have good reading
      value -= (Constants.ArmConstants.ArmSlopConstants.topZoneAdjustment / 2.0);
      motorEncoderConfidentCalibrated = EncoderState.TOP_SLOP_ZONE;
      System.out.println("++++++++++++> Arm init in top slop zone");
    } else if (value >= Constants.ArmConstants.ArmSlopConstants.bottomZoneHiEdge) {
      // In the mid zone -- no offset to motor encoder needed
      motorEncoderConfidentCalibrated = EncoderState.CALIBRATED;
    } else if (value >= Constants.ArmConstants.ArmSlopConstants.bottomZoneLowEdge) {
      // In the bottom slop zone -- assume midount but note we dont have good reading
      value -= Constants.ArmConstants.ArmSlopConstants.bottomZoneAdjustment / 2.0;
      motorEncoderConfidentCalibrated = EncoderState.BOTTOM_SLOP_ZONE;
      System.out.println("++++++++++++++> Arm init in bottom slop zone");
    } else {
      // Below the bottom slop zone -- apply adjustment
      value -= Constants.ArmConstants.ArmSlopConstants.bottomZoneAdjustment;
      motorEncoderConfidentCalibrated = EncoderState.CALIBRATED;
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
