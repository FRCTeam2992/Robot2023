// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMotorLead;
  private TalonFX elevatorMotorFollow;

  private Solenoid elevatorSolenoid;

  private DigitalOutput elevatorLimitSwitch;

  private int dashboardCounter = 0;

  public enum ElevatorState {
    Undeployed(false),
    Deployed(true);

    public final boolean solenoidSetting;

    private ElevatorState(boolean solenoidSetting) {
      this.solenoidSetting = solenoidSetting;
    }
  }

  public enum ElevatorPosition {
    HARD_STOP_TOP(32.25),
    HARD_STOP_BOTTOM(0.0),
    SOFT_STOP_TOP(31.75),
    SOFT_STOP_BOTTOM(0.5),
    CUBE_LOW(0.5),
    CUBE_MID(1.5),
    CUBE_TOP(16.5),
    CONE_LOW(0.5),
    CONE_MID(0.5),
    CONE_TOP(27.7),
    INTAKE_BACKSTOP(6.0),
    SPINDEXER_GRAB(3.0);

    public final double positionInches;

    private ElevatorPosition(double positionInches) {
      this.positionInches = positionInches;
    }
  }

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorLead = new TalonFX(Constants.ElevatorConstants.DeviceIDs.elevatorMotorLead);
    elevatorMotorLead.setInverted(TalonFXInvertType.CounterClockwise);
    elevatorMotorLead.setNeutralMode(NeutralMode.Brake);

    elevatorMotorFollow = new TalonFX(Constants.ElevatorConstants.DeviceIDs.elevatorMotorFollow);
    elevatorMotorFollow.setNeutralMode(NeutralMode.Brake);
    elevatorMotorFollow.set(TalonFXControlMode.Follower, elevatorMotorLead.getDeviceID());
    elevatorMotorFollow.setInverted(TalonFXInvertType.OpposeMaster);

    elevatorSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
        Constants.ElevatorConstants.DeviceIDs.elevatorSolenoid);

    elevatorLimitSwitch = new DigitalOutput(Constants.ElevatorConstants.DeviceIDs.elevatorLimitSwitch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {
      SmartDashboard.putNumber("Lead Elevator Encoder", getLeadElevatorPostion());
      SmartDashboard.putNumber("Follow Elevator Encoder", getFollowElevatorPostion());

      SmartDashboard.putNumber("Elevator Inches", getElevatorInches());

      dashboardCounter = 0;
    }
  }

  public void configureMotorFollowing() {
    elevatorMotorFollow.set(TalonFXControlMode.Follower, elevatorMotorLead.getDeviceID());
  }

  public double getLeadElevatorPostion() {
    return elevatorMotorLead.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getFollowElevatorPostion() {
    return elevatorMotorFollow.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getElevatorInches() {
    return encoderClicksToInches(getLeadElevatorPostion());
  }

  public void setElevatorSpeed(double speed) {
    if (getElevatorInches() < ElevatorPosition.SOFT_STOP_BOTTOM.positionInches) {
      speed = Math.max(0.0, speed);
    } else if (getElevatorInches() > ElevatorPosition.SOFT_STOP_TOP.positionInches) {
      speed = Math.min(0.0, speed);
    }
    elevatorMotorLead.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setElevatorPosition(double inches) {
    if (inches < ElevatorPosition.SOFT_STOP_BOTTOM.positionInches) {
      inches = ElevatorPosition.SOFT_STOP_BOTTOM.positionInches;
    } else if (inches > ElevatorPosition.SOFT_STOP_TOP.positionInches) {
      inches = ElevatorPosition.SOFT_STOP_TOP.positionInches;
    }
    elevatorMotorLead.set(TalonFXControlMode.MotionMagic, inchesToEncoderClicks(inches));
    System.out.println("MOVING: " + inchesToEncoderClicks(inches));
  }

  public void setElevatorState(ElevatorState state) {
    elevatorSolenoid.set(state.solenoidSetting);
  }

  public void deployElevator(boolean toggle) {
    elevatorSolenoid.set(toggle);
  }

  public boolean getElevatorSolenoidState() {
    return elevatorSolenoid.get();
  }

  public void onDisable() {
    setElevatorState(ElevatorState.Undeployed);
    setElevatorSpeed(0.0);
  }

  private double encoderClicksToInches(double encoderClicks) {
    return encoderClicks / Constants.ElevatorConstants.encoderClicksPerInch;
  }

  private double inchesToEncoderClicks(double inches) {
    return inches * Constants.ElevatorConstants.encoderClicksPerInch;
  }

  public void zeroElevatorEncoders() {
    elevatorMotorLead.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
    elevatorMotorFollow.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
  }

  private void setPIDConstants(TalonFX motor) {
    motor.config_kP(0, Constants.ElevatorConstants.PIDConstants.P);
    motor.config_kI(0, Constants.ElevatorConstants.PIDConstants.I);
    motor.config_kD(0, Constants.ElevatorConstants.PIDConstants.D);
    motor.config_kF(0, Constants.ElevatorConstants.PIDConstants.FF);

    motor.configMotionCruiseVelocity(Constants.ElevatorConstants.PIDConstants.cruiseVelocity);
    motor.configMotionAcceleration(Constants.ElevatorConstants.PIDConstants.acceleration);

  }
}
