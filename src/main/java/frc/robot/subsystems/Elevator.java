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

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorLead = new TalonFX(Constants.ElevatorConstants.DeviceIDs.elevatorMotorLead);
    elevatorMotorLead.setInverted(TalonFXInvertType.Clockwise);
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
      SmartDashboard.putNumber("Elevator Encoder", getElevatorPostion());
      SmartDashboard.putNumber("Elevator Inches", getElevatorInches());

      dashboardCounter = 0;
    }
  }

  public void configureMotorFollowing() {
    elevatorMotorFollow.set(TalonFXControlMode.Follower, elevatorMotorLead.getDeviceID());
  }

  public double getElevatorPostion() {
    return elevatorMotorLead.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getElevatorInches(){
    return encoderClicksToInches(getElevatorPostion());
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotorLead.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setElevatorPosition(double inches) {
    elevatorMotorLead.set(TalonFXControlMode.MotionMagic, inchesToEncoderClicks(inches));
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
}
