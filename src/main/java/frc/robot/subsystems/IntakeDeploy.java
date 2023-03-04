// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeploy extends SubsystemBase {
  private CANSparkMax intakeDeployMotor;

  private DigitalOutput intakeDeployLimitSwitch;
  private Debouncer intakeDeployDebouncer;

  private double dashboardCounter = 0;

  public enum IntakeDeployState {
    GroundIntake(22.0), // dummy value
    LoadStation(10.0), // dummy value
    Homed(0.0);

    public final double intakeSpot;

    private IntakeDeployState(double intakeSpot) {
      this.intakeSpot = intakeSpot;
    }
  }

  public boolean goingToHome = true;

  /** Creates a new IntakeDeploy. */
  public IntakeDeploy() {
    intakeDeployMotor = new CANSparkMax(Constants.IntakeDeployConstants.DeviceIDs.intakeDeployMotor,
        MotorType.kBrushless);
    intakeDeployMotor.setInverted(false);
    intakeDeployMotor.setIdleMode(IdleMode.kBrake);

    intakeDeployLimitSwitch = new DigitalOutput(Constants.IntakeDeployConstants.DeviceIDs.intakeDeployLimitSwitch);
    intakeDeployDebouncer = new Debouncer(.01, DebounceType.kBoth);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ > 5) {

      // SmartDashboard.putBoolean("Intake Deploy Limit Db",
      // getIntakeDeployLimitSwitchDebounced());

      dashboardCounter = 0;
    }
    SmartDashboard.putBoolean("Intake Deploy Limit Raw",
        getIntakeDeployLimitSwitchRaw());
    // if (getIntakeDeployLimitSwitchRaw() && goingToHome) {

    // goingToHome = false; // it has reached home
    // }

  }

  public boolean getIntakeDeployLimitSwitchRaw() {
    return !intakeDeployLimitSwitch.get();
  }

  // public boolean getIntakeDeployLimitSwitchDebounced() {
  // return
  // intakeDeployDebouncer.calculate(getIntakeDeployLimitSwitchDebounced());
  // }

  public void setIntakeDeploySpeed(double speed) {
    intakeDeployMotor.set(speed);
  }

  public void setIntakeDeployEncoderPosition(double position) {
    intakeDeployMotor.getEncoder().setPosition(position);
  }

  public void setIntakeDeployPosition(double position) {
    intakeDeployMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void setIntakeDeployState(IntakeDeployState state) {
    if (state == IntakeDeployState.Homed) {
      goingToHome = true;
    }
    setIntakeDeployPosition(state.intakeSpot);
  }

  public void setPIDConstants() {
    intakeDeployMotor.getPIDController().setP(Constants.IntakeDeployConstants.PIDConstants.P);
    intakeDeployMotor.getPIDController().setI(Constants.IntakeDeployConstants.PIDConstants.I);
    intakeDeployMotor.getPIDController().setD(Constants.IntakeDeployConstants.PIDConstants.D);
    intakeDeployMotor.getPIDController().setFF(Constants.IntakeDeployConstants.PIDConstants.FF);
  }
}
