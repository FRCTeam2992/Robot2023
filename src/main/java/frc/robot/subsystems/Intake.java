// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Spindexer. */
  private CANSparkMax intakeMotorTop;
  private CANSparkMax intakeMotorBottom;

  private int dashboardCounter = 0;

  public Intake() {
    intakeMotorTop = new CANSparkMax(Constants.IntakeConstants.DeviceIDs.intakeMotorTop, MotorType.kBrushless);
    intakeMotorTop.setInverted(true);
    intakeMotorTop.setIdleMode(IdleMode.kCoast);

    intakeMotorBottom = new CANSparkMax(Constants.IntakeConstants.DeviceIDs.intakeMotorBottom, MotorType.kBrushless);
    intakeMotorBottom.setInverted(true);
    intakeMotorBottom.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {

      dashboardCounter = 0;
    }
  }

  public void setIntakeTopSpeed(double speed) {
    intakeMotorTop.set(speed);
  }

  public void setIntakeBottomSpeed(double speed) {
    intakeMotorBottom.set(speed);
  }

  public void onDisable() {
    setIntakeBottomSpeed(0.0);
    setIntakeTopSpeed(0.0);
  }
}
