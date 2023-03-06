// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private Solenoid clawSolenoid;

  private int dashboardCounter;

  public enum ClawState {
    Opened(true),
    Closed(false);

    public final boolean solenoidSetting;

    private ClawState(boolean solenoidSetting) {
      this.solenoidSetting = solenoidSetting;
    }
  }

  public Claw() {
    clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClawConstants.DeviceIDs.clawSolenoid);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {

      dashboardCounter = 0;
    }
  }

  public void setClawState(ClawState state) {
    clawSolenoid.set(state.solenoidSetting);
  }

  public ClawState getClawState() {
    if (clawSolenoid.get()) {
      return ClawState.Opened;
    }
    return ClawState.Closed;
  }

  public void onDisable() {
    // Solenoid needs to be in false state when disabled.
    setClawState(ClawState.Closed);
  }
}
