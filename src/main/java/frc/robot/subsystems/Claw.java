// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private Solenoid armSolenoid;

  private int dashboardCounter;

  public enum ClawStates {
    Opened,
    Closed
  }

  public Claw() {
    armSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dashboardCounter++ >= 5) {

      dashboardCounter = 0;
    }
  }

  public void setClawState(ClawStates state) {
    switch (state) {
      case Opened:
        armSolenoid.set(true);
        break;
      case Closed:
        armSolenoid.set(false);
        break;
    }
  }
}
