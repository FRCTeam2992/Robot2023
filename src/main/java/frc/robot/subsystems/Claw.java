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
  private Solenoid armSolenoid60;
  private Solenoid armSolenoid40;


  private int dashboardCounter;

  public enum ClawStates {
    Opened,
    Closed_Cone,
    Closed_Cube,
    Unused
  }

  public Claw() {
    armSolenoid60 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClawConstants.DeviceIDs.ClawSolenoid60);
    armSolenoid40 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClawConstants.DeviceIDs.ClawSolenoid40);

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
        armSolenoid60.set(true);
        armSolenoid40.set(true);
        break;
      case Closed_Cone:
        armSolenoid60.set(false);
        armSolenoid40.set(false);
        break;
      case Closed_Cube:
        armSolenoid60.set(false);
        armSolenoid40.set(true);
        break;
      case Unused:
        armSolenoid60.set(true);
        armSolenoid40.set(false);
        break;
    } 
  }
}
