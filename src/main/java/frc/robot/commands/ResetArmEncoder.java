// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm.EncoderState;

public class ResetArmEncoder extends CommandBase {

  private Arm mArm;
  private Elevator mElevator;

  private boolean unsafeToFix = false;

  /** Creates a new ResetArmEncoder. */
  public ResetArmEncoder(Arm arm, Elevator elev) {
    mArm = arm;
    mElevator = elev;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    unsafeToFix = false; // Assume it's safe to fix the slop unless we find a reason not to
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArm.clearMotorEncoder(); // Tell arm not to trust prior calibration
    mArm.initArmMotorEncoder(); // Try once more to init
    unsafeToFix = false; // Assume its safe to try unless we find something
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mArm.motorEncoderCalibrated() == EncoderState.CANCODER_FAILED) {
      // CanCoder is broken so PRAY we are already OK
      unsafeToFix = true;
      return;
    }

    if (mArm.motorEncoderCalibrated() == EncoderState.CALIBRATED) {
      return; // Already done -- no work to do
    }

    double curPos = mArm.getArmCANCoderPositionCorrected();
    if ((curPos <= Constants.ArmConstants.ArmSlopConstants.topZoneHiEdge) &&
        (curPos >= Constants.ArmConstants.ArmSlopConstants.topZoneLowEdge - 1.0)) {
      // In the top slop zone so push motor down slowly
      mArm.setArmSpeed(-0.1);
    } else if ((curPos <= Constants.ArmConstants.ArmSlopConstants.bottomZoneHiEdge + 1.0) &&
        (curPos >= Constants.ArmConstants.ArmSlopConstants.bottomZoneLowEdge)) {
      // In the bottom slop zone -- only do this if elevator high enough
      if (mElevator.getElevatorInches() > frc.lib.manipulator.Constants.AVOID_HEIGHT_INCHES) {
        mArm.setArmSpeed(0.1);
      } else {
        unsafeToFix = true; // In bottom slop zone and too low to safely move so live with it
      }
    } else {
      // Not in a slop zone so stop motor and init
      mArm.setArmSpeed(0.0);
      mArm.initArmMotorEncoder();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArm.setArmSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (unsafeToFix || mArm.motorEncoderCalibrated() == EncoderState.CALIBRATED);
  }
}
