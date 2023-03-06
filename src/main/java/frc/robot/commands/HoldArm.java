// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class HoldArm extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm mArm;

  private Timer timer;

  public HoldArm(Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mArm = subsystem;
    timer = new Timer();
    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > Constants.ArmConstants.holdPositionMaxTime) {
      mArm.setArmSpeed(0.0);
    } else {
      mArm.holdArm();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
