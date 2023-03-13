// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ToggleClawState extends CommandBase {
  /** Creates a new SetClawState. */
  private Claw mClaw;

  public ToggleClawState(Claw subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mClaw = subsystem;

    addRequirements(mClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mClaw.toggleClawState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
