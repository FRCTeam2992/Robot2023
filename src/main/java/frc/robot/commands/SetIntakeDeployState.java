// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployState;

public class SetIntakeDeployState extends CommandBase {
  /** Creates a new SetIntakeDeployState. */
  private IntakeDeploy mIntakeDeploy;

  private IntakeDeployState mState;

  public SetIntakeDeployState(IntakeDeploy subsystem, IntakeDeployState state) {
    mIntakeDeploy = subsystem;
    mState = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeDeploy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      mIntakeDeploy.setIntakeDeployState(mState);
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
      return false;
  }
}
