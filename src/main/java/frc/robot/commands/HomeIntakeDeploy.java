// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeDeploy;

public class HomeIntakeDeploy extends CommandBase {
  /** Creates a new HomeIntakedeploy. */
  private IntakeDeploy mIntakeDeploy;

  public HomeIntakeDeploy(IntakeDeploy subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntakeDeploy = subsystem;
    addRequirements(mIntakeDeploy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntakeDeploy.goingToHome = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeDeploy.setIntakeDeploySpeed(-.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (mIntakeDeploy.getIntakeDeployLimitSwitchRaw()) {
      mIntakeDeploy.setIntakeDeployEncoderPosition(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mIntakeDeploy.getIntakeDeployLimitSwitchRaw();
  }
}
