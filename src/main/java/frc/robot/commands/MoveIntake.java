// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveIntake extends CommandBase {
  /** Creates a new MoveIntake. */
  private Intake mIntake;

  private double mTopIntakeSpeed;
  private double mBottomIntakeSpeed;

  public MoveIntake(Intake subsystem, double topIntakeSpeed, double bottomIntakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = subsystem;
    mTopIntakeSpeed = topIntakeSpeed;
    mBottomIntakeSpeed = bottomIntakeSpeed;

    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setIntakeTopSpeed(mTopIntakeSpeed);
    mIntake.setIntakeBottomSpeed(mBottomIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
