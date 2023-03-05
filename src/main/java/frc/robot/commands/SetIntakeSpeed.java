// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntakeSpeed extends CommandBase {
  /** Creates a new SetIntakeSpeed. */
  private Intake mIntake;

  private double mIntakeTopSpeed;
  private double mIntakeBottomSpeed;

  public SetIntakeSpeed(Intake subsystem, double intakeTopSpeed, double intakeBottomSpeed) {
    mIntake = subsystem;
    mIntakeTopSpeed = intakeTopSpeed;
    mIntakeBottomSpeed = intakeBottomSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setIntakeTopSpeed(mIntakeTopSpeed);
    mIntake.setIntakeBottomSpeed(mIntakeBottomSpeed);
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
