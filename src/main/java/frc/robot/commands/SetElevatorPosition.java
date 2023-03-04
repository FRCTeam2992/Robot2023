// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends CommandBase {
  /** Creates a new SetElevatorPosition. */
  private Elevator mElevator;

  private double mPosition;
  private boolean holdPosition = true; // By default we hold the position and command doesnt end.

  public SetElevatorPosition(Elevator subsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = subsystem;
    mPosition = position;
    holdPosition = true;

    addRequirements(mElevator);
  }

  public SetElevatorPosition(Elevator subsystem, double position, boolean hold) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = subsystem;
    mPosition = position;
    holdPosition = hold;

    addRequirements(mElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mElevator.setElevatorPosition(mPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (holdPosition) {
      return false;
    } else {
      return mElevator.atPosition();
    }
  }
}
