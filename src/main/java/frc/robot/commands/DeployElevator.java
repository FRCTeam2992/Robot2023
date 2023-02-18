// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class DeployElevator extends CommandBase {
  /** Creates a new DeployIntake. */
  private Elevator mElevator;

  private ElevatorState mElevatorState;

  public DeployElevator(Elevator subsystem, ElevatorState elevatorState) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = subsystem;

    mElevatorState = elevatorState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mElevator.setElevatorState(mElevatorState);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
