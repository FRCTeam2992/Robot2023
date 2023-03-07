// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class HoldElevator extends CommandBase {
  /** Creates a new StopElevator. */
  private Elevator mElevator;

  private Timer timer;

  public HoldElevator(Elevator subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = subsystem;
    timer = new Timer();
    addRequirements(mElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    mElevator.setElevatorSpeed(0.0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > Constants.ElevatorConstants.holdPositionMaxTime) {
      mElevator.setElevatorSpeed(0.0);
    } else {
      if (timer.get() > 0.150) {
        mElevator.holdElevator();
      }
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
