// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spindexer;

public class MoveSpindexer extends CommandBase {
  /** Creates a new MoveSpindexer. */
  private Spindexer mSpindexer;

  private double mSpindexerSpeed;

  public MoveSpindexer(Spindexer subsystem, double spindexerspeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSpindexer = subsystem;
    mSpindexerSpeed = spindexerspeed;

    addRequirements(mSpindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSpindexer.setSpindexerSpeed(mSpindexerSpeed);
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
