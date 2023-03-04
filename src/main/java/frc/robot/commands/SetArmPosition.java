// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends CommandBase {
  /** Creates a new SetArmPosition. */
  private Arm mArm;
  private double mAngle;
  private boolean holdPosition = true; // By default command stays running and holds position unless constructer called
                                       // with false here

  public SetArmPosition(Arm subsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mArm = subsystem;
    mAngle = angle;

    addRequirements(mArm);
  }

  public SetArmPosition(Arm subsystem, double angle, boolean hold) {
    mArm = subsystem;
    mAngle = angle;
    holdPosition = hold;

    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mArm.setArmPosition(mAngle);
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
      return mArm.atPosition();
    }
  }
}
