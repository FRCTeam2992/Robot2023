// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.ButterflyWheels;

public class EndgameOnlyDeployButterflyWheels extends ConditionalCommand {
  /** Creates a new EndgameOnlyDeployButterflyWheels. */
  public EndgameOnlyDeployButterflyWheels(ButterflyWheels butterflyWheels, RobotState robotState) {
    super(
      new DeployButterflyWheels(butterflyWheels),
      new InstantCommand(),
      robotState::isInEndgameMode);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
