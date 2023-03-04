// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.lib.manipulator.Waypoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveTowerFromSpindexerToBackstop extends SequentialCommandGroup {
  /** Creates a new MoveTowerFromSpindexerToBackstop. */
  public MoveTowerFromSpindexerToBackstop(Elevator mElevator, Arm mArm) {
    addCommands(
        new UnsafeMoveTowerToPosition(
            mElevator, mArm,
            new Waypoint(
                Elevator.ElevatorPosition.INTAKE_BACKSTOP.positionInches,
                Arm.ArmPosition.SPINDEXER_GRAB.positionDegrees)),
        new UnsafeMoveTowerToPosition(
            mElevator, mArm,
            new Waypoint(
                Elevator.ElevatorPosition.INTAKE_BACKSTOP.positionInches,
                Arm.ArmPosition.INTAKE_BACKSTOP.positionDegrees)));
  }
}
