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
public class UnsafeMoveTowerFromSpindexerToBackstop extends SequentialCommandGroup {
    /**
     * Creates a new MoveTowerFromSpindexerToBackstop.
     * Note: This command group should NEVER be attached to an arbitrary
     * input, as it does not respect no-fly zones. It is only to
     * be used within a planned sequence of moves with a known
     * safe starting point.
     */
    public UnsafeMoveTowerFromSpindexerToBackstop(Elevator mElevator, Arm mArm) {
        addCommands(
                new UnsafeMoveTowerToPosition(
                        mElevator, mArm,
                        new Waypoint(
                                frc.robot.Constants.TowerConstants.intakeBackstop.height(),
                                frc.robot.Constants.TowerConstants.intakeGrab.angle()),
                        false),
                new UnsafeMoveTowerToPosition(
                        mElevator, mArm,
                        frc.robot.Constants.TowerConstants.intakeBackstop,
                        true));
    }
}
