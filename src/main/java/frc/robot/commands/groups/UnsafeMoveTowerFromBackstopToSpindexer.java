// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.lib.manipulator.Waypoint;
import frc.robot.Constants.TowerConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnsafeMoveTowerFromBackstopToSpindexer extends SequentialCommandGroup {
    /**
     * Creates a new MoveTowerFromSpindexerToBackstop.
     * Note: This command group should NEVER be attached to an arbitrary
     * input, as it does not respect no-fly zones. It is only to
     * be used within a planned sequence of moves with a known
     * safe starting point.
     */
    public UnsafeMoveTowerFromBackstopToSpindexer(Elevator mElevator, Arm mArm) {
        double waypointHeight = TowerConstants.intakeBackstop.height();
        double waypointAngle = TowerConstants.intakeGrabCone.angle();
        Waypoint waypoint = new Waypoint(waypointHeight, waypointAngle);
        Waypoint grab = TowerConstants.intakeGrabCone;
        addCommands(
                new UnsafeMoveTowerToPosition(mElevator, mArm, waypoint).asProxy(),
                new UnsafeMoveTowerToPosition(mElevator, mArm, grab).asProxy());
    }
}
