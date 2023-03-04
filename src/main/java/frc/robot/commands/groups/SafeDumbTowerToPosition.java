package frc.robot.commands.groups;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.manipulator.Waypoint;
import frc.lib.manipulator.WaypointSafety;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class SafeDumbTowerToPosition extends SequentialCommandGroup {
        Elevator mElevator;
        Arm mArm;
        Waypoint mEnd;

        public SafeDumbTowerToPosition(Elevator elevator, Arm arm, Waypoint point) {
                mElevator = elevator;
                mArm = arm;
                mEnd = point;
                addCommands(

                                // First we check is we are starting in a planned path zone and move safely out
                                // if needed
                                new SelectCommand(
                                                Map.ofEntries(
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.Safe,
                                                                                new InstantCommand()),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.NoFlyCrossbar,
                                                                                new UnsafeMoveTowerToPosition(elevator,
                                                                                                mArm,
                                                                                                frc.lib.manipulator.Constants.Waypoints.WAYPOINT_CROSSBAR_ENTRY,
                                                                                                false)),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.NoFlyRobotBase,
                                                                                new UnsafeMoveTowerToPosition(mElevator,
                                                                                                mArm,
                                                                                                frc.lib.manipulator.Constants.Waypoints.WAYPOINT_BASE_ENTRY,
                                                                                                false)),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.PlannedPathCrossbar,
                                                                                new UnsafeMoveTowerToPosition(elevator,
                                                                                                mArm,
                                                                                                frc.lib.manipulator.Constants.Waypoints.WAYPOINT_CROSSBAR_ENTRY,
                                                                                                false)),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.PlannedPathRobotBase,
                                                                                new UnsafeMoveTowerToPosition(mElevator,
                                                                                                mArm,
                                                                                                frc.lib.manipulator.Constants.Waypoints.WAYPOINT_BASE_ENTRY,
                                                                                                false))),
                                                this::checkStart),

                                // Next we move to our safe center waypoint
                                new UnsafeMoveTowerToPosition(mElevator, mArm,
                                                frc.lib.manipulator.Constants.Waypoints.WAYPOINT_BASE_AVOID,
                                                false),

                                // Next we check if we are ending in a Path Plan only zone and add that waypoint
                                // if required
                                new SelectCommand(
                                                Map.ofEntries(
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.Safe,
                                                                                new InstantCommand()),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.NoFlyCrossbar,
                                                                                new InstantCommand()),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.NoFlyRobotBase,
                                                                                new InstantCommand()),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.PlannedPathCrossbar,
                                                                                new UnsafeMoveTowerToPosition(elevator,
                                                                                                mArm,
                                                                                                frc.lib.manipulator.Constants.Waypoints.WAYPOINT_CROSSBAR_ENTRY,
                                                                                                false)),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.PlannedPathRobotBase,
                                                                                new UnsafeMoveTowerToPosition(mElevator,
                                                                                                mArm,
                                                                                                frc.lib.manipulator.Constants.Waypoints.WAYPOINT_BASE_ENTRY,
                                                                                                false))),
                                                this::checkEnd),

                                // And now we can move into the final spot if it is safe
                                new SelectCommand(
                                                Map.ofEntries(
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.Safe,
                                                                                new UnsafeMoveTowerToPosition(mElevator,
                                                                                                mArm, mEnd, true)),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.NoFlyCrossbar,
                                                                                new InstantCommand()),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.NoFlyRobotBase,
                                                                                new InstantCommand()),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.PlannedPathCrossbar,
                                                                                new UnsafeMoveTowerToPosition(elevator,
                                                                                                mArm, mEnd, true)),
                                                                Map.entry(WaypointSafety.WaypointSafetyClassification.PlannedPathRobotBase,
                                                                                new UnsafeMoveTowerToPosition(mElevator,
                                                                                                mArm, mEnd, true))),
                                                this::checkEnd)

                );
        }

        private WaypointSafety.WaypointSafetyClassification checkStart() {
                return WaypointSafety.nonSafeZones(
                                new Waypoint(mElevator.getElevatorInches(), mArm.getArmMotorPositionDeg()));
        }

        private WaypointSafety.WaypointSafetyClassification checkEnd() {
                return WaypointSafety.nonSafeZones(mEnd);
        }

}
