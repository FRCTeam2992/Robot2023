package frc.lib.manipulator;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public final class Constants {
        public static final double MAX_HEIGHT_INCHES = Elevator.ElevatorPosition.HARD_STOP_TOP.positionInches;
        public static final double MAX_ANGLE_DEGREES = Arm.ArmPosition.TOP_HARD_STOP.positionDegrees;
        public static final double AVOID_HEIGHT_INCHES = 9.0;
        public static final double BASE_AVOID_INCHES = 10.0;
        public static final double BASE_AVOID_DEGREES = 150.0;
        public static final double BASE_ENTRY_INCHES = 10.0;
        public static final double BASE_ENTRY_DEGREES = Arm.ArmPosition.SPINDEXER_GRAB.positionDegrees;
        public static final double CROSSBAR_ENTRY_INCHES = 3.0;
        public static final double CROSSBAR_ENTRY_DEGREES = 180.0;
        public static final double CROSSBAR_AVOID_BOTTOM_INCHES = 3.0;
        public static final double CROSSBAR_AVOID_TOP_INCHES = 9.0;
        public static final double CROSSBAR_AVOID_DEGREES = 205.0;

        public static final class NoFlyZones {
                // Corner x, corner y, width, height
                public static final NoFlyZone INSIDE_ROBOT_1 = new NoFlyZone(-10.0, -20.0, 2.0, 15.0);
                public static final NoFlyZone INSIDE_ROBOT_2 = new NoFlyZone(-10.0, 15.0, 6.0, 27.0);
                public static final NoFlyZone INSIDE_ROBOT_3 = new NoFlyZone(-10.0, 27.0, 9.0, 90.0);
                public static final NoFlyZone ELEVATOR_CROSSBAR = new NoFlyZone(
                                3.0, 205.0, 9.0, MAX_ANGLE_DEGREES + 20.0);
        }

        public static final class PlannedPathZones {
                public static final NoFlyZone ROBOT_BASE = new NoFlyZone(-10.0, -20.0, 9.0, 27.0);
                public static final NoFlyZone BELOW_CROSSBAR = new NoFlyZone(
                                -10.0, CROSSBAR_AVOID_DEGREES, 3.0, MAX_ANGLE_DEGREES + 20.0);
        }

        public static final class Boundaries {
                public static final NoFlyBoundary BOUNDARY_BASE = new NoFlyBoundary(
                                NoFlyBoundary.NoFlyBoundaryType.AT_ANGLE,
                                0.0, 6.0, 90.0);
                public static final NoFlyBoundary BOUNDARY_CROSSBAR = new NoFlyBoundary(
                                NoFlyBoundary.NoFlyBoundaryType.AT_HEIGHT,
                                MAX_ANGLE_DEGREES - 10.0, MAX_ANGLE_DEGREES, MAX_HEIGHT_INCHES - 3.0);
        }

        public static final class Waypoints {
                public static final Waypoint WAYPOINT_BASE_ENTRY = new Waypoint(BASE_ENTRY_INCHES, BASE_ENTRY_DEGREES);
                public static final Waypoint WAYPOINT_BASE_AVOID = new Waypoint(BASE_AVOID_INCHES, BASE_AVOID_DEGREES);
                public static final Waypoint WAYPOINT_CROSSBAR_ENTRY = new Waypoint(
                                CROSSBAR_ENTRY_INCHES,
                                CROSSBAR_ENTRY_DEGREES);
        }
}
