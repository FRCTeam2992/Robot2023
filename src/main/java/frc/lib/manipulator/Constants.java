package frc.lib.manipulator;

public final class Constants {
        public static final double INCHES_MAX = 32.0;
        public static final double DEGREES_MAX = 212.0;

        public static final class NoFlyZones {
                // Corner x, corner y, width, height
                public static final NoFlyZone INSIDE_ROBOT = new NoFlyZone(6.0, 27.0, 9.0, 90.0);
                public static final NoFlyZone ELEVATOR_CROSSBAR = new NoFlyZone(
                                INCHES_MAX - 6.0, DEGREES_MAX - 10.0,
                                INCHES_MAX - 3.0, DEGREES_MAX);
        }

        public static final class PlannedPathZones {
                public static final NoFlyZone ROBOT_BASE = new NoFlyZone(0.0, 0.0, 9.0, 27.0);
                public static final NoFlyZone ABOVE_CROSSBAR = new NoFlyZone(
                                INCHES_MAX - 3.0, DEGREES_MAX - 10.0,
                                INCHES_MAX, DEGREES_MAX);
        }

        public static final class Boundaries {
                public static final NoFlyBoundary BOUNDARY_BASE = new NoFlyBoundary(
                                NoFlyBoundary.NoFlyBoundaryType.AT_ANGLE,
                                0.0, 6.0, 90.0);
                public static final NoFlyBoundary BOUNDARY_CROSSBAR = new NoFlyBoundary(
                                NoFlyBoundary.NoFlyBoundaryType.AT_HEIGHT,
                                DEGREES_MAX - 10.0, DEGREES_MAX, INCHES_MAX - 3.0);
        }

        public static final class Waypoints {
                public static final Waypoint WAYPOINT_BASE_ENTRY = new Waypoint(10.0, 15.0);
                public static final Waypoint WAYPOINT_BASE_AVOID = new Waypoint(9.0, 90.0);
                public static final Waypoint WAYPOINT_CROSSBAR_ENTRY = new Waypoint(
                                INCHES_MAX - 2.0,
                                DEGREES_MAX - 10.0);
                public static final Waypoint WAYPOINT_CROSSBAR_AVOID = new Waypoint(
                                INCHES_MAX - 3.0,
                                DEGREES_MAX - 10.0);
        }
}
