package frc.lib.manipulator;

public final class Constants {
    public static final double INCHES_MAX = 32.0;
    public static final double DEGREES_MAX = 212.0;

    public static final class Zones {
        // Corner x, corner y, width, height
        public static final NoFlyZone ZONE_1 = new NoFlyZone(0.0, 0.0, 2.0, 15.0);
        public static final NoFlyZone ZONE_2 = new NoFlyZone(2.0, 15.0, 6.0, 27.0);
        public static final NoFlyZone ZONE_3 = new NoFlyZone(6.0, 27.0, 9.0, 90.0);
        public static final NoFlyZone ZONE_4 = new NoFlyZone(
                INCHES_MAX - 6.0, DEGREES_MAX - 10.0,
                INCHES_MAX - 3.0, DEGREES_MAX);
    }

    public static final class Boundaries {
        public static final NoFlyBoundary BOUNDARY_Z2L = new NoFlyBoundary(
                NoFlyBoundary.NoFlyBoundaryType.AT_ANGLE,
                0.0, 6.0, 15.0);
        public static final NoFlyBoundary BOUNDARY_Z3L = new NoFlyBoundary(
                NoFlyBoundary.NoFlyBoundaryType.AT_ANGLE,
                0.0, 6.0, 27.0);
        public static final NoFlyBoundary BOUNDARY_Z3R = new NoFlyBoundary(
                NoFlyBoundary.NoFlyBoundaryType.AT_ANGLE,
                0.0, 6.0, 90.0);
        public static final NoFlyBoundary BOUNDARY_Z4B = new NoFlyBoundary(
                NoFlyBoundary.NoFlyBoundaryType.AT_HEIGHT,
                DEGREES_MAX - 10.0, DEGREES_MAX, INCHES_MAX - 6.0);
        public static final NoFlyBoundary BOUNDARY_Z4T = new NoFlyBoundary(
                NoFlyBoundary.NoFlyBoundaryType.AT_HEIGHT,
                DEGREES_MAX - 10.0, DEGREES_MAX, INCHES_MAX - 3.0);
    }

    public static final class Waypoints {
        public static final Waypoint WAYPOINT_Z2TL = new Waypoint(6.0, 15.0);
        public static final Waypoint WAYPOINT_Z3TL = new Waypoint(9.0, 27.0);
        public static final Waypoint WAYPOINT_Z3TR = new Waypoint(9.0, 90.0);
        public static final Waypoint WAYPOINT_Z4BL = new Waypoint(INCHES_MAX - 6.0, DEGREES_MAX - 10.0);
        public static final Waypoint WAYPOINT_Z4TL = new Waypoint(INCHES_MAX - 3.0, DEGREES_MAX - 10.0);
    }
}
