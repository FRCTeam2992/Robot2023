package frc.lib.manipulator;

public class WaypointSafety {
    static public enum WaypointSafetyClassification {
        Safe,
        NoFlyRobotBase,
        NoFlyCrossbar,
        PlannedPathRobotBase,
        PlannedPathCrossbar
    }

    static public WaypointSafetyClassification nonSafeZones(Waypoint p) {
        // System.out.println(">>>>>>>>>>>>>>>>> WayPointSafety: testing point: {" +
        // p.height() + "," + p.angle() + ")");
        if (Constants.NoFlyZones.INSIDE_ROBOT_1.contains(p) ||
                Constants.NoFlyZones.INSIDE_ROBOT_2.contains(p) ||
                Constants.NoFlyZones.INSIDE_ROBOT_3.contains(p)) {
            // System.out.println("Returning NoFlyRobotBase");
            return WaypointSafetyClassification.NoFlyRobotBase;
        }
        if (Constants.NoFlyZones.ELEVATOR_CROSSBAR.contains(p)) {
            // System.out.println("Returning NoFlyCrossbar");
            return WaypointSafetyClassification.NoFlyCrossbar;
        }
        if (Constants.PlannedPathZones.ROBOT_BASE.contains(p)) {
            // System.out.println("Returning PlannedPathBase");
            return WaypointSafetyClassification.PlannedPathRobotBase;
        }
        if (Constants.PlannedPathZones.BELOW_CROSSBAR.contains(p)) {
            // System.out.println("Returing PlannedPathCrossBar");
            return WaypointSafetyClassification.PlannedPathCrossbar;
        }
        // System.out.println("Returning Safe");
        return WaypointSafetyClassification.Safe;
    }
}
