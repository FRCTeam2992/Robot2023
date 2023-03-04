package frc.lib.manipulator;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

public class WaypointPathSegment extends Line2D.Double {
    public WaypointPathSegment(Waypoint p1, Waypoint p2) {
        super((Point2D.Double) p1, (Point2D.Double) p2);
    }

    public void setWaypoints(Waypoint p1, Waypoint p2) {
        this.x1 = p1.x;
        this.y1 = p1.y;
        this.x2 = p2.x;
        this.y2 = p2.y;
    }
}
