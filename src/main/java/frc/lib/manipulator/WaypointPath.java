package frc.lib.manipulator;

import java.util.ArrayList;
import java.awt.geom.Line2D;

public class WaypointPath {
    private Waypoint _start;
    private Waypoint _end;
    private ArrayList<Waypoint> _path;
    private WaypointPathSegment _testSegment;

    public WaypointPath(Waypoint startPoint, Waypoint endPoint) {
        this._start = startPoint;
        this._end = endPoint;
        this._path = new ArrayList<Waypoint>();
        this._path.add(this._start);
        this._path.add(null);
        this._path.add(null);
        this._path.add(null);
        this._path.add(this._end);
        this._testSegment = new WaypointPathSegment(this._start, this._end);
    }

    public ArrayList<Waypoint> calculatePath() {
        if (endpointInNoFlyZone()) {
            throw new Error("No available path: endpoints in no fly zone");
        }
        int intersections = 0 | (
            pathSegmentIntersectsBoundary(
                Constants.Boundaries.BOUNDARY_Z2L,
                this._start, this._end)
        ) | (
            pathSegmentIntersectsBoundary(
                Constants.Boundaries.BOUNDARY_Z3L,
                this._start, this._end) << 1
        ) | (
            pathSegmentIntersectsBoundary(
                Constants.Boundaries.BOUNDARY_Z3L,
                this._start, this._end) << 2
        ) | (
            pathSegmentIntersectsBoundary(
                Constants.Boundaries.BOUNDARY_Z3L,
                this._start, this._end) << 3
        ) | (
            pathSegmentIntersectsBoundary(
                Constants.Boundaries.BOUNDARY_Z3L,
                this._start, this._end) << 4
        );

        switch(intersections) {
            case 0b00001:
                // B1 crossed
                break;
            case 0b00011:
                // B1 and B2 crossed
                break;
            case 0b00111:
                // B1, B2 and B3 crossed
                break;
            case 0b00110:
                // B2 and B3 crossed
                break;
            case 0b01010:
                // B2 and B4 crossed
                break;
            case 0b10010:
                // B2 and B5 crossed
                break;
            case 0b01000:
                // B4 crossed
                break;
            case 0b10000:
                // B5 crossed
                break;
            case 0b11000:
                // B4 and B5 crossed
                break;
        }
        
        if () {
            this._path.set(3, Constants.Waypoints.WAYPOINT_Z3TL);
        }

        return this._path;
    }

    private boolean endpointInNoFlyZone() {
        return (Constants.Zones.ZONE_1.contains(this._start) ||
                Constants.Zones.ZONE_1.contains(this._end) ||
                Constants.Zones.ZONE_2.contains(this._start) ||
                Constants.Zones.ZONE_2.contains(this._end) ||
                Constants.Zones.ZONE_3.contains(this._start) ||
                Constants.Zones.ZONE_3.contains(this._end) ||
                Constants.Zones.ZONE_4.contains(this._start) ||
                Constants.Zones.ZONE_4.contains(this._end));
    }

    private int pathSegmentIntersectsBoundary(NoFlyBoundary boundary, Waypoint p1, Waypoint p2) {
        this._testSegment.setWaypoints(p1, p2);
        return boundary.intersects(this._testSegment) ? 1 : 0;
    }
}
