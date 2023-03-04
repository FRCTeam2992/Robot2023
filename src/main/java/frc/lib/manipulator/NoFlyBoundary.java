package frc.lib.manipulator;

import java.awt.geom.Line2D;

public class NoFlyBoundary extends Line2D.Double {
    public static enum NoFlyBoundaryType {
        AT_ANGLE,
        AT_HEIGHT
    }

    public NoFlyBoundary(NoFlyBoundaryType btype, double min, double max, double at) {
        super(min, at, max, at);
        if (btype == NoFlyBoundaryType.AT_HEIGHT) {
            this.x1 = at;
            this.x2 = at;
            this.y1 = min;
            this.y2 = max;
        }
    }

    public boolean intersects(WaypointPathSegment segment) {
        return super.intersectsLine((Line2D.Double) segment);
    }
}
