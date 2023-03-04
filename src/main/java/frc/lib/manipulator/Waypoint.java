package frc.lib.manipulator;

import java.awt.geom.Point2D;

public class Waypoint extends Point2D.Double {
    public Waypoint(double height, double angle) {
        super(height, angle);
    }

    public double height() {
        return this.x;
    }

    public double angle() {
        return this.y;
    }
}
