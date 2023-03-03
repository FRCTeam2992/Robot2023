package frc.lib.manipulator;

import java.awt.geom.Rectangle2D;

public class NoFlyZone extends Rectangle2D.Double {
    public NoFlyZone(double minHeight, double minAngle, double maxHeight, double maxAngle) {
        super(minHeight, minAngle, maxHeight - minHeight, maxAngle - minAngle);
    }
}
