package frc.lib.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum AutoStartPosition {
    Inner_Most("InnerMost",
            new Pose2d(1.89, 4.983, Rotation2d.fromDegrees(0.0))),
    Center_Inner("Center Inner",
            new Pose2d(1.89, 3.307, Rotation2d.fromDegrees(0.0))),
    Center_Outer("Center Outer",
            new Pose2d(1.89, 2.189, Rotation2d.fromDegrees(0.0))),
    Outer_Most("OuterMost",
            new Pose2d(1.89, 0.513, Rotation2d.fromDegrees(0.0)));

    public String description;
    public Pose2d startPose;

    private AutoStartPosition(String description, Pose2d pose) {
        this.description = description;
        this.startPose = pose;
    }

}