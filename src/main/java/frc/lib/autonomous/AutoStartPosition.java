package frc.lib.autonomous;

import frc.robot.Constants.ScoringGridConstants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public enum AutoStartPosition {
    // Path planner start poses MUST match these starting pose values!
    LoadStationEnd("Load Station End",
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Red.grid3CenterYMeters - ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Blue.grid6CenterYMeters + ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
            new PathPlannerTrajectory()),
    CenterLoadStationSide("Center Load Station Side",
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Red.grid2CenterYMeters - ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Blue.grid7CenterYMeters + ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
                    PathPlanner.loadPath("CenterWallSideInitialMove", new PathConstraints(.4, .4))),
    WallEnd("Wall End",
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Red.grid1CenterYMeters + ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Blue.grid8CenterYMeters - ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
            new PathPlannerTrajectory()),
    CenterWallSide("Center Wall Side",
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Red.grid2CenterYMeters + ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Blue.grid7CenterYMeters - ScoringGridConstants.conePoleOffsetYMeters,
                    Rotation2d.fromDegrees(0.0)),
            PathPlanner.loadPath("CenterWallSideInitialMove", new PathConstraints(.4, .4)));

    public String description;
    public Pose2d startPoseRed;
    public Pose2d startPoseBlue;
    public PathPlannerTrajectory initScoreTrajectory;

    private AutoStartPosition(String description, Pose2d poseRed, Pose2d poseBlue, PathPlannerTrajectory trajectory) {
        this.description = description;
        this.startPoseRed = poseRed;
        this.startPoseBlue = poseBlue;
        this.initScoreTrajectory = trajectory;
    }

    public Pose2d getStartPose() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return this.startPoseRed;
        } else {
            return this.startPoseBlue;
        }
    }

    public PathPlannerTrajectory getInitialTrajectory() {
        return this.initScoreTrajectory;
    }

}