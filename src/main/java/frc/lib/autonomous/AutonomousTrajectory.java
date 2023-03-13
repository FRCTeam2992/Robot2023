package frc.lib.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public enum AutonomousTrajectory {
    LoadStationMobility(PathPlanner.loadPath("LoadStationMobility", new PathConstraints(.5, .5))),
    WallMobility(PathPlanner.loadPath("WallMobility", new PathConstraints(.5, .5))),
    LoadStationMobilityIntake(PathPlanner.loadPath("LoadStationMobilityIntake", 4.0, 3.0)),
    WallMobilityIntake(PathPlanner.loadPath("WallMobilityIntake", 4.0, 3.0)),
    LoadStationMobilityBalance(PathPlanner.loadPath("LoadStationMobilityBalance", new PathConstraints(2.8, 2.25))),
    WallMobilityBalance(PathPlanner.loadPath("WallMobilityBalance", new PathConstraints(2.8, 2.25))),
    LoadStation2Scores(PathPlanner.loadPath("LoadStation2Scores", 4.0, 3.0)),
    Wall2Scores(PathPlanner.loadPath("Wall2Scores", 4.0, 3.0)),
    CenterBalanceLoadStationSide(PathPlanner.loadPath("CenterBalanceLoadStationSide", new PathConstraints(2.8, 2.25))),
    CenterBalanceWallSide(PathPlanner.loadPath("CenterBalanceWallSide", new PathConstraints(2.8, 2.25)));

    public PathPlannerTrajectory trajectory;

    private AutonomousTrajectory(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }
}
