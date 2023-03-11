// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.element.ModuleElement.DirectiveKind;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.manipulator.Waypoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class RobotState {
    public enum EndgameModeState {
        InEndgame,
        NotInEndgame
    }

    public EndgameModeState endgameMode = EndgameModeState.NotInEndgame;
    
    public boolean isInEndgameMode() {
        return endgameMode == EndgameModeState.InEndgame;
    }

    public enum TargetingGrid {
        GridLoadingOuter(3, 6),
        GridCoopertition(2, 7),
        GridWallOuter(1, 8),
        GridCenter(2, 7),
        GridDriverLeft(1, 6),
        GridDriverRight(3, 8);

        public int targetIdBlue;
        public int targetIdRed;

        private TargetingGrid(int targetIdRed, int targetIdBlue) {
            this.targetIdBlue = targetIdBlue;
            this.targetIdRed = targetIdRed;
        }

        public int getTargetId() {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                return this.targetIdRed;
            }
            return this.targetIdBlue;
        }

        public double getGridCenterYMeters() {
            switch (this.getTargetId()) {
                case 1:
                    return Constants.ScoringGridConstants.Red.grid1CenterYMeters;
                case 2:
                    return Constants.ScoringGridConstants.Red.grid2CenterYMeters;
                case 3:
                    return Constants.ScoringGridConstants.Red.grid3CenterYMeters;
                case 6:
                    return Constants.ScoringGridConstants.Blue.grid6CenterYMeters;
                case 7:
                    return Constants.ScoringGridConstants.Blue.grid7CenterYMeters;
                case 8:
                    return Constants.ScoringGridConstants.Blue.grid8CenterYMeters;
                default:
                    // NOTE: This case cannot happen due to the enum definition.
                    return 0.0;                    
            }
        }
    }

    public enum GridTargetingPosition {
        HighLeft(Constants.TowerConstants.scoreConeHigh, -Constants.ScoringGridConstants.conePoleOffsetYMeters),
        HighRight(Constants.TowerConstants.scoreConeHigh, Constants.ScoringGridConstants.conePoleOffsetYMeters),
        HighCenter(Constants.TowerConstants.scoreCubeHigh, 0.0),
        MidLeft(Constants.TowerConstants.scoreConeMid, -Constants.ScoringGridConstants.conePoleOffsetYMeters),
        MidRight(Constants.TowerConstants.scoreConeMid, Constants.ScoringGridConstants.conePoleOffsetYMeters),
        MidCenter(Constants.TowerConstants.scoreCubeMid, 0.0),
        LowLeft(Constants.TowerConstants.scoreFloor, -Constants.ScoringGridConstants.conePoleOffsetYMeters),
        LowRight(Constants.TowerConstants.scoreFloor, Constants.ScoringGridConstants.conePoleOffsetYMeters),
        LowCenter(Constants.TowerConstants.scoreFloor, 0.0);

        public Waypoint towerWaypoint;
        public double lateralScoringOffsetMeters;

        private GridTargetingPosition(Waypoint waypoint, double yOffset) {
            this.towerWaypoint = waypoint;
            this.lateralScoringOffsetMeters = yOffset;
        }
    }

    public TargetingGrid currentTargetedGrid = TargetingGrid.GridDriverLeft;
    public GridTargetingPosition currentTargetPosition = GridTargetingPosition.MidCenter;
}
