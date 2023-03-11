// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public double allianceCoordinateCenterY;

        private TargetingGrid(int targetIdRed, int targetIdBlue) {
            this.targetIdBlue = targetIdBlue;
            this.targetIdRed = targetIdRed;
            // TODO: add alliance coordinate center Y reference for grid
        }
    }

    public enum GridTargetingPosition {
        HighLeft(Constants.TowerConstants.scoreConeHigh),
        HighRight(Constants.TowerConstants.scoreConeHigh),
        HighCenter(Constants.TowerConstants.scoreCubeHigh),
        MidLeft(Constants.TowerConstants.scoreConeMid),
        MidRight(Constants.TowerConstants.scoreConeMid),
        MidCenter(Constants.TowerConstants.scoreCubeMid),
        LowLeft(Constants.TowerConstants.scoreFloor),
        LowRight(Constants.TowerConstants.scoreFloor),
        LowCenter(Constants.TowerConstants.scoreFloor);

        public Waypoint towerWaypoint;

        private GridTargetingPosition(Waypoint waypoint) {
            this.towerWaypoint = waypoint;
        }
    }

    public TargetingGrid currentTargetedGrid = TargetingGrid.GridDriverLeft;
    public GridTargetingPosition currentTargetPosition = GridTargetingPosition.MidCenter;

    public void setTargetPosition(GridTargetingPosition position) {
        currentTargetPosition = position;
    }
}
