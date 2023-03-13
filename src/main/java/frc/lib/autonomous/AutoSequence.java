package frc.lib.autonomous;

import java.util.Arrays;
import java.util.List;

public enum AutoSequence {
    Do_Nothing("Do Nothing",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.CenterLoadStationSide,
            AutoStartPosition.CenterWallSide,
            AutoStartPosition.WallEnd),
    SideMobilityOnly("Side Mobility Only",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.WallEnd),
    SideMobilityBalance("Side Mobility + Balance",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.WallEnd),
    SideMobilityIntake("Side Mobility + Grab Cube",
            AutoStartPosition.LoadStationEnd,
            AutoStartPosition.WallEnd),
    CenterBalance("Center Balance",
            AutoStartPosition.CenterLoadStationSide,
            AutoStartPosition.CenterWallSide);

    public String description;
    public List<AutoStartPosition> allowedStartPositions;

    private AutoSequence(String description, AutoStartPosition... allowedStartPositions) {
        this.description = description;
        this.allowedStartPositions = Arrays.asList(allowedStartPositions);
    }
}