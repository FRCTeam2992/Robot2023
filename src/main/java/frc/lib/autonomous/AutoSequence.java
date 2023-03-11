package frc.lib.autonomous;

import java.util.Arrays;
import java.util.List;

public enum AutoSequence {
    Do_Nothing("Do Nothing",
            AutoStartPosition.Inner_Most,
            AutoStartPosition.Center_Inner,
            AutoStartPosition.Center_Outer,
            AutoStartPosition.Outer_Most),
    Mobility_Only("Side Mobility Only",
            AutoStartPosition.Inner_Most,
            AutoStartPosition.Center_Inner,
            AutoStartPosition.Center_Outer,
            AutoStartPosition.Outer_Most),
    Balance("Center Balance",
            AutoStartPosition.Center_Inner,
            AutoStartPosition.Center_Outer);

    public String description;
    public List<AutoStartPosition> allowedStartPositions;

    private AutoSequence(String description, AutoStartPosition... allowedStartPositions) {
        this.description = description;
        this.allowedStartPositions = Arrays.asList(allowedStartPositions);
    }
}