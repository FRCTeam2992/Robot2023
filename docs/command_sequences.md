# Command Sequences

## Top-level Command Sequences

These are the top-level human- or auto-triggered command sequences needed.

### Intaking Game Pieces

- Intake a cube or cone from the floor
  1. `BackstopClaw`
  2. Deploy Intake outside bot perimeter
  3. `SpinIntake(+power)`
  4. `AlignGamePiece(cw)`
- Intake a cube or cone from the loading shelf/ramp
  1. `BackstopClaw`
  2. `SpinIntake(-power)`
  3. `AlignGamePiece(cw)`
- Grab a cube or cone from the Spindexer (assumptions: piece detected aligned (debounced limit switch depressed for duration?) and `AlignGamePiece(cw)` active)
  1. Ensure Claw position above Spindexer
  2. Move Claw over game piece
  3. Close Claw on game piece
  4. Stop Spindexer spinning

### Scoring Game Pieces

- Drive to aligned position for scoring (?)
- `Score(floor)`: Score a cube or cone on the floor
- `Score(mid_cube)`: Score a cube on the mid-level shelf
- `Score(mid_cone)`: Score a cone on the mid-level pole
- `Score(high_cube)`: Score a cube on the high-level shelf
- `Score(high_cone)`: Score a cone on the high-level pole
- `ReleaseGamePiece`: Open Claw to release piece
- Panic stop and return home

### Parking on Charging Station

- X Drive Wheels
- Deploy Butterfly Wheels (Endgame only)

### Endgame: Buddy Driving with Alliance Partner

- Deploy Bumper Eater and set to eating/spinning
- Stop Bumper Eater eating/spinning (maybe just held button for spinning is released)

## Subcommand Sequences

These are command sequences that are used by other command sequences

- `Score(scoreType)`: Score on the `scoreType` game piece location
  1. `CarriageUp`
  2. `DeployElevator`
  3. `MoveToSetPoint(scoreType)`
- `CarriageUp`: Move carriage up above threshold for moving arm
- `DeployElevator`: Extend the Elevator assembly outside the bot perimeter
- `MoveToSetPoint(scoreType)`:
  1. Move Carriage to set point for `scoreType`
  2. Move Arm to set point for `scoreType`
- `BackstopClaw`: Ensure Claw is in position to backstop piece intake
- `SpinIntake(power)`: Spin the Intake wheels with the given `power` (negative `power` for reverse)
- `AlignGamePiece(direction)`: Spin the Spindexer in the given direction
