# Subsystems & Controls

[Back to Project Documentation](./)

## Drive train (on CANivore)

Swerves each have(4 sets of):

- 1 Turn motor (Falcon 500 + Talon FX)
- 1 Drive motor (Falcon 500 + Talon FX)
- 1 absolute position encoder (CANCoder)

## Butterfly wheels

- 1 pneumatic solenoid controlling 2 pistons for release
- one-way switch, can't put them back(spring loaded)
- need some protection against deploying before last 30s of match

## Bumper eater (buddy drive/balance)

- motor to spin velcro flaps (VictorSPX controller, BAG motor)
- pneumatic solenoid controlling piston to extend(may be spring loaded or one way extension)

## Intake

- 2x motors to spin wheels (NEO 550 motor, SPARK MAX encoder)
- TBD maybe use "follower mode" on one, and only need to control the other or one motor per each roller
- Solenoid (2 pistons)controlling deployed state

## Spindexer

- 1 BAG motor, Victor SPX encoder
- possibly detection electronics (limit switch? on the springed finger)
- possibly Arduino color sensor assembly (bc of I2C port bug)

## Elevator

- 1 pneumatic solenoid (2 pistons) for extending elevator
- 2x motors to raise (Falcon 500 motor, Talon FX encoder)
- one motor in "follower mode" ganged together, with the following motor in reverse
- limit switch at the bottom, when hits, reset encoder position to zero (can't do every cycle -- slow CAN bus operation; need to debounce)
- subsystem API needs:
  - "Home Elevator" control, in case we lose encoder count (with reasonable speed), (may be manual or automatic)
  - move to certain position (distance from Home) via "motion magic" (see 2019 code)
    - may require 2 PID/MotionMagic profiles
  - hold position (use encoder to provide resistance against falling maybe until timeout)
  - state machine that controls when arm can swing, when elevator can move
    - arm can't swing until elevator is up
    - elevator can't come down until arm is swung out of the way

## Arm

- pivot motor (Falcon 500 motor, Talon FX encoder + fixed absolute position encoder)
- can also use "motion magic" for the movements, may need some FF terms that will be variable across the arc
- state control based on elevator position (see above)

## Claw

- 1 solenoid to control 2 pistons
- Spring-loaded open, solenoid triggered to close

| Sol. | Claw State |
|------|------------|
|    0 | opened     |
|    1 | closed     |
