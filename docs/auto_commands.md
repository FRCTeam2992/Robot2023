# Autonomous Commands

## Start Positions
  1. Leftmost
  2. Left center (column 4)
  3. Right center (column 6)
  4. Rightmost

## Auto selections
  1. Do nothing (all positions) -- Priority 1
  2. Preload score high (all positions) - Easy and prereq for others
  3. Preload & mobility (position 1 and 9) 
  4. Preload & balance (position 4 and 6) - Priority 1<or>
  5. Preload, cross for mobility & balance (position 4 and 6) - Priority 2<or>
  6. Preload, cross for mobility, intake 1 & balance (position 4 or 6) - Priority 3
  7. Preload, mobiltity and balance (position 1 or 9) -- priority 1 <or> 
  8. Preload, mobility, intake 1 (position 1 or 9) - Priority 2<or>
  8. Preload, mobility, intake and score cube high - Priority 2<or>
  9. Preload, mobility, intake 1 & balance - Priority 1


Concept:  Instead of a whole host of separate commands, have configurable start position separate from sequence and apply logic to make sure consistent

## From leftmost/rightmost end positions
  1. Select left(spot 1)/right(spot 9) position
  2. Select score level
    - None (likely only used if we have hw issue)
    - Mid -- do we ever use this?
    - High
  3. Move Sequence
    - None (only if we have issues likely)
    - Only move for mobility
    - Move & attempt 2 scores
    - Move, grab 1 & balance far side
    - Others?

## From Center left (spot 4) or center right (spot 6)
  1. Select center left(4) or center right(6)
  2. Select score level
    - None (likely only used if we have hw issue)
    - Mid -- do we ever use this?
    - High
  3. Move Sequence
    - None (only if we have issues likely)
    - Balance
    - Cross for mobility & balance
    - Cross, grab 1 piece & balance
    - Others?


# Initial start position
  ## Set odometry based on selected start position

# Initial scoring -- sequential
  ## Set scoring target as appropriate
  ## Move to scoring position CG -- use withTimeout?
  ## Open Claw -- with timeout or wait following
  ## Move to intake backstop -- withTImeout?
  
# Move sequences
  ## None -- just an empty command 

  ## Center balance -- just a path to follow
  ## Move for mobility -- just a path to follow


