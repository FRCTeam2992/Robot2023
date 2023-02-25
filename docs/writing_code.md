# Writing Robot Code

This doc provides some examples that may be useful for people getting started writing robot code with WPIlib.

## Subsystems

A subsystem class looks like this. Note that it inherits from (`extends`) `SubsystemBase`.

```{java}
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
```

### Adding methods to read state of the subsystem

You may also want a method to query the state of a particular component, for example, a limit switch (which would use the class `DigitalOutput`). To create that, you can add the following type of method to the subsystem class.

```{java}
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
```

### Adding methods that generate commands for the subsystem

In a subsystem, you may want to add a method that creates a Command using that subsystem. To do that, you can add a method like this.

```{java}
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
```

Note that you'll need to add the following import line at the top of the file.

```{java}
import edu.wpi.first.wpilibj2.command.CommandBase;
```

## Commands

Once you have a subsystem, you will want to create `Command` classes that make use of it to accomplish robot tasks. To create a `Command` for `ExampleSubsystem` defined above, you would create a class that looks like this.

```{java}
package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  private final ExampleSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
```
