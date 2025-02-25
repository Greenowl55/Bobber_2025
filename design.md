# Code Design

## Subsystems
- Drivetrain
- Elevator
- Coral
- Algae
- FishHookTilt

Subsystems are groupings where each command is exclusive with the others: Coral + Algae + Tilt cannot be a Subsystem because you may want to command those independently of each other, and the CommandScheduler will not allow that. 

## Commands

Commands have three phases: `initialize()`, `execute()`, and `end(boolean)`.

- `initialize()` - runs once when scheduled
- ` execute()` - runs every tick until the command is done
- `end(boolean)` - runs when the command finishes or is interrupted, if it's interrupted it's called as `end(true)`
- `isFinished()` - is run to see if your command is done.  It defaults to `true` if not overridden.

### Algae

- RollIn
- RollOut

#### Open questions

- Do we need multiple speeds?

### Coral

- Eject
- Intake

### Elevator

- Up
- Down
- SetPoint

#### Open Questions

- Can we reuse one `SetPoint` command for all the set points we want, or do they each have to be their own command?

### FishHookTilt

- AngleUp
- AngleDown
- SetPoint

#### Open Questions

- Can we reuse one `SetPoint` command for all the set points we want, or do they each have to be their own command?


### Drivetrain

This is basically already provided for us.

