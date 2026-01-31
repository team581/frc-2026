# Subsystem bringup checklist

Checklist to complete before bringing up a new subsystem on the robot.

## Code preparation

### Motor configuration

- [ ] All voltage outputs set to 0
- [ ] All PID gains set to 0
- [ ] Position mechanisms using motion magic should use `PositionVoltage` instead of `MotionMagicVoltage`
  - Allows proper PID tuning without motion profiling interfering
- [ ] Current limits configured
- [ ] Setpoints set to reasonable/safe values
  - Position mechanisms: setpoints within expected range of motion
  - Flywheels: minimum velocity of 0

### Logging

- [ ] Log values needed for tuning
  - Position mechanisms: position, applied voltage, stator current
  - Rollers: applied voltage, stator current
  - Flywheels: velocity, applied voltage, stator current
- [ ] Use `GlobalConfig.IS_DEVELOPMENT` for verbose logs if needed

### Testing plan

- [ ] Identify which `RobotManager` states will be used to test
  - [ ] Determine if custom bringup-only states and/or button bindings are needed
- [ ] Plan state transitions to test

## On-robot verification

### Before enabling

- [ ] Verify motor directions by moving mechanism by hand and observing logged position/velocity
  - Rollers: positive voltage for intaking, negative voltage for outtaking
  - Flywheels: positive voltage for shooting
  - Position mechanism: positive voltage for upwards/clockwise motion
    - A turret would have positive voltage for moving clockwise
    - An elevator would have positive voltage for moving upwards
- [ ] Confirm gearing ratios match reality
  - Move the mechanism by hand and observe if the logged position matches how much you moved it
- [ ] Verify external sensors (ex. Banner sensors) aren't inverted and are tuned for the correct range
