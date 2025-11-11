# Simple Rotary Example

Example of a subsystem that has has linear extention based on some shaft rotation, in this case a sprocket. The mechanism extends a set amount of distance per sprocket rotation.

# Values to Tune

## CAN ID
Self explanatory, just the CAN ID of the motor.

## Inverted Value
Value should be set to make the polarity make sense, whichever direction "positive distance" makes sense to be in. Elevators should be positive when raised

## Current Limits
Current limit should proably be around 40ish.

## Gear Ratio
The Gear Ratio value is super critical here, make sure its right and check encoder outputs before running anything. Keep this as a formula for the ratio made up of all of the individual reductions this makes it much easier to double check that its right. Be careful to not do integer division on these values. Make sure atleast one of the values being divided is a double by adding a "d" to it or adding .0 to the value. If elevator uses cascade rigging make sure stage count is in the conversion factor.

## Motion Magic Values
Motion Velocity and Accel Constraints to "smoothen" motion. Start lower and tune from there. Make use the logged values to make sure you are actually reaching the desired constraints before increasing them, you can have good values and a bad tune.

## Motor
Only used for sim, but easy to set so do it.

## Setpoints
Setpoints are kept in an enum so you can control the subsystem in plain english. Just add the setpoint values you use to the enum with proper naming.
