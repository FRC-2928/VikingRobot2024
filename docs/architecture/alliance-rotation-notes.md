# Alliance Rotation & Operator Perspective — Reference Notes

This document captures the analysis of how CTRE's swerve API handles alliance-dependent
rotation, what the 2026 robot did (and why it's confusing), and what our clean approach
should be when we layer in target-heading modes.

**Status:** Tabled for now. Revisit when implementing aim-at-target drive modes.

---

## Coordinate System Basics

- WPILib uses **blue-origin coordinates**: +X toward red wall, 0° heading = facing red wall.
- `getPose()` always returns blue-origin values regardless of alliance.
- A blue robot facing its speaker: heading ≈ 180°. A red robot facing its speaker: heading ≈ 0°.

## CTRE `ForwardPerspectiveValue` Modes

| Mode | Translation | Target Direction | Use Case |
|------|-------------|-----------------|----------|
| `OperatorPerspective` | Rotated by operator forward direction (180° for red) | Also rotated by operator forward | Teleop — "push forward = away from driver" |
| `BlueAlliance` | No rotation applied; input must be in blue-origin | No rotation applied | Path following, field-absolute targets |

Set once via `setOperatorPerspectiveForward()`. All `FieldCentric`, `ApplyFieldSpeeds`, and
`FieldCentricFacingAngle` requests respect this.

## The Three Cases

### Case 1: Translation (Joystick → Field Speeds)

No manual alliance flip needed. Pass raw joystick-scaled values. The API handles it via
operator perspective. This is what `FieldCentric` was designed for.

### Case 2: Rotation Rate (Joystick → Angular Velocity)

Alliance-independent. Turning left is turning left. No flip anywhere.

### Case 3: Target Heading (Face a Field-Absolute Target)

This is where complexity lives. The speaker/hub is at a fixed field position.
"Face the speaker" is a different blue-origin angle per alliance.

The API (in `OperatorPerspective` mode) also rotates `withTargetDirection()` by the
operator perspective. So if you pass a driver-relative angle (0° = "face away from me"),
it works for symmetric targets. But for field-absolute geometry (atan2 to a coordinate),
you're computing in blue-origin — and the API will rotate your answer again.

**Options when we get here:**
1. Use `ForwardPerspective = BlueAlliance` on the heading request. Pass blue-origin
   angles directly. Translation inputs must also be blue-origin in this mode.
2. Keep `OperatorPerspective` and pre-subtract the operator perspective from computed
   angles (undo what the API will add).
3. Use separate request instances: one `FieldCentric` (OperatorPerspective) for
   translation-only modes, one `FieldCentricFacingAngle` (BlueAlliance) for aim modes.

## What 2026 Did (For Reference — Don't Copy)

### The Double-Flip Bug

`calculateSpeedsBasedOnJoystickInputs()` manually negated vx/vy for red alliance.
`ApplyFieldSpeeds` (in OperatorPerspective mode) ALSO rotates by 180° for red.
Net: 360° rotation = identity. It "works" because two flips cancel.

### The Un-Flip in INTAKE_DRIVE

`applyIntakeDrive()` uses `FieldCentricFacingAngle`. Since `calculateSpeeds` already
flipped, and the API would flip again, the code un-flips back to raw values:
```java
boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
double blueVx = isRed ? -vx : vx;  // un-negate what calculateSpeeds negated
double blueVy = isRed ? -vy : vy;
```

### Confusing Helper Functions

| Function | Actual behavior | Expected behavior (from name) |
|----------|----------------|-------------------------------|
| `applyAllianceRotation(angle)` | No-op for RED, adds π for BLUE | "Apply" suggests adding the alliance offset — but doesn't for red |
| `invertAllianceRotation(angle)` | Adds π for RED, no-op for BLUE | "Invert" suggests removing — but it's adding for red |

The root problem: no documented frame contracts. Functions evolved through trial-and-error
without specifying what frame their inputs/outputs use.

## Our Clean Approach (When We Get There)

1. Establish frame contracts: every function that computes or accepts an angle must
   document whether it's blue-origin, driver-relative, or robot-relative.
2. Don't manually flip joystick translation — let `OperatorPerspective` handle it.
3. For field-absolute targets, use `BlueAlliance` perspective on the heading request
   (option 3 above: separate request instances per mode).
4. All geometry (atan2 to game elements) stays in blue-origin. Alliance only affects
   which game element coordinates you target, NOT the frame of the result.
