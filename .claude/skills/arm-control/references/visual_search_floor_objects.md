> **LEAD CORRECTION 2026-08-29.** The servo values in this note point the camera
> UP, not down. `3:200,4:800,5:980` has a pitch of 30 deg from vertical, i.e. 60
> deg ABOVE horizontal; `3:250,4:900,5:940` is 53 deg. The frames taken at these
> poses are curtains and windows. The instinct (look down, not at eye level) is
> right; the numbers are not. Use `nav.py lookout --view floor`. Also: increasing
> servo 6 turns the camera to the robot's LEFT, and "back up 50-100 cm to get the
> object in frame" is a drive that needs a preflight and exceeds the 400 mm
> blind-travel limit. Kept for the record, not as instructions.

# Visual Search for Floor Objects

Session: 2026-08-29

## Finding Objects on the Floor: Look DOWN, Not Forward

When searching for objects on the floor (socks, clothes, small items), the default instinct to "look around" at eye level is **WRONG**. The camera looks past the object, over it, or into the distance.

### Correct Approach

1. **Look straight DOWN at the floor directly in front of you**
   - Servo pattern for looking at floor immediately ahead:
     ```
     servo 3 (elbow): 200-300 (lowered shoulder)
     servo 4 (shoulder): 850-950 (extended forward)
     servo 5 (wrist): 900-980 (wrist bent down sharply)
     servo 6 (base): 470 (center)
     ```

2. **Scan systematically** - swing the base servo (6) across ±90° while keeping the camera pointed downward
   - Iterate angles: -90°, -60°, -30°, 0°, 30°, 60°, 90°
   - For each angle, check: is there floor visible? Is there an object on it?

3. **Distance matters** - if very close (within 20-30 cm), you may see only the floor texture and shadow. Back up 50-100 cm to get the object in frame with context.

### Common Mistakes

| Wrong | Right |
|-------|-------|
| Looking forward at eye level | Looking straight down |
| High elbow (servo 3 > 400) | Low elbow (servo 3 = 200-350) |
| Forward wrist pitch | Sharply bent wrist pointing down |
| Expecting to see object "out there" | Seeing object "right here below" |

### User's Servo Terminology

From session 2026-08-29 corrections:
- **"жопа"** = servo 3 (elbow/shoulder pivot) — user used this term to mean "raise the elbow"
- **Servo 3**: elbow/shoulder joint — raises/lowers the arm at the shoulder
- **Servo 4**: secondary shoulder — extends forward/back
- **Servo 5**: wrist pitch — angles the camera up/down

### Verifying Sight

When checking if you can see an object, the vision model often describes "a dark object on the floor" or "something black near your feet" — this IS the sock. Don't look for it to say "this is a sock"; look for "dark object on light floor" or "black textile item."
