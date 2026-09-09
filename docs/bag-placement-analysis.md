# Putting an object into the yellow bag — why two runs failed, and what to change

Written 2026-09-09 after two attempts that both ended with the bar ON the bag
rather than IN it. Evidence in `nav_state/frames/bag_analysis_20260909/`,
including two third-person photographs from the user, which carried more
information than anything the wrist camera produced all evening.

## What actually happened

Run 1: released 7 cm above the bag's bottom — a drop, not a place.
Run 2: descended to contact and released — but the contact was the bag's *outer
wall*, so the bar landed across the rim.

Both were reported by me as probably-successful before checking, and both were
caught by looking afterwards. The verification that finally worked was a
top-down view with the bar surrounded by yellow; the checks that failed were
`yellow fills 39% of frame`, `bag centroid onto the closing point`, and
`contact height reached the floor`.

## The four reasons, in order of how much they cost

### 1. Every colour measure I used answers the wrong question

"Yellow fills X% of the frame" and "yellow under the closing point" both say
THE BAG IS IN VIEW. Neither can say THE OPENING IS UNDER THE GRIPPER, because
from beside a bag this tall the wrist camera never sees the opening at all —
every frame is the outer wall. I tried three variants of the same mistake and
read each as progress.

The centroid variant fails for an additional reason: the bag is larger than the
frame at working distance, so its blob is clipped on every edge and its centroid
is the centroid of the clipping. One such reading asked for a 138 mm radius
correction that would have driven the hand inside `rig.R_MIN_CHASSIS`.

### 2. Contact height cannot tell inside from beside

The floor inside a thin plastic bag and the floor next to it are the same
height. `place_until_contact` reaching z ≈ GRASP_Z therefore proves the arm went
down, not that it went down *into* anything. It is still the right way to place —
it is just not a check on WHERE.

### 3. The reach envelope, if the bag really is rigid

With the floor at GRASP_Z and a 200 mm rim, the rim top is z = +127, and
`kin.max_reach` collapses there:

| z | max reach | usable window above `R_MIN_CHASSIS` = 140 |
|---|---|---|
| +100 | 201 mm | 140..201 |
| +120 | 171 mm | 140..171 |
| +127 | 145 mm | **140..145 — five millimetres** |
| +140 | 0 | none |

Clearing a rigid 20 cm rim leaves a 5 mm corridor. No odometry-free chassis
holds that.

### 4. But the bag is NOT rigid, and the photographs show it

This is the part I got wrong in my own analysis. In both third-person frames the
bag is a soft carrier bag that has **slumped**: the walls fold inward and the
mouth sits well below its nominal 200 mm. So the table above is the worst case,
not the situation. The real obstacle was never the rim height — it was that I
could not SEE the mouth to aim at it.

## What to change for the next run

**Aim at the black handles, not the yellow body.** The bag has two black
handles that stand up and bracket the opening. They are high-contrast against
both the yellow bag and the pale tile, they sit at the mouth's edge by
construction, and the midpoint between them IS the centre of the opening. The
yellow mask ignores them completely. This is the single biggest change
available: it converts "is the bag under me" into "is the OPENING under me",
which is the question that actually matters.

**Approach head-on, not from the side.** In both photographs the arm is reaching
roughly perpendicular to the chassis. The arm's best-tested and widest envelope
is straight ahead (base 470). Driving so the bag is dead ahead at R ≈ 180-200
puts it in the strong part of the envelope instead of the edge.

**Clear the cables off the floor first.** Both photographs show black cable
looped around the chassis and lying beside the wheels. The wheels jammed once
this evening — the user freed them — and I then built a whole false theory about
non-linear drive response out of the measurements that jam produced. The cables
are the most likely cause and they are trivially removable.

**Do not drive onto the door threshold rail.** The second photograph has the
chassis up against the metal track. Crossing it tilts the robot, which moves
GRASP_Z and the arm's sag — the two numbers everything else is calibrated
against.

**Verify from above BEFORE releasing, not after.** The check that works is: bar
surrounded by yellow on all sides, seen from above the mouth. Make it a
precondition of opening the jaws, the way the contact height already is.

## Numbers confirmed this session

- Close **5 mm lower** than the aim suggests (`rig.GRASP_Z_CLOSE_OFFSET`). Four
  grasps stalled at the empty-jaw value with a perfect lateral aim; the user's
  5 mm caught it first try. A pixel aim cannot see this axis.
- Base gain **1.84 px/unit** on the Aurora, against 4.9 recorded for the old
  camera.
- Radius→pixel gain is **not constant between poses**: 0.89 px/mm measured at one
  pose, ~2 px/mm at another, and using the stale value threw the target out of
  frame. Measure it where you will use it.
- Increasing R moves the blob DOWN the frame. I had this sign inverted once and
  it walked the hand into the chassis limit.
