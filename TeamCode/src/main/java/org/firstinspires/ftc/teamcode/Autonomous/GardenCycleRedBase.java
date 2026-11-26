package org.firstinspires.ftc.teamcode.Autonomous;

// Preload and garden pickup like the three cycle, then back to the first shoot pose and shoot once our CELL there
// is up, pick up the extra flower on the park wall, shoot again from the same spot, then park. Poses are in ThreeCycleRedBase
public abstract class GardenCycleRedBase extends ThreeCycleRedBase {
    // Parks at 26 s instead of the three cycle deadline
    protected static final double GARDEN_PARK_DEADLINE = 26.0;

    // Always back to the first shoot pose, the volley there waits for our CELL to be up
    @Override
    protected PathState afterGardenIntake() {
        if (hive != null) {
            hive.check();
        }
        return PathState.RETURN_TO_SHOOT1;
    }

    @Override protected PathState afterExtraPickup() { return PathState.RETURN_TO_SHOOT1; }
    @Override protected double getParkDeadline() { return GARDEN_PARK_DEADLINE; }
}
