package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A regular octagon with the robot's nose always pointed along its direction
 * of travel, like a car rather than a crab.
 *
 * HOW THE HEADING WORKS
 * ---------------------
 * setTangentHeadingInterpolation() tells Pedro to aim the robot down the
 * tangent of whatever path it's on. On a straight leg the tangent is constant,
 * so the robot faces along that leg for its whole length, then swings 45
 * degrees at each corner into the next one. Eight legs, eight 45 degree turns,
 * back to where it started facing the way it started.
 *
 * You never specify a single heading anywhere in this file. Pedro derives all
 * eight from the geometry.
 *
 * WHY ONE PATHCHAIN AND NOT EIGHT followPath CALLS
 * ------------------------------------------------
 * All eight legs go into a single PathChain, so Pedro flows from one to the
 * next without stopping and without waiting on isBusy between them. Eight
 * separate followPath calls would make the robot come to a halt at every
 * corner. One chain makes it one continuous lap.
 *
 * GEOMETRY
 * --------
 * Side length 18 in, which gives a 43.5 x 43.5 in footprint, about 3.6 ft
 * square. Starting at (0,0) the octagon runs from x = -12.7 to x = +30.7 and
 * y = 0 to y = 43.5, so give it roughly a 5 x 5 ft clear area with the robot
 * in the bottom-middle-right of it. Change SIDE to resize; the vertices are
 * computed from it.
 *
 * WHAT TO WATCH
 *   The nose should lead the whole way round, never crabbing sideways.
 *   Corners should be crisp 45 degree swings, not lazy drifts.
 *   It should close the loop and end facing its original direction.
 *   x, y and heading should all return near 0.
 */
@Autonomous(name = "Octagon (nose forward)", group = "Tuning")
public class OctagonTest extends OpMode {

    /** How many laps before it stops. Set higher to watch it repeat. */
    private static final int LAPS = 1;

    /**
     * Rotate back to the starting heading once the lap closes.
     *
     * The last leg travels at 315 degrees, so nose-forward means the robot
     * arrives home pointing 315, not 0. You cannot have both on that leg: the
     * direction of travel IS 315. So all eight legs stay honestly nose-forward
     * and then the robot squares itself up in place at the end.
     *
     * Set false if you would rather it stop facing its direction of travel.
     */
    private static final boolean SQUARE_UP_AT_END = true;

    /** Heading to finish on, degrees. Matches the starting heading. */
    private static final double FINISH_HEADING_DEG = 0;

    private Follower follower;
    private PathChain octagon;

    private int lapsDone;
    private boolean running;
    private int state;

    // Regular octagon walked from the origin: each leg is SIDE long and turns
    // 45 degrees counterclockwise from the last. Closes exactly on (0,0).
    private final Pose v0 = new Pose(  0.000,  0.000);
    private final Pose v1 = new Pose( 18.000,  0.000);
    private final Pose v2 = new Pose( 30.728, 12.728);
    private final Pose v3 = new Pose( 30.728, 30.728);
    private final Pose v4 = new Pose( 18.000, 43.456);
    private final Pose v5 = new Pose(  0.000, 43.456);
    private final Pose v6 = new Pose(-12.728, 30.728);
    private final Pose v7 = new Pose(-12.728, 12.728);

    private void buildOctagon() {
        octagon = follower.pathBuilder()
                .addPath(new BezierLine(v0, v1)).setTangentHeadingInterpolation()
                .addPath(new BezierLine(v1, v2)).setTangentHeadingInterpolation()
                .addPath(new BezierLine(v2, v3)).setTangentHeadingInterpolation()
                .addPath(new BezierLine(v3, v4)).setTangentHeadingInterpolation()
                .addPath(new BezierLine(v4, v5)).setTangentHeadingInterpolation()
                .addPath(new BezierLine(v5, v6)).setTangentHeadingInterpolation()
                .addPath(new BezierLine(v6, v7)).setTangentHeadingInterpolation()
                .addPath(new BezierLine(v7, v0)).setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        buildOctagon();
        telemetry.addLine("Needs about 5 x 5 ft clear.");
        telemetry.addLine("Robot starts at a corner, drives +X first.");
        telemetry.update();
    }

    @Override
    public void start() {
        lapsDone = 0;
        running = true;
        state = 0;
        follower.followPath(octagon);
    }

    @Override
    public void loop() {
        follower.update();

        if (running && !follower.isBusy()) {
            switch (state) {
                case 0:
                    // Lap finished. The robot is home but pointing 315 degrees.
                    if (SQUARE_UP_AT_END) {
                        // turnTo takes the shortest route, and 315 -> 0 is a
                        // 45 degree swing, so this is one short counterclockwise
                        // nudge rather than going the long way round.
                        follower.turnTo(Math.toRadians(FINISH_HEADING_DEG));
                        state = 1;
                    } else {
                        state = 1;
                    }
                    break;

                case 1:
                    // Squared up (or skipped). Count the lap and decide.
                    lapsDone++;
                    if (lapsDone < LAPS) {
                        follower.followPath(octagon);
                        state = 0;
                    } else {
                        running = false;
                    }
                    break;
            }
        }

        Pose pose = follower.getPose();
        telemetry.addData("laps", "%d / %d", lapsDone, LAPS);
        telemetry.addData("running", running);
        telemetry.addData("phase", state == 0 ? "driving octagon" : "squaring up");
        telemetry.addLine();
        telemetry.addData("x", "%.2f", pose.getX());
        telemetry.addData("y", "%.2f", pose.getY());
        telemetry.addData("heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addLine();
        telemetry.addLine("--- after a full lap these should be near 0 ---");
        telemetry.addData("closure error (in)", "%.2f", Math.hypot(pose.getX(), pose.getY()));
        telemetry.addData("heading error (deg)", "%.2f",
                Math.toDegrees(pose.getHeading()) - FINISH_HEADING_DEG);
        telemetry.update();
    }
}
