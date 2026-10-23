package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * One continuous 360 degree rotation in place.
 *
 * WHY THIS DOESN'T USE follower.turn()
 * ------------------------------------
 * Every built-in way of asking Pedro to rotate tops out at 180 degrees:
 *
 *   follower.turn(2*PI) sets the target to (currentHeading + 2*PI), which
 *   normalises straight back to the current heading. The error is zero, so the
 *   turn "completes" instantly and the robot never moves.
 *
 *   HeadingInterpolator.linear() normalises both endpoints and then uses
 *   getSmallestAngleDifference(), so it always takes the short way round. Ask
 *   for 360 and you get 0.
 *
 * So this closes the loop itself on getTotalHeading(), which ACCUMULATES
 * instead of normalising (the localizer does totalHeading += delta every
 * cycle). That is the only value in Pedro that can tell the difference between
 * "turned 0 degrees" and "turned 360 degrees".
 *
 * HOW IT DRIVES
 * -------------
 * A proportional controller on the angle remaining. It leaves at MAX_POWER,
 * holds that through most of the turn, and ramps down smoothly over roughly
 * the last 100 degrees, so the whole thing is one continuous motion that eases
 * into the target rather than four separate turns.
 *
 * WHAT TO WATCH
 * -------------
 *   TURNED should finish at 360.0, and the tape mark on the chassis should
 *   line back up with the mark on the floor. If the number says 360 but the
 *   robot is visibly off, your turnTicksToInches is wrong.
 *
 *   DRIFT is the important one. During a pure rotation the robot's X and Y
 *   should not change. If drift climbs to several inches, the pod offsets in
 *   Constants.java (leftPodY / rightPodY / strafePodX) are wrong. That is a
 *   measurement problem, not a tuning problem.
 *
 * TUNING IF IT MISBEHAVES
 *   Overshoots past 360 and comes back  -> lower MAX_POWER, or lower KP
 *   Stalls before reaching 360          -> raise MIN_POWER
 *   Slows down too early and crawls     -> raise KP
 */
@Autonomous(name = "Spin Test (360)", group = "Tuning")
public class SpinTest extends OpMode {

    /** Full turn. Set to -360 to spin clockwise instead. */
    private static final double TARGET_DEGREES = 360.0;

    /** Fastest it will spin. Start conservative and raise it once it behaves. */
    private static final double MAX_POWER = 0.45;

    /**
     * Floor power, so it doesn't stall against friction. Lowered from 0.12,
     * which was strong enough to shove the robot straight through the target.
     */
    private static final double MIN_POWER = 0.07;

    /**
     * Proportional gain, power per radian remaining. LOWER means it starts
     * easing off EARLIER, because the ramp begins once KP * remaining drops
     * below MAX_POWER. At 0.30 that's about 86 degrees out, instead of 52.
     */
    private static final double KP = 0.30;

    /** Close enough, in degrees. */
    private static final double TOLERANCE_DEG = 1.0;

    /** Must stay inside tolerance this long before we call it done. */
    private static final double SETTLE_MS = 350;

    private Follower follower;

    private double startTotalHeading;
    private boolean finished;
    private double worstDrift;
    private double finalTurned;
    private double settleStart = -1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        telemetry.addLine("Mark the floor and the chassis so you can check");
        telemetry.addLine("the robot really comes back square.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
        startTotalHeading = follower.getTotalHeading();
        finished = false;
        worstDrift = 0;
        settleStart = -1;
        resetRuntime();
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();
        double drift = Math.hypot(pose.getX(), pose.getY());
        if (drift > worstDrift) worstDrift = drift;

        // Accumulated, NOT normalised. This is what makes a full turn possible.
        double turnedRad = follower.getTotalHeading() - startTotalHeading;
        double targetRad = Math.toRadians(TARGET_DEGREES);
        double remainingRad = targetRad - turnedRad;

        boolean insideTolerance = Math.abs(remainingRad) < Math.toRadians(TOLERANCE_DEG);

        if (!finished) {
            if (insideTolerance) {
                // Cut power, but keep watching. The controller stays live, so
                // if momentum carries the robot past the target, remainingRad
                // goes negative on the next loop and we drive it back. The old
                // version declared victory here and let the robot coast on.
                follower.setTeleOpDrive(0, 0, 0, true);

                if (settleStart < 0) settleStart = getRuntime() * 1000.0;
                if (getRuntime() * 1000.0 - settleStart > SETTLE_MS) {
                    finished = true;
                    finalTurned = Math.toDegrees(turnedRad);
                }
            } else {
                settleStart = -1;   // drifted back out, restart the settle clock

                // Proportional, clamped between a floor and a ceiling. The sign
                // comes from remainingRad, so overshooting reverses the turn.
                double power = KP * remainingRad;
                double magnitude = Math.min(Math.abs(power), MAX_POWER);
                magnitude = Math.max(magnitude, MIN_POWER);
                power = Math.copySign(magnitude, remainingRad);

                // setTeleOpDrive(forward, strafe, turn, robotCentric).
                // Positive turn is counterclockwise.
                follower.setTeleOpDrive(0, 0, power, true);
            }
        } else {
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        telemetry.addData("TURNED (deg)", "%.2f", Math.toDegrees(turnedRad));
        telemetry.addData("target (deg)", "%.1f", TARGET_DEGREES);
        telemetry.addData("remaining (deg)", "%.2f", Math.toDegrees(remainingRad));
        telemetry.addLine();
        telemetry.addLine("--- these should stay near ZERO ---");
        telemetry.addData("x", "%.2f", pose.getX());
        telemetry.addData("y", "%.2f", pose.getY());
        telemetry.addData("DRIFT (in)", "%.2f", drift);
        telemetry.addData("worst drift (in)", "%.2f", worstDrift);

        if (finished) {
            telemetry.addLine();
            telemetry.addData("FINISHED at (deg)", "%.2f", finalTurned);
            telemetry.addLine("Check the chassis mark against the floor mark.");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) follower.setTeleOpDrive(0, 0, 0, true);
    }
}
