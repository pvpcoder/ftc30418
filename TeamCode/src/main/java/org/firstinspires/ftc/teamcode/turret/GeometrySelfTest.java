package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

/**
 * Assertions for the aiming maths. Runs on the robot, touches no hardware,
 * moves nothing. Run it once after any change to Geometry.
 *
 * This is an OpMode rather than a JUnit test on purpose: this project has no
 * test source set or test dependency, and adding one would mean editing the
 * shared Gradle files for a handful of pure functions. If you later want these
 * in CI, the assertions below port to JUnit unchanged once you add
 *
 *     testImplementation 'junit:junit:4.13.2'
 *
 * to TeamCode's dependencies and move them to TeamCode/src/test/java.
 *
 * Every check here is arithmetic that must hold regardless of what hardware
 * you eventually bolt on. None of it proves the robot is aimed correctly:
 * that needs a protractor and your eyes.
 */
@TeleOp(name = "Turret Geometry SELF TEST", group = "Turret")
public class GeometrySelfTest extends OpMode {

    private final List<String> results = new ArrayList<>();
    private int passed = 0, failed = 0;

    private static final double EPS = 1e-6;

    @Override
    public void init() {
        runAll();
    }

    @Override
    public void loop() {
        telemetry.addData("RESULT", failed == 0 ? "ALL PASS" : failed + " FAILED");
        telemetry.addData("passed", passed);
        telemetry.addData("failed", failed);
        telemetry.addLine();
        for (String r : results) telemetry.addLine(r);
        telemetry.update();
    }

    // =================================================================

    private void runAll() {
        testNormalize();
        testRotate();
        testFtcPoseConversion();
        testFieldRoundTrip();
        testPivotOffsetsMatter();
        testCameraTurretRotationMovesLens();
        testAngleWrapShortestRoute();
        testServoClampArithmetic();
        testStaleObservationRejected();
    }

    // --- basics ---

    private void testNormalize() {
        near("normalize 0", Geometry.normalize(0), 0);
        near("normalize 2pi -> 0", Geometry.normalize(2 * Math.PI), 0);
        near("normalize 3pi -> pi", Geometry.normalize(3 * Math.PI), Math.PI);
        near("normalize -3pi/2 -> pi/2", Geometry.normalize(-3 * Math.PI / 2), Math.PI / 2);
        check("normalize keeps range", Math.abs(Geometry.normalize(100.0)) <= Math.PI + EPS);
    }

    private void testRotate() {
        Geometry.Vec v = new Geometry.Vec(1, 0);
        Geometry.Vec r = Geometry.rotate(v, Math.PI / 2);
        near("rotate +90 x", r.x, 0);
        near("rotate +90 y", r.y, 1);

        Geometry.Vec back = Geometry.rotate(r, -Math.PI / 2);
        near("rotate round trip x", back.x, 1);
        near("rotate round trip y", back.y, 0);
    }

    private void testFtcPoseConversion() {
        // SDK: x right, y forward. Ours: x forward, y left.
        // A tag 10 forward and 3 to the RIGHT must come out as
        // 10 forward and -3 left.
        Geometry.Vec v = Geometry.fromFtcPose(3, 10);
        near("ftcPose forward", v.x, 10);
        near("ftcPose right becomes negative left", v.y, -3);
    }

    private void testFieldRoundTrip() {
        Geometry.Vec inRobot = new Geometry.Vec(24, -6);
        Geometry.Vec robotPos = new Geometry.Vec(100, 50);
        double heading = Math.toRadians(37);

        Geometry.Vec field = Geometry.robotToField(inRobot, robotPos, heading);
        Geometry.Vec back = Geometry.fieldToRobot(field, robotPos, heading);

        near("field round trip x", back.x, inRobot.x);
        near("field round trip y", back.y, inRobot.y);
    }

    // --- the property that matters most ---

    private void testPivotOffsetsMatter() {
        // One target, two turrets 12 inches apart across the robot.
        Geometry.Vec target = new Geometry.Vec(60, 0);
        Geometry.Vec leftPivot  = new Geometry.Vec(0, +6);
        Geometry.Vec rightPivot = new Geometry.Vec(0, -6);

        double aLeft  = Geometry.desiredYaw(target, leftPivot);
        double aRight = Geometry.desiredYaw(target, rightPivot);

        check("two pivots give different angles", Math.abs(aLeft - aRight) > 1e-3);
        check("left pivot aims right of centre", aLeft < 0);
        check("right pivot aims left of centre", aRight > 0);

        // And the difference must SHRINK with distance. This is exactly why a
        // fixed angular correction between the turrets is wrong.
        Geometry.Vec farTarget = new Geometry.Vec(600, 0);
        double fLeft  = Geometry.desiredYaw(farTarget, leftPivot);
        double fRight = Geometry.desiredYaw(farTarget, rightPivot);

        check("turret disagreement shrinks with range",
                Math.abs(fLeft - fRight) < Math.abs(aLeft - aRight));
    }

    private void testCameraTurretRotationMovesLens() {
        // Lens 4 inches out in front of the pivot, pivot at the robot origin,
        // camera square on its bracket.
        Geometry.Vec lensOffset = new Geometry.Vec(4, 0);
        Geometry.Vec pivot = new Geometry.Vec(0, 0);
        Geometry.Vec tagStraightAhead = new Geometry.Vec(50, 0);   // 50 in down the axis

        // Turret at 0: tag should be 54 inches straight ahead in robot frame.
        Geometry.Vec at0 = Geometry.tagToRobotFrame(tagStraightAhead, 0, lensOffset, 0, pivot);
        near("turret 0, tag ahead x", at0.x, 54);
        near("turret 0, tag ahead y", at0.y, 0);

        // Turret swung 90 degrees CCW: the whole assembly, lens included,
        // rotates. The tag must now appear 54 inches to the LEFT, not 50 left
        // plus a stray 4 forward. This is the check that catches treating a
        // rotating camera as if only its heading changed.
        Geometry.Vec at90 = Geometry.tagToRobotFrame(
                tagStraightAhead, 0, lensOffset, Math.PI / 2, pivot);
        near("turret 90, tag left x", at90.x, 0);
        near("turret 90, tag left y", at90.y, 54);
    }

    // --- servo-facing arithmetic ---

    private void testAngleWrapShortestRoute() {
        // A turret mounted facing 170 deg, asked to point at -170 deg.
        // The short way is +20 deg through 180, not -340.
        double mount = Math.toRadians(170);
        double want  = Math.toRadians(-170);
        double delta = Math.toDegrees(Geometry.angleDelta(mount, want));
        near("wrap takes short route", delta, 20);

        double delta2 = Math.toDegrees(
                Geometry.angleDelta(Math.toRadians(-170), Math.toRadians(170)));
        near("wrap short route other way", delta2, -20);
    }

    private void testServoClampArithmetic() {
        // Mirrors the clamp inside ServoTurret without needing hardware.
        double min = 0.20, max = 0.80;

        near("clamp below", clamp(0.05, min, max), min);
        near("clamp above", clamp(0.95, min, max), max);
        near("clamp inside", clamp(0.50, min, max), 0.50);

        // Mapping must be its own inverse inside the band.
        double forwardPos = 0.50, posPerDeg = 0.002, direction = 1.0;
        double mountDeg = 0;
        double yawDeg = 30;
        double pos = forwardPos + direction * posPerDeg * (yawDeg - mountDeg);
        double backDeg = mountDeg + (pos - forwardPos) / (direction * posPerDeg);
        near("yaw->pos->yaw round trip", backDeg, yawDeg);

        check("30 deg stays inside a 0.20-0.80 band", pos > min && pos < max);

        // 200 degrees must NOT fit in that band, and must clamp.
        double farPos = forwardPos + direction * posPerDeg * 200;
        check("200 deg exceeds the band", farPos > max);
        near("and clamps to max", clamp(farPos, min, max), max);
    }

    private void testStaleObservationRejected() {
        PoseBuffer buf = new PoseBuffer(100);   // 100 ms window
        long t0 = 1_000_000_000L;
        buf.record(t0, 10, 20, 0.5);

        check("finds a pose at the same instant", buf.nearest(t0) != null);
        check("finds a pose 50 ms later", buf.nearest(t0 + 50_000_000L) != null);
        check("refuses a pose 500 ms later", buf.nearest(t0 + 500_000_000L) == null);

        PoseBuffer empty = new PoseBuffer(100);
        check("empty buffer returns null, not a guess", empty.nearest(t0) == null);
    }

    // =================================================================

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void near(String name, double actual, double expected) {
        boolean ok = Math.abs(actual - expected) < 1e-6;
        record(name, ok, ok ? "" : String.format(" (got %.6f, want %.6f)", actual, expected));
    }

    private void check(String name, boolean ok) {
        record(name, ok, "");
    }

    private void record(String name, boolean ok, String extra) {
        if (ok) { passed++; results.add("PASS  " + name); }
        else    { failed++; results.add("FAIL  " + name + extra); }
    }
}
