package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * DUAL TURRET APRILTAG AIMING TEST
 *
 * Drive the robot by hand. Two servo horns try to keep pointing at the centre
 * of one printed AprilTag. The webcam rides on the first servo; the second
 * servo aims from its own position using the same sightings.
 *
 * This is a pointing proof of concept with bare horns. No shooters, no
 * ballistics, no autonomous paths. It does not touch the competition TeleOp
 * or any existing OpMode.
 *
 * ===================================================================
 * GAMEPAD MAP
 * ===================================================================
 * GAMEPAD 1 - driving, exactly as usual, never overridden by the turrets
 *   left stick        translate, FIELD CENTRIC
 *   right stick X     rotate
 *   right bumper      hold for 30% speed
 *   OPTIONS           re-zero "driver forward" to the way the robot faces now
 *   START             toggle field centric <-> robot centric
 *
 * GAMEPAD 2 - turrets (gamepad 1 also works, so one controller is enough)
 *   A                 enable automatic tracking
 *   B                 disable tracking, hold current servo positions
 *   X                 select CAMERA turret for manual jogging
 *   Y                 select SECOND turret for manual jogging
 *   dpad left/right   jog the selected turret by one small step
 *   dpad up           send the selected turret to its calibrated forward pos
 *   left bumper       send BOTH turrets to their forward positions
 *   back              forget the learned tag position
 *
 * Automatic tracking starts DISABLED every time. Nothing moves on init.
 * There is no startup sweep and no unverified centring command: until you
 * press dpad up or left bumper, we do not know where the horns are and we do
 * not pretend to.
 *
 * ===================================================================
 * FIELD CENTRIC DRIVING
 * ===================================================================
 * Push the left stick away from you and the robot travels in one fixed field
 * direction regardless of which way its nose is pointing. That is the whole
 * point here: while the turrets are swinging around tracking a tag, the nose
 * of the robot stops being a useful reference for the driver.
 *
 * The heading comes from the follower, which is the same odometry estimate the
 * turret maths already depends on. No second source, no second IMU read.
 *
 * "Driver forward" is whichever way the robot was pointing at INIT, so set it
 * down facing away from the driver station. If it ever drifts, or you pick the
 * robot up and put it back turned, press OPTIONS to re-zero.
 *
 * ===================================================================
 * STATES
 * ===================================================================
 *   CONFIG_REQUIRED     measurements missing from TurretConfig
 *   DISABLED            tracking off, servos holding
 *   ACQUIRING           tracking on, no usable sighting yet
 *   TRACKING_ESTIMATED  aiming at a target learned from vision, kept current
 *                       with odometry. "ESTIMATED" because turret angle is a
 *                       command-based estimate, not measured.
 *   PREDICTING          tag not visible; aiming from odometry alone, briefly
 *   LOST                prediction expired; servos holding. A held servo
 *                       angle does NOT keep pointing at the tag once the
 *                       chassis moves.
 *   LIMIT               a turret cannot reach the angle it was asked for
 *   VISION_ONLY         no capture-time pose available; see note below
 */
@TeleOp(name = "Dual Turret AprilTag Aim TEST", group = "Turret")
public class DualTurretAimTest extends OpMode {

    private Follower follower;
    private TagTracker tracker;
    private ServoTurret camTurret, secondTurret;
    private PoseBuffer poseBuffer;

    /**
     * RAW SERVO ACCESS, independent of calibration.
     *
     * A ServoTurret needs FORWARD_POS and POS_PER_DEG to exist, but you find
     * those two numbers BY JOGGING THE SERVO. Without this, calibrating is
     * impossible: you would need the calibration in order to produce the
     * calibration.
     *
     * So we always grab the bare Servo objects too. Jogging works from the
     * very first run with every value UNSET, and reports the raw command
     * position, which is exactly the number you write into TurretConfig.
     *
     * Raw jogging is clamped to 0..1 only. The configured travel limits do not
     * exist yet, which is precisely why you do this with bare horns and
     * nothing mounted.
     */
    private com.qualcomm.robotcore.hardware.Servo rawCam, rawSecond;
    private double rawCamPos = 0.5, rawSecondPos = 0.5;
    private boolean rawCamCommanded = false, rawSecondCommanded = false;

    private boolean configOk, visionOk;
    private String initError = null;

    // Learned target.
    private Geometry.Vec targetField = null;     // FIELD frame, the good case
    private Geometry.Vec targetRobotOnly = null; // ROBOT frame fallback
    private boolean visionOnlyMode = false;
    private long lastGoodObservationNanos = 0;

    // Controls.
    private boolean trackingEnabled = false;
    private boolean selectCam = true;
    private static final double JOG_STEP = 0.005;

    /**
     * Driving frame.
     *
     * fieldCentric true  - stick directions are field directions
     * fieldCentric false - stick directions are relative to the robot's nose
     *
     * fieldOffset is the rotation applied to the driver's stick vector, in
     * radians. 0 means "forward on the stick is the way the robot faced at
     * INIT". Setting it to the current heading makes forward mean "the way the
     * robot is facing right now", which is what the re-zero button does.
     */
    private boolean fieldCentric = true;
    private double fieldOffset = 0.0;

    // Edge detection.
    private boolean prevA, prevB, prevX, prevY, prevUp, prevLeft, prevRight, prevLB, prevBack;
    private boolean prevOptions, prevStart;

    private String state = "DISABLED";
    private String lastNote = "";

    // =================================================================

    @Override
    public void init() {
        configOk = TurretConfig.aimingReady();

        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(0, 0, 0));
        } catch (Exception e) {
            initError = "Follower init failed: " + e.getMessage();
        }

        poseBuffer = new PoseBuffer(TurretConfig.POSE_BUFFER_MS);

        // Vision needs only the tag size, so diagnostics work long before the
        // mounting measurements exist.
        if (TurretConfig.visionReady()) {
            try {
                tracker = new TagTracker(hardwareMap);
                visionOk = true;
            } catch (Exception e) {
                visionOk = false;
                initError = "Camera init failed: " + e.getMessage();
            }
        }

        // Build a turret only if EVERY value it needs is measured. A partly
        // configured servo would do NaN arithmetic and write garbage
        // positions, which is worse than not existing.
        boolean camCal = allSet(TurretConfig.CAM_SERVO_FORWARD_POS, TurretConfig.CAM_SERVO_POS_PER_DEG,
                TurretConfig.CAM_SERVO_DIRECTION, TurretConfig.CAM_MOUNT_ANGLE_DEG,
                TurretConfig.CAM_SERVO_MIN_POS, TurretConfig.CAM_SERVO_MAX_POS);
        boolean secondCal = allSet(TurretConfig.SECOND_SERVO_FORWARD_POS, TurretConfig.SECOND_SERVO_POS_PER_DEG,
                TurretConfig.SECOND_SERVO_DIRECTION, TurretConfig.SECOND_MOUNT_ANGLE_DEG,
                TurretConfig.SECOND_SERVO_MIN_POS, TurretConfig.SECOND_SERVO_MAX_POS);

        // Bare servo handles, always, so calibration jogging is possible.
        // Same widened pulse range as the calibrated wrappers, so the numbers
        // you read while jogging are the numbers the tracker will use.
        try {
            rawCam = ServoTurret.applyPwmRange(hardwareMap, TurretConfig.CAM_SERVO_NAME);
        } catch (Exception e) { rawCam = null; }
        try {
            rawSecond = ServoTurret.applyPwmRange(hardwareMap, TurretConfig.SECOND_SERVO_NAME);
        } catch (Exception e) { rawSecond = null; }

        try {
            if (camCal) {
                camTurret = new ServoTurret(hardwareMap,
                        TurretConfig.CAM_SERVO_NAME,
                        TurretConfig.CAM_SERVO_FORWARD_POS,
                        TurretConfig.CAM_SERVO_POS_PER_DEG,
                        TurretConfig.CAM_SERVO_DIRECTION,
                        TurretConfig.CAM_MOUNT_ANGLE_DEG,
                        TurretConfig.CAM_SERVO_MIN_POS,
                        TurretConfig.CAM_SERVO_MAX_POS,
                        TurretConfig.CAM_MAX_DEG_PER_SEC,
                        TurretConfig.CAM_DEADBAND_DEG);
            }
            if (secondCal) {
                secondTurret = new ServoTurret(hardwareMap,
                        TurretConfig.SECOND_SERVO_NAME,
                        TurretConfig.SECOND_SERVO_FORWARD_POS,
                        TurretConfig.SECOND_SERVO_POS_PER_DEG,
                        TurretConfig.SECOND_SERVO_DIRECTION,
                        TurretConfig.SECOND_MOUNT_ANGLE_DEG,
                        TurretConfig.SECOND_SERVO_MIN_POS,
                        TurretConfig.SECOND_SERVO_MAX_POS,
                        TurretConfig.SECOND_MAX_DEG_PER_SEC,
                        TurretConfig.SECOND_DEADBAND_DEG);
            }
        } catch (Exception e) {
            initError = "Servo init failed: " + e.getMessage();
        }

        telemetry.addLine("Dual turret aim test. Tracking starts DISABLED.");
        telemetry.addLine("Nothing will move until you command it.");
        telemetry.addLine();
        telemetry.addLine("Driving is FIELD CENTRIC. Point the robot away from");
        telemetry.addLine("the driver station before pressing play.");
        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        driveRobot();
        readControls(now);

        // Record the EXISTING follower's pose with a timestamp. This is not a
        // second localizer; update() is called exactly once, above.
        Pose pose = (follower != null) ? follower.getPose() : null;
        boolean poseValid = pose != null
                && !Double.isNaN(pose.getX())
                && !Double.isNaN(pose.getY())
                && !Double.isNaN(pose.getHeading());
        if (poseValid) {
            poseBuffer.record(now, pose.getX(), pose.getY(), pose.getHeading());
        } else {
            // Localization is unusable; anything learned from it is suspect.
            invalidateTarget("localization invalid");
        }

        if (visionOk) {
            // Camera controls only exist once streaming has started, so this
            // is attempted every loop and becomes a no-op once it succeeds.
            tracker.lockFocusIfNeeded();
            consumeVision(now, poseValid);
        }

        if (trackingEnabled && configOk) {
            runAiming(now, pose, poseValid);
        } else if (!configOk) {
            state = "CONFIG_REQUIRED";
        } else {
            state = "DISABLED";
        }

        reportTelemetry(now, pose, poseValid);
    }

    @Override
    public void stop() {
        if (tracker != null) tracker.close();
    }

    // =================================================================
    // Driving - always responsive, never overridden
    // =================================================================

    /**
     * FIELD CENTRIC DRIVE.
     *
     * Pedro's flag is named for the frame the numbers are IN, which reads
     * backwards the first time you see it:
     *
     *   isRobotCentric = true   these numbers are in the robot's frame, so
     *                           rotate them by the heading before using them
     *   isRobotCentric = false  these numbers are already field-frame, use
     *                           them as they are
     *
     * So field centric driving passes FALSE. The extra argument is a rotation
     * applied to the field frame, which is how the re-zero button works.
     *
     * Heading comes from the follower's pose, so if localization is bad the
     * driving degrades with it. That is the right trade: the turret maths is
     * already built on the same estimate, and a quietly separate IMU read
     * would mean the driver and the turrets disagreeing about which way the
     * robot is facing.
     */
    private void driveRobot() {
        if (follower == null) return;

        double scale = gamepad1.right_bumper ? 0.30 : 1.0;

        double forward = -gamepad1.left_stick_y * scale;
        double strafe  = -gamepad1.left_stick_x * scale;
        double turn    = -gamepad1.right_stick_x * scale;

        if (fieldCentric) {
            follower.setTeleOpDrive(forward, strafe, turn, false, fieldOffset);
        } else {
            follower.setTeleOpDrive(forward, strafe, turn, true);
        }

        follower.update();
    }

    // =================================================================
    // Controls
    // =================================================================

    private void readControls(long now) {
        // Turret controls accept EITHER gamepad, so a single controller can do
        // the whole test. No clash with driving: the drivetrain only reads the
        // sticks, the RIGHT bumper, OPTIONS and START, and none of those
        // overlap with the turret buttons below.
        boolean a  = gamepad1.a || gamepad2.a;
        boolean b  = gamepad1.b || gamepad2.b;
        boolean x  = gamepad1.x || gamepad2.x;
        boolean y  = gamepad1.y || gamepad2.y;
        boolean up    = gamepad1.dpad_up    || gamepad2.dpad_up;
        boolean left  = gamepad1.dpad_left  || gamepad2.dpad_left;
        boolean right = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean lb    = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean back  = gamepad1.back || gamepad2.back;

        // --- driving frame, gamepad 1 only ---------------------------
        if (gamepad1.options && !prevOptions) {
            if (follower != null) {
                fieldOffset = follower.getPose().getHeading();
                lastNote = String.format("driver forward re-zeroed to %.0f deg",
                        Math.toDegrees(fieldOffset));
            }
        }
        prevOptions = gamepad1.options;

        if (gamepad1.start && !prevStart) {
            fieldCentric = !fieldCentric;
            lastNote = fieldCentric ? "driving: FIELD CENTRIC" : "driving: robot centric";
        }
        prevStart = gamepad1.start;

        if (a && !prevA) {
            if (configOk) { trackingEnabled = true;  lastNote = "tracking enabled"; }
            else          { lastNote = "cannot enable: config incomplete"; }
        }
        if (b && !prevB) { trackingEnabled = false; lastNote = "tracking disabled, holding"; }
        if (x && !prevX) { selectCam = true;  lastNote = "manual: CAMERA turret"; }
        if (y && !prevY) { selectCam = false; lastNote = "manual: SECOND turret"; }

        ServoTurret sel = selectCam ? camTurret : secondTurret;

        if (up && !prevUp) {
            if (sel != null) {
                sel.goToForward(now);
                lastNote = sel.getName() + " -> forward position";
            } else {
                lastNote = "no calibrated forward position yet - jog to find it";
            }
        }
        if (left && !prevLeft)  jogSelected(-JOG_STEP, sel, now);
        if (right && !prevRight) jogSelected(+JOG_STEP, sel, now);
        if (lb && !prevLB) {
            if (camTurret != null)    camTurret.goToForward(now);
            if (secondTurret != null) secondTurret.goToForward(now);
            lastNote = "both turrets -> forward position";
        }
        if (back && !prevBack) invalidateTarget("cleared by driver");

        prevA = a; prevB = b; prevX = x; prevY = y;
        prevUp = up; prevLeft = left; prevRight = right;
        prevLB = lb; prevBack = back;
    }

    /**
     * Jog the selected turret. Uses the calibrated wrapper when it exists,
     * otherwise drives the bare servo so calibration can be discovered.
     */
    private void jogSelected(double delta, ServoTurret sel, long now) {
        if (sel != null) {
            sel.jogPosition(delta, now);
            lastNote = sel.getName() + (delta > 0 ? " jog +" : " jog -");
            return;
        }

        com.qualcomm.robotcore.hardware.Servo raw = selectCam ? rawCam : rawSecond;
        if (raw == null) { lastNote = "servo not found in configuration"; return; }

        if (selectCam) {
            rawCamPos = clamp01(rawCamPos + delta);
            raw.setPosition(rawCamPos);
            rawCamCommanded = true;
            lastNote = String.format("RAW %s -> %.4f", TurretConfig.CAM_SERVO_NAME, rawCamPos);
        } else {
            rawSecondPos = clamp01(rawSecondPos + delta);
            raw.setPosition(rawSecondPos);
            rawSecondCommanded = true;
            lastNote = String.format("RAW %s -> %.4f", TurretConfig.SECOND_SERVO_NAME, rawSecondPos);
        }
    }

    private static double clamp01(double v) { return Math.max(0, Math.min(1, v)); }

    private static boolean allSet(double... vals) {
        for (double v : vals) if (!TurretConfig.isSet(v)) return false;
        return true;
    }

    private void invalidateTarget(String why) {
        if (targetField != null || targetRobotOnly != null) lastNote = "target forgotten: " + why;
        targetField = null;
        targetRobotOnly = null;
        visionOnlyMode = false;
    }

    // =================================================================
    // Vision -> target estimate
    // =================================================================

    // Raw measurements from the most recent accepted sighting, recorded
    // BEFORE any of the geometry that needs mounting measurements. This is
    // what makes vision diagnostics usable while everything else is still
    // UNCONFIGURED: you can check range and bearing against a tape measure
    // long before a turret exists to aim.
    private double rawRange = Double.NaN, rawBearing = Double.NaN;
    private double rawX = Double.NaN, rawY = Double.NaN, rawMargin = Double.NaN;
    private long   rawNanos = 0;

    private void consumeVision(long now, boolean poseValid) {
        TagTracker.Observation obs = tracker.poll(now);
        if (obs == null) return;

        rawRange   = obs.range;
        rawBearing = obs.bearingDeg;
        rawX       = obs.xInches;
        rawY       = obs.yInches;
        rawMargin  = obs.decisionMargin;
        rawNanos   = obs.captureNanos;

        // The camera turret's angle WHEN THE SHUTTER FIRED. If the turret has
        // never been commanded we have no estimate, and guessing would put the
        // tag in the wrong place entirely.
        if (camTurret == null || !camTurret.hasBeenCommanded()) {
            lastNote = "sighting ignored: camera turret not yet commanded to forward";
            return;
        }
        double camYawAtCapture = camTurret.estimatedYawAt(obs.captureNanos);
        if (Double.isNaN(camYawAtCapture)) {
            lastNote = "sighting ignored: no turret angle estimate at capture time";
            return;
        }

        // Camera frame -> robot frame.
        Geometry.Vec tagInCam = Geometry.fromFtcPose(obs.xInches, obs.yInches);
        Geometry.Vec tagInRobot = Geometry.tagToRobotFrame(
                tagInCam,
                Math.toRadians(TurretConfig.CAM_MOUNT_YAW_DEG),
                new Geometry.Vec(TurretConfig.CAM_LENS_OFFSET_X_IN, TurretConfig.CAM_LENS_OFFSET_Y_IN),
                camYawAtCapture,
                new Geometry.Vec(TurretConfig.CAM_PIVOT_X_IN, TurretConfig.CAM_PIVOT_Y_IN));

        // Robot frame -> field frame, using the pose at capture.
        PoseBuffer.Sample at = poseValid ? poseBuffer.nearest(obs.captureNanos) : null;

        if (at == null) {
            // VISION-ONLY FALLBACK.
            // We have a sighting but cannot say where the robot was when it
            // was taken, so we cannot place the tag on the field. We keep the
            // target in ROBOT coordinates instead.
            //
            // LIMITATION: a robot-frame target is only correct while the robot
            // does not move. Drive, and the aim goes stale immediately, with
            // no way to correct it. Useful for bench testing with the chassis
            // stationary; not useful while driving.
            targetRobotOnly = tagInRobot;
            visionOnlyMode = true;
            lastGoodObservationNanos = now;
            return;
        }

        Geometry.Vec observedField = Geometry.robotToField(tagInRobot, at.pos(), at.heading);

        if (targetField == null) {
            targetField = observedField;
        } else {
            double jump = observedField.minus(targetField).norm();
            if (jump > TurretConfig.MAX_TARGET_JUMP_IN) {
                lastNote = String.format("rejected %.1f in jump in target", jump);
                return;
            }
            double k = TurretConfig.TARGET_SMOOTHING;
            targetField = new Geometry.Vec(
                    targetField.x + k * (observedField.x - targetField.x),
                    targetField.y + k * (observedField.y - targetField.y));
        }

        visionOnlyMode = false;
        targetRobotOnly = null;
        lastGoodObservationNanos = now;
    }

    // =================================================================
    // Aiming
    // =================================================================

    private double camRequested = Double.NaN, secondRequested = Double.NaN;
    private double camRange = Double.NaN, secondRange = Double.NaN;

    private void runAiming(long now, Pose pose, boolean poseValid) {
        double sinceObsMs = (now - lastGoodObservationNanos) / 1e6;
        boolean haveTarget = (targetField != null) || (targetRobotOnly != null);

        if (!haveTarget) { state = "ACQUIRING"; return; }

        if (sinceObsMs > TurretConfig.PREDICT_TIMEOUT_MS) {
            // Hold. Note clearly that holding an ANGLE is not holding an AIM:
            // if the chassis keeps moving the horns are pointing at empty air.
            state = "LOST";
            return;
        }

        Geometry.Vec targetInRobot;
        if (targetField != null && poseValid) {
            targetInRobot = Geometry.fieldToRobot(targetField,
                    new Geometry.Vec(pose.getX(), pose.getY()), pose.getHeading());
        } else if (targetRobotOnly != null) {
            targetInRobot = targetRobotOnly;   // stale the instant we move
        } else {
            state = "ACQUIRING";
            return;
        }

        boolean fresh = sinceObsMs <= TurretConfig.MAX_FRAME_AGE_MS;
        if (visionOnlyMode)  state = "VISION_ONLY";
        else if (fresh)      state = "TRACKING_ESTIMATED";
        else                 state = "PREDICTING";

        // Each turret solves from ITS OWN pivot. No copying, no fixed offset.
        if (camTurret != null && camTurret.hasBeenCommanded()) {
            Geometry.Vec pivot = new Geometry.Vec(
                    TurretConfig.CAM_PIVOT_X_IN, TurretConfig.CAM_PIVOT_Y_IN);
            camRequested = Geometry.desiredYaw(targetInRobot, pivot);
            camRange = Geometry.rangeFromPivot(targetInRobot, pivot);
            camTurret.aimAt(camRequested, now);
        }

        if (secondTurret != null && secondTurret.hasBeenCommanded()) {
            Geometry.Vec pivot = new Geometry.Vec(
                    TurretConfig.SECOND_PIVOT_X_IN, TurretConfig.SECOND_PIVOT_Y_IN);
            secondRequested = Geometry.desiredYaw(targetInRobot, pivot);
            secondRange = Geometry.rangeFromPivot(targetInRobot, pivot);
            secondTurret.aimAt(secondRequested, now);
        }

        boolean limited =
                (camTurret != null && camTurret.getLimitState() != ServoTurret.LimitState.OK)
             || (secondTurret != null && secondTurret.getLimitState() != ServoTurret.LimitState.OK);
        if (limited) state = "LIMIT";
    }

    // =================================================================
    // Telemetry
    // =================================================================

    private void reportTelemetry(long now, Pose pose, boolean poseValid) {
        telemetry.addData("STATE", state);
        if (initError != null) telemetry.addData("INIT ERROR", initError);
        if (!lastNote.isEmpty()) telemetry.addData("note", lastNote);

        telemetry.addLine();
        telemetry.addData("tracking", trackingEnabled ? "ENABLED" : "disabled");
        telemetry.addData("manual select", selectCam ? "CAMERA turret" : "SECOND turret");

        // --- driving ---
        telemetry.addLine();
        telemetry.addData("driving", fieldCentric ? "FIELD CENTRIC" : "robot centric");
        telemetry.addData("driver forward", "%.0f deg (OPTIONS re-zeroes)",
                Math.toDegrees(fieldOffset));
        telemetry.addData("drive speed", gamepad1.right_bumper ? "30%" : "full");

        // --- configuration ---
        telemetry.addLine();
        String[] missing = TurretConfig.missingSettings();
        telemetry.addData("config", missing.length == 0
                ? "complete"
                : missing.length + " value(s) UNCONFIGURED");
        for (int i = 0; i < Math.min(missing.length, 6); i++) {
            telemetry.addData("  missing", missing[i]);
        }
        if (missing.length > 6) telemetry.addData("  ...", (missing.length - 6) + " more");
        telemetry.addData("tag size", TurretConfig.isSet(TurretConfig.TAG_SIZE_IN)
                ? String.format("%.3f in", TurretConfig.TAG_SIZE_IN) : "UNCONFIGURED");
        telemetry.addData("camera calibration", TurretConfig.intrinsicsProvided()
                ? "explicit from TurretConfig"
                : "SDK default (built-in table or FOV estimate) - ranges approximate");

        // --- camera ---
        telemetry.addLine();
        if (tracker != null) {
            telemetry.addData("camera", tracker.getCameraState());
            telemetry.addData("fps", "%.1f", tracker.getFps());
            telemetry.addData("detections", tracker.getDetectionCount());
            telemetry.addData("ids in frame", tracker.getVisibleIds());
            telemetry.addData("target id", TurretConfig.TARGET_TAG_ID + " (aiming)");
            if (tracker.isSecondaryVisible()) {
                telemetry.addData("neighbour " + TurretConfig.SECONDARY_TAG_ID,
                        "visible, deliberately ignored");
            }
            telemetry.addData("last reject", tracker.getLastReject());
            telemetry.addData("FOCUS", tracker.getFocusReport());
        } else {
            telemetry.addData("camera", TurretConfig.visionReady()
                    ? "NOT INITIALISED" : "waiting on TAG_SIZE_IN");
        }
        // RAW TAG MEASUREMENTS. Straight from the camera, no mounting
        // geometry applied, so these work with every other value UNCONFIGURED.
        // Use them for the tape-measure sanity check.
        telemetry.addLine();
        if (Double.isNaN(rawRange)) {
            telemetry.addData("RAW tag", "no accepted sighting yet");
        } else {
            double rawAgeMs = (now - rawNanos) / 1e6;
            telemetry.addData("RAW range", "%.1f in", rawRange);
            telemetry.addData("RAW bearing", "%.1f deg (+ = left of centre)", rawBearing);
            telemetry.addData("RAW x,y", "%.1f right, %.1f fwd (SDK frame)", rawX, rawY);
            telemetry.addData("RAW margin", "%.0f", rawMargin);
            telemetry.addData("RAW frame age", "%.0f ms", rawAgeMs);
        }

        double ageMs = lastGoodObservationNanos == 0
                ? Double.NaN : (now - lastGoodObservationNanos) / 1e6;
        telemetry.addData("observation age", Double.isNaN(ageMs)
                ? "never" : String.format("%.0f ms", ageMs));
        telemetry.addData("aiming source", state.equals("TRACKING_ESTIMATED") ? "FRESH VISION"
                : state.equals("PREDICTING") ? "ODOMETRY PREDICTION"
                : state.equals("VISION_ONLY") ? "VISION ONLY (robot frame, stale if moving)"
                : "none");

        // --- localization ---
        telemetry.addLine();
        telemetry.addData("localization", poseValid ? "valid" : "INVALID");
        if (pose != null) {
            telemetry.addData("robot", "x %.1f  y %.1f  h %.1f deg",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        }
        telemetry.addData("target (field)", targetField == null ? "none" : targetField.toString());
        if (targetRobotOnly != null) {
            telemetry.addData("target (robot only)", targetRobotOnly.toString());
        }

        // --- turrets ---
        telemetry.addLine();
        reportTurret("CAM ", camTurret, camRequested, camRange);
        reportTurret("2ND ", secondTurret, secondRequested, secondRange);

        telemetry.addLine();
        telemetry.addLine("Angles below are COMMAND-BASED ESTIMATES.");
        telemetry.addLine("Servos have no encoders; this is not measured.");

        telemetry.update();
    }

    private void reportTurret(String tag, ServoTurret t, double requested, double range) {
        if (t == null) {
            // Calibration mode. Show the raw command position, which is the
            // number you copy into TurretConfig once the horn points where
            // you want it.
            boolean isCam = tag.startsWith("CAM");
            boolean commanded = isCam ? rawCamCommanded : rawSecondCommanded;
            double pos = isCam ? rawCamPos : rawSecondPos;
            boolean present = (isCam ? rawCam : rawSecond) != null;

            if (!present) {
                telemetry.addData(tag, "SERVO NOT IN CONFIG");
            } else if (!commanded) {
                telemetry.addData(tag, "RAW jog mode - dpad left/right to move");
            } else {
                telemetry.addData(tag + "RAW pos", "%.4f  <- write this into TurretConfig", pos);
            }
            return;
        }
        if (!t.hasBeenCommanded()) {
            telemetry.addData(tag, "position unknown - press dpad up or LB first");
            return;
        }
        telemetry.addData(tag + "requested", Double.isNaN(requested)
                ? "-" : String.format("%.1f deg", Math.toDegrees(requested)));
        telemetry.addData(tag + "estimated", "%.1f deg", Math.toDegrees(t.getCommandedYaw()));
        telemetry.addData(tag + "servo cmd", "%.4f", t.getCommandedPos());
        telemetry.addData(tag + "range", Double.isNaN(range) ? "-" : String.format("%.1f in", range));
        if (t.getLimitState() != ServoTurret.LimitState.OK) {
            telemetry.addData(tag + "LIMIT", "%s, short by %.1f deg",
                    t.getLimitState(), t.getLimitShortfallDeg());
        }
    }
}
