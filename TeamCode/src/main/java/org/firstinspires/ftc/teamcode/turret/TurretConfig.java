package org.firstinspires.ftc.teamcode.turret;

/**
 * EVERY tunable value for the dual-turret AprilTag aiming test lives here.
 * Nothing else in this package has a magic number in it.
 *
 * =====================================================================
 * UNITS AND CONVENTIONS - read this before changing anything
 * =====================================================================
 *
 * Distances : INCHES  (matches Pedro and the FTC AprilTag SDK output)
 * Angles    : DEGREES in this file. Converted to radians internally.
 * Time      : System.nanoTime() throughout.
 *
 * ROBOT FRAME
 *   +X is FORWARD out of the front of the robot
 *   +Y is to the robot's LEFT
 *   +yaw is COUNTERCLOCKWISE seen from above
 *   Origin is the robot's centre of rotation, the same point the Pedro pod
 *   offsets in Constants.java are measured from.
 *
 * CAMERA FRAME (what the SDK gives us)
 *   The SDK's ftcPose uses x = RIGHT, y = FORWARD, z = UP.
 *   That is a different handedness from our robot frame, so Geometry
 *   converts it once, explicitly, and nothing else touches ftcPose.
 *
 * =====================================================================
 * UNCONFIGURED VALUES
 * =====================================================================
 * Anything set to UNSET is a real physical measurement nobody has taken
 * yet. The OpMode refuses to enable automatic aiming while any required
 * value is UNSET, and tells you which. Nothing here is a guess dressed up
 * as a default. Vision diagnostics and manual servo calibration both work
 * with everything still UNSET, which is how you measure them in the first
 * place.
 */
public final class TurretConfig {

    private TurretConfig() {}

    /** Marker for "nobody has measured this yet". */
    public static final double UNSET = Double.NaN;

    public static boolean isSet(double v) { return !Double.isNaN(v); }

    // =================================================================
    // 1. HARDWARE NAMES
    //    Put these EXACT strings into the Robot Controller configuration
    //    on the Driver Station. Webcam goes under "USB Devices" as a
    //    Webcam; both servos go under "Servos" on the hub's servo ports.
    // =================================================================

    public static final String WEBCAM_NAME       = "turretCam";
    public static final String CAM_SERVO_NAME    = "camTurret";
    public static final String SECOND_SERVO_NAME = "aimTurret";

    // =================================================================
    // 2. TARGET
    // =================================================================

    /**
     * Which tag to aim at. Page 1 of BIOBUZZAprilTagProduction.pdf carries
     * tag36h11 IDs 30, 31, 32 and 33. 30 is the leftmost. We deliberately
     * track this one ID only, never an average of several.
     */
    public static final int TARGET_TAG_ID = 30;

    /**
     * Also on page 1, also measured, also in the tag library. We do NOT aim at
     * it. It is here so that if the camera is looking at the wrong tag the
     * telemetry says so explicitly, instead of just reporting "no detection"
     * and leaving you to guess whether the camera is blind or merely pointed
     * at the neighbour. Aiming filters to TARGET_TAG_ID alone.
     */
    public static final int SECONDARY_TAG_ID = 31;

    /**
     * MEASURED 2026-09-20 on our actual printout.
     *
     * Width of the OUTER BLACK SQUARE, excluding the surrounding white margin.
     * Tags 30 and 31 both came out at 4.0 in (0.1016 m), so one constant
     * covers both library entries.
     *
     * If you ever reprint the page, MEASURE IT AGAIN. Printers scale, "Fit to
     * page" and "Actual size" give different results, and the official
     * competition tag size does not apply to a printout. Every distance the
     * camera reports scales linearly with this number, so a 5% error here is
     * a 5% error in every range.
     */
    public static final double TAG_SIZE_IN = 4.0;

    // =================================================================
    // 3. CAMERA
    // =================================================================

    /**
     * EMEET C960. Advertised 1080p, but AprilTag detection does not want
     * 1080p: it costs frame rate and buys nothing, because tag corner
     * accuracy is limited by focus and motion blur long before pixels.
     * 640x480 is the sane starting point and is what the SDK samples use.
     *
     * If the camera refuses this resolution the OpMode reports the failure
     * instead of silently falling back.
     */
    public static final int CAMERA_WIDTH  = 640;
    public static final int CAMERA_HEIGHT = 480;

    /**
     * LENS INTRINSICS - fx, fy, cx, cy in pixels, for the resolution above.
     *
     * LEAVE THESE UNSET UNLESS YOU HAVE CALIBRATED YOUR OWN C960.
     *
     * When these are UNSET we do not call setLensIntrinsics at all, and the
     * SDK falls back to its own logic: it looks for a matching entry in its
     * built-in calibration file by USB vendor/product ID, and if it finds
     * none it estimates intrinsics from a guessed field of view.
     *
     * Do NOT paste in the 578.272/578.272/402.145/221.506 numbers from the
     * SDK samples. Those belong to a Logitech C270 at 640x480 and are wrong
     * for a C960.
     *
     * HOW TO CHECK WHAT YOU ACTUALLY GOT
     *   Run the OpMode and read the CAMERA CALIBRATION telemetry line, then
     *   check the Robot Controller log (Driver Station -> three dots ->
     *   "Logs", or adb logcat) for a line from the vision subsystem naming
     *   the calibration it chose. It will say either that it matched a known
     *   camera or that it is estimating from FOV.
     *
     * WHAT IT COSTS YOU IF UNCALIBRATED
     *   Estimated intrinsics mostly distort RANGE. Bearing to a tag near the
     *   centre of frame stays roughly right, so pointing works, but the
     *   reported distance can be off by 10-20% and gets worse toward the
     *   edges of the image. For this pointing test that is tolerable. For
     *   anything that has to compute a shot, calibrate first.
     *
     * HOW TO CALIBRATE
     *   Use the 3DF Zephyr / OpenCV chessboard procedure documented in
     *   ftc-docs under "Camera Calibration", then either add your numbers
     *   here or drop a teamwebcamcalibrations.xml onto the Robot Controller.
     */
    public static final double CAMERA_FX = UNSET;
    public static final double CAMERA_FY = UNSET;
    public static final double CAMERA_CX = UNSET;
    public static final double CAMERA_CY = UNSET;

    /**
     * LOCK THE LENS.
     *
     * The C960 autofocuses. Focusing physically moves the lens, which changes
     * its effective focal length, which is the exact number the range
     * calculation divides by. The SDK assumes a fixed lens and has no idea
     * this is happening.
     *
     * We saw it directly: 24 in read 24.1, then after a trip to 48 in and back
     * to the SAME 24 in spot it read 27.5. A stable bias does not change
     * between visits to the same place. A hunting autofocus does.
     *
     * With this true we put the camera in Fixed focus mode at startup, so the
     * lens stops moving and the intrinsics stop drifting.
     */
    public static final boolean LOCK_FOCUS = true;

    /**
     * Focus length to lock to, in whatever units the camera reports, or UNSET
     * to freeze whatever it happens to be focused at when the OpMode starts.
     *
     * UNSET is the safe default but it means the lock point depends on what
     * the camera saw during init, so point it at the tag before pressing INIT.
     * Once you find a value that works across your whole working range, read
     * it off the FOCUS telemetry and hard-code it here so every run is
     * identical.
     */
    public static final double FIXED_FOCUS_LENGTH = UNSET;

    /**
     * EMPIRICAL RANGE CORRECTION.
     *
     * Multiplies the camera's reported x, y and range. 1.0 means no change.
     *
     * WHY THIS IS LEGITIMATE AND NOT A FUDGE
     * An error in the assumed focal length scales the entire translation
     * vector the PnP solve produces - x, y, z and range all by the SAME
     * factor. Bearing is atan2(x, y), so the factor cancels and the angle
     * stays correct. That is precisely the pattern we measured: bearing
     * agreed with x and y at every distance, while range was consistently
     * high by the same percentage.
     *
     * So multiplying by a single constant here is mathematically equivalent
     * to correcting fx and fy, for the one thing fx and fy were wrong about.
     * It is not equivalent to a real calibration: it cannot fix lens
     * distortion, and it cannot fix a principal point that is off centre.
     * Those matter near the edges of the frame and for accurate pose of a
     * tilted tag. For pointing at a tag near the middle of the image, this is
     * enough.
     *
     * HOW TO FIND YOUR NUMBER
     *   1. Lock focus (see LOCK_FOCUS) and leave it locked.
     *   2. Measure lens-to-tag with a tape at two distances, say 24 and 48 in.
     *   3. Read RAW range at each.
     *   4. RANGE_SCALE = average of (tape / RAW range).
     *   Both ratios should agree within a percent or two. If they do not,
     *   something else is wrong and scaling will not fix it.
     *
     * MEASURED 2026-09-20, lens-to-tag by tape, focus locked:
     *   24 in tape -> 28.4 reported   ratio 1.1833
     *   24 in tape -> 28.5 reported   ratio 1.1875   (separate run, repeatable)
     *   48 in tape -> 57.2 reported   ratio 1.1917
     *
     * Mean over-read 1.1875, spread across the three only 0.7%. That tight a
     * spread across a 2:1 change in distance is the signature of a pure scale
     * error, which is what makes a single constant the right fix.
     *
     * 1 / 1.1875 = 0.8421
     */
    public static final double RANGE_SCALE = 0.8421;

    /**
     * Reject detections whose decision margin is below this. Low margin means
     * the decoder was unsure and the corners are probably sloppy. 30 is a
     * reasonable floor for a well-lit printed tag.
     */
    public static final double MIN_DECISION_MARGIN = 30.0;

    /** Ignore tags further away than this. Corner noise dominates out here. */
    public static final double MAX_ACCEPT_RANGE_IN = 120.0;

    // =================================================================
    // 4. GEOMETRY - camera turret
    // =================================================================

    /**
     * MEASURE THESE. Where the CAMERA turret's pivot axis sits in the robot
     * frame, in inches. +X forward, +Y left, from the robot's centre of
     * rotation.
     */
    // MEASURED 2026-09-24: camera servo shaft sits ~6 in ahead of the robot
    // centre. Robot centre found by halving 15.5 in across the wheels and
    // 11 in front to back, same origin as the Pedro pod offsets.
    public static final double CAM_PIVOT_X_IN = 6.0;
    // MEASURED 2026-09-24: 2 in to the RIGHT of the centreline, so negative.
    public static final double CAM_PIVOT_Y_IN = -2.0;

    /**
     * MEASURE THESE. Where the camera LENS sits relative to the camera
     * turret's PIVOT, in the turret's own rotating frame, in inches.
     *
     * This is the offset that makes a rotating camera different from a fixed
     * one. If the lens sat exactly on the pivot axis both of these would be
     * zero and the turret angle would only change the camera's heading. In
     * reality the lens is out in front of the pivot, so rotating the turret
     * swings the lens through an arc and moves it in the robot frame too.
     *
     * +X is out along the direction the camera looks when the turret is at
     * its forward position. +Y is to the left of that.
     */
    // MEASURED 2026-09-24: webcam zip-tied to the tip of the servo horn, so
    // the lens sits 1.5 in ahead of the shaft. Centred side to side on the
    // horn, hence Y offset 0.
    //
    // This is the number that makes rotating-camera geometry work. Swing the
    // turret 90 degrees and the lens does not just look 90 degrees across, it
    // physically travels 1.5 in around the pivot. Ignoring that puts the tag
    // in the wrong place by that much, which at close range is a real error.
    public static final double CAM_LENS_OFFSET_X_IN = 1.5;
    public static final double CAM_LENS_OFFSET_Y_IN = 0.0;

    /**
     * MEASURE THIS. The yaw of the camera's optical axis relative to the
     * camera turret's own forward direction, in degrees, counterclockwise
     * positive. If the camera points straight along the turret's forward
     * direction this is 0. If it is deliberately cocked to one side on its
     * bracket, put that angle here.
     */
    // Camera looks straight along the horn, not cocked on its bracket.
    public static final double CAM_MOUNT_YAW_DEG = 0.0;

    // =================================================================
    // 5. GEOMETRY - second turret
    // =================================================================

    /**
     * MEASURE THESE. Where the SECOND turret's pivot axis sits in the robot
     * frame, in inches. This turret aims from ITS OWN position. We never copy
     * the camera turret's servo position across, and we never apply a fixed
     * angular fudge for the spacing between them, because the correct offset
     * changes with distance to the target.
     */
    // MEASURED 2026-09-24: 5.5 in ahead of centre, 3.5 in to the RIGHT.
    // Both turrets sit right of the centreline, 1.5 in apart from each other.
    public static final double SECOND_PIVOT_X_IN = 5.5;
    public static final double SECOND_PIVOT_Y_IN = -3.5;

    // =================================================================
    // 6. SERVO CALIBRATION
    //
    //    goBILDA dual-mode servos: 2000-0025-0004 (Super Speed) on the
    //    camera, 2000-0025-0003 (Speed) on the second turret.
    //
    //    THESE MUST BE IN POSITIONAL MODE, NOT CONTINUOUS MODE.
    //    Choosing "Servo" rather than "Continuous Rotation" in the Robot
    //    Controller configuration does NOT reprogram the servo. The mode
    //    lives in the servo's own firmware and is changed with goBILDA's
    //    programmer or the switch on the unit. See the setup guide.
    // =================================================================

    /**
     * MEASURE THESE. The servo command position, 0..1, at which each turret
     * points along its MOUNT_ANGLE below. Find it by jogging the servo by
     * hand in the OpMode's manual mode with a bare horn fitted, until the
     * pointer aims where you want "forward" to be.
     */
    // MEASURED 2026-09-20: camera pointing straight out the robot's front.
    public static final double CAM_SERVO_FORWARD_POS    = 0.6350;
    // MEASURED 2026-09-20: horn pointing straight out the robot's front.
    public static final double SECOND_SERVO_FORWARD_POS = 0.4350;

    /**
     * MEASURE THESE. How much servo command corresponds to one degree of
     * actual turret rotation, always POSITIVE. Direction is handled
     * separately below.
     *
     * DO NOT assume 1.0 of command equals the advertised 300 degrees. The
     * usable range depends on the servo's programmed mode, the hub's pulse
     * width limits, and any gearing between servo and turret. Measure it:
     * command two positions a known distance apart, protractor the horn at
     * each, and divide.
     *
     *   posPerDeg = (pos2 - pos1) / (measuredDeg2 - measuredDeg1)
     */
    // MEASURED 2026-09-20 using the camera itself as the protractor.
    // Tag bearing went -0.6 -> +23.4 deg over +0.100 of command, so the
    // turret rotated 24.0 deg. 0.100 / 24.0 = 0.0041667, i.e. full 0-1
    // command travel spans about 240 deg.
    //
    // A hand protractor read 25 deg for the same move. We used the camera
    // because bearing is scale-invariant and we had already verified it
    // against x and y at several distances, so it resolves tenths of a degree
    // where eyeballing a protractor resolves maybe two.
    public static final double CAM_SERVO_POS_PER_DEG    = 0.0041667;
    // MEASURED 2026-09-20 by sighting a right angle against the chassis,
    // which is far easier to judge by eye than reading a protractor.
    //   forward 0.4350 -> 90 deg right at 0.7550
    //   0.3200 of command per 90 deg = 0.0035556 per deg
    //   full 0-1 travel about 281 deg
    //
    // NOTE: the camera turret measured 0.0041667, about 240 deg of full
    // travel. A 15% difference between the two. They are different goBILDA
    // models (0003 Speed here, 0004 Super Speed on the camera) so some
    // difference is expected, but it is also the size of error a mis-sighted
    // right angle would produce. Worth a cross-check once both turrets track:
    // if one consistently leads or lags the other on the same target, this
    // number is the first suspect.
    public static final double SECOND_SERVO_POS_PER_DEG = 0.0035556;

    /**
     * +1 if increasing servo command rotates the turret COUNTERCLOCKWISE
     * seen from above, -1 if it rotates it clockwise. Determine this during
     * manual calibration, not by guessing.
     */
    // MEASURED 2026-09-20: rising command swings the camera CLOCKWISE seen
    // from above, i.e. toward the robot's right, which is negative yaw.
    public static final double CAM_SERVO_DIRECTION    = -1.0;
    // MEASURED 2026-09-20: rising command swings clockwise seen from above,
    // same sense as the camera turret.
    public static final double SECOND_SERVO_DIRECTION = -1.0;

    /**
     * MEASURE THESE. Which way each turret points, in the robot frame, when
     * its servo is at its forward position. 0 means straight out the front
     * of the robot. Degrees, counterclockwise positive.
     */
    // At CAM_SERVO_FORWARD_POS the camera points straight out the robot's
    // front, which is 0 in our frame by definition.
    public static final double CAM_MOUNT_ANGLE_DEG    = 0.0;
    // At SECOND_SERVO_FORWARD_POS the horn points straight out the robot's
    // front, which is 0 in our frame by definition.
    public static final double SECOND_MOUNT_ANGLE_DEG = 0.0;

    /**
     * SAFE TRAVEL LIMITS, in servo command units, 0..1. Start these TIGHT,
     * a narrow band either side of the forward position, and widen them only
     * after you have watched the turret move through the whole range without
     * binding, hitting the other turret, or straining a wire.
     *
     * These are hard clamps. Nothing in the code commands outside them.
     */
    // CHECKED 2026-09-20: the camera turret travels the full 0.0 to 1.0
    // without binding. Pulled in slightly at both ends so we never command
    // a servo hard against its own internal stop, where it buzzes and draws
    // current for no motion.
    //
    // With forward at 0.6350 and 240 deg of full travel this allows roughly
    // 148 deg counterclockwise and 83 deg clockwise. Asymmetric, which the
    // aiming code handles: it reports LIMIT rather than trying to go further.
    //
    // WATCH THE USB CABLE. The servo is happy at both extremes; the cable may
    // not be. If it coils or tugs near either end, tighten these.
    public static final double CAM_SERVO_MIN_POS    = 0.02;
    public static final double CAM_SERVO_MAX_POS    = 0.98;
    // CHECKED 2026-09-20: travels the full 0.0 to 1.0 without binding.
    // Pulled in at both ends so we never drive the servo against its own
    // internal stop. With forward at 0.4350 and 281 deg of full travel this
    // allows roughly 117 deg counterclockwise and 153 deg clockwise.
    public static final double SECOND_SERVO_MIN_POS = 0.02;
    public static final double SECOND_SERVO_MAX_POS = 0.98;

    // =================================================================
    // 7. MOTION SHAPING
    //    These have real defaults. They are behaviour, not measurements.
    // =================================================================

    /**
     * Rate limits, degrees per second of commanded turret angle. Independent
     * per turret because the two servos are different speeds: the 0004 is
     * the Super Speed unit, the 0003 is the slower Speed unit.
     *
     * These are DELIBERATELY SLOW. This is a proof of concept. Slow motion
     * keeps the command-based angle estimate honest, because an unloaded
     * servo asked to move slowly actually tracks its command fairly well,
     * whereas one slammed across its range does not.
     */
    /**
     * WIDEN THE SERVO PULSE RANGE.
     *
     * The SDK maps command 0..1 onto 600-2400 microseconds by default. The
     * REV hub can produce 500-2500, and goBILDA servos are specified for that
     * wider range. Using it gives 2000/1800 = 11% more mechanical travel for
     * free, which on the camera turret takes roughly 240 deg up to 267 deg.
     *
     * (goBILDA's advertised 300 deg is measured at a still wider pulse range
     * than the hub can generate, so 267 is the realistic ceiling here.)
     *
     * !!! TURNING THIS ON INVALIDATES YOUR SERVO CALIBRATION !!!
     *
     * The same command now maps to a different angle, so FORWARD_POS and
     * POS_PER_DEG must both be re-measured. Predicted starting points, to be
     * confirmed by re-measuring rather than trusted:
     *
     *   POS_PER_DEG scales by 1800/2000 = 0.9
     *     camera  0.0041667 -> about 0.00375
     *     second  0.0035556 -> about 0.00320
     *
     *   FORWARD_POS moves toward the middle, because the same physical angle
     *   now sits at a smaller fraction of a wider span. Re-jog to find it.
     */
    // OFF for now. Turning it on gains about 27 deg of sweep but invalidates
    // FORWARD_POS and POS_PER_DEG on both servos, so it costs a recalibration.
    // Worth doing before the real turret is built; not worth interrupting the
    // pointing test for.
    public static final boolean USE_WIDE_PWM = false;

    public static final double PWM_MIN_US = 500;
    public static final double PWM_MAX_US = 2500;

    public static final double CAM_MAX_DEG_PER_SEC    = 90.0;
    public static final double SECOND_MAX_DEG_PER_SEC = 60.0;

    /**
     * Don't rewrite the servo for changes smaller than this. Stops the horn
     * buzzing on camera noise.
     */
    // RAISED 2026-09-24 from 0.4. At 0.4 the turrets visibly shook, chasing
    // frame-to-frame noise in the tag corners. 0.8 deg is still far finer
    // than the mechanism can honestly hold with no encoder feedback.
    public static final double CAM_DEADBAND_DEG    = 0.8;
    public static final double SECOND_DEADBAND_DEG = 0.8;

    /**
     * Low-pass on the target estimate, 0..1. 0 means "never move", 1 means
     * "jump straight to every new observation". Lower is smoother and laggier.
     */
    // LOWERED 2026-09-24 from 0.35, which let too much per-frame noise reach
    // the servos. At 0.15 each sighting nudges the stored target by 15% of
    // the disagreement, so genuine movement still gets followed within a few
    // frames at 30 fps while jitter averages out.
    //
    // If tracking now feels sluggish behind a moving tag, raise this. If it
    // shakes, lower it. This is the first dial to reach for, ahead of the
    // deadbands.
    public static final double TARGET_SMOOTHING = 0.15;

    // =================================================================
    // 8. TIMING
    // =================================================================

    /**
     * An observation older than this is not allowed to update the target at
     * all. The geometry uses the turret angle and robot pose from the moment
     * the frame was captured, and our history buffer only reaches back so far.
     */
    public static final double MAX_FRAME_AGE_MS = 300.0;

    /**
     * After losing sight of the tag we keep aiming for this long using
     * odometry alone, clearly reported as PREDICTING. After it expires we
     * hold the last commands and report LOST.
     *
     * Keep this SHORT. Prediction quality decays with odometry drift, and a
     * confidently wrong turret is worse than one that admits it is lost.
     */
    public static final double PREDICT_TIMEOUT_MS = 750.0;

    /** How long a pose stays in the capture-time lookup buffer. */
    public static final double POSE_BUFFER_MS = 1000.0;

    /**
     * Reject an observation that moves the stored target further than this
     * in one step. A genuine target is stationary; a jump this big is a
     * misread, a reflection, or a wrong-tag detection.
     */
    public static final double MAX_TARGET_JUMP_IN = 24.0;

    // =================================================================
    // 9. READINESS
    // =================================================================

    /** Values needed before automatic aiming may be enabled. */
    public static String[] missingSettings() {
        java.util.List<String> missing = new java.util.ArrayList<>();
        check(missing, "TAG_SIZE_IN", TAG_SIZE_IN);
        check(missing, "CAM_PIVOT_X_IN", CAM_PIVOT_X_IN);
        check(missing, "CAM_PIVOT_Y_IN", CAM_PIVOT_Y_IN);
        check(missing, "CAM_LENS_OFFSET_X_IN", CAM_LENS_OFFSET_X_IN);
        check(missing, "CAM_LENS_OFFSET_Y_IN", CAM_LENS_OFFSET_Y_IN);
        check(missing, "CAM_MOUNT_YAW_DEG", CAM_MOUNT_YAW_DEG);
        check(missing, "SECOND_PIVOT_X_IN", SECOND_PIVOT_X_IN);
        check(missing, "SECOND_PIVOT_Y_IN", SECOND_PIVOT_Y_IN);
        check(missing, "CAM_SERVO_FORWARD_POS", CAM_SERVO_FORWARD_POS);
        check(missing, "CAM_SERVO_POS_PER_DEG", CAM_SERVO_POS_PER_DEG);
        check(missing, "CAM_SERVO_DIRECTION", CAM_SERVO_DIRECTION);
        check(missing, "CAM_MOUNT_ANGLE_DEG", CAM_MOUNT_ANGLE_DEG);
        check(missing, "CAM_SERVO_MIN_POS", CAM_SERVO_MIN_POS);
        check(missing, "CAM_SERVO_MAX_POS", CAM_SERVO_MAX_POS);
        check(missing, "SECOND_SERVO_FORWARD_POS", SECOND_SERVO_FORWARD_POS);
        check(missing, "SECOND_SERVO_POS_PER_DEG", SECOND_SERVO_POS_PER_DEG);
        check(missing, "SECOND_SERVO_DIRECTION", SECOND_SERVO_DIRECTION);
        check(missing, "SECOND_MOUNT_ANGLE_DEG", SECOND_MOUNT_ANGLE_DEG);
        check(missing, "SECOND_SERVO_MIN_POS", SECOND_SERVO_MIN_POS);
        check(missing, "SECOND_SERVO_MAX_POS", SECOND_SERVO_MAX_POS);
        return missing.toArray(new String[0]);
    }

    /** Vision diagnostics need only the tag size. */
    public static boolean visionReady() { return isSet(TAG_SIZE_IN); }

    public static boolean aimingReady() { return missingSettings().length == 0; }

    public static boolean intrinsicsProvided() {
        return isSet(CAMERA_FX) && isSet(CAMERA_FY) && isSet(CAMERA_CX) && isSet(CAMERA_CY);
    }

    private static void check(java.util.List<String> out, String name, double v) {
        if (!isSet(v)) out.add(name);
    }
}
