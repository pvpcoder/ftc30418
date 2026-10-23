package org.firstinspires.ftc.teamcode.turret;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * One webcam, one AprilTag pipeline, one tag ID.
 *
 * SDK 12 NOTE
 * -----------
 * In SDK 12 AprilTagDetection became a base class with two subclasses:
 * AprilTagSingleDetection (one physical tag) and AprilTagClusterDetection
 * (several tags treated as one rigid group). The per-tag fields - id,
 * metadata, center - moved onto AprilTagSingleDetection. ftcPose stayed on
 * the base class.
 *
 * We only ever want single tags, so every detection is tested with instanceof
 * and anything that is not a single tag is skipped. We never build a cluster
 * library, so clusters should not appear at all; the check is there so a
 * future change cannot silently feed us one.
 *
 * Deliberately does NOT use the season field database. We build a two-entry
 * tag library from the size we physically measured, with no field position
 * attached, because the printout lives wherever we put it. Because our library
 * entries have no field pose, detection.robotPose is null. That is correct and
 * expected: we use ftcPose, which needs only tag size and lens intrinsics, and
 * work out the field position ourselves from our own odometry.
 */
public class TagTracker {

    /** One accepted sighting, with the timestamp that makes it usable. */
    public static final class Observation {
        public final long   captureNanos;
        public final double xInches;     // SDK frame: right of optical axis
        public final double yInches;     // SDK frame: forward along axis
        public final double range;
        public final double bearingDeg;
        public final double decisionMargin;

        Observation(long captureNanos, double x, double y,
                    double range, double bearingDeg, double margin) {
            this.captureNanos = captureNanos;
            this.xInches = x;
            this.yInches = y;
            this.range = range;
            this.bearingDeg = bearingDeg;
            this.decisionMargin = margin;
        }
    }

    /** Why the last frame was thrown away, for telemetry. */
    public enum Reject {
        NONE, NO_DETECTION, WRONG_ID, NO_POSE, LOW_MARGIN, TOO_FAR,
        NO_TIMESTAMP, STALE, ALREADY_SEEN
    }

    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    private long lastAcceptedCaptureNanos = 0;
    private Reject lastReject = Reject.NO_DETECTION;
    private int lastSeenId = -1;
    private int detectionCount = 0;
    private boolean secondaryVisible = false;
    private String visibleIds = "";

    public TagTracker(HardwareMap hw) {
        // Our two measured tags, at their measured size, with NO field
        // position. Both 30 and 31 are present; only 30 is ever aimed at.
        // Having 31 in the library means a wrong-tag sighting reports
        // WRONG_ID and names what it saw, rather than looking identical to a
        // blind camera.
        AprilTagLibrary library = new AprilTagLibrary.Builder()
                .addTag(TurretConfig.TARGET_TAG_ID,
                        "target-" + TurretConfig.TARGET_TAG_ID,
                        TurretConfig.TAG_SIZE_IN,
                        DistanceUnit.INCH)
                .addTag(TurretConfig.SECONDARY_TAG_ID,
                        "neighbour-" + TurretConfig.SECONDARY_TAG_ID,
                        TurretConfig.TAG_SIZE_IN,
                        DistanceUnit.INCH)
                .build();

        AprilTagProcessor.Builder pb = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true);

        // Only override the SDK's own calibration logic if we actually have
        // numbers for THIS camera. See the long note in TurretConfig.
        if (TurretConfig.intrinsicsProvided()) {
            pb.setLensIntrinsics(TurretConfig.CAMERA_FX, TurretConfig.CAMERA_FY,
                                 TurretConfig.CAMERA_CX, TurretConfig.CAMERA_CY);
        }

        processor = pb.build();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, TurretConfig.WEBCAM_NAME))
                .setCameraResolution(new Size(TurretConfig.CAMERA_WIDTH, TurretConfig.CAMERA_HEIGHT))
                .addProcessor(processor)
                .build();
    }

    // -----------------------------------------------------------------
    // Focus lock
    // -----------------------------------------------------------------

    private String focusReport = "not attempted";
    private boolean focusLocked = false;
    private double lockedFocusLength = Double.NaN;

    /**
     * Freeze the lens.
     *
     * Camera controls do not exist until the portal is STREAMING, so call this
     * every loop; it becomes a no-op once it succeeds.
     *
     * Why it matters: AprilTag range comes from the apparent size of the tag
     * divided by the lens's focal length. An autofocus lens changes its focal
     * length every time it refocuses. We measured this directly - the same
     * 24 in spot read 24.1 and then 27.5 after a trip to 48 in and back.
     */
    public void lockFocusIfNeeded() {
        if (focusLocked || !TurretConfig.LOCK_FOCUS) return;
        if (!isStreaming()) { focusReport = "waiting for stream"; return; }

        try {
            FocusControl fc = portal.getCameraControl(FocusControl.class);
            if (fc == null) {
                focusReport = "camera exposes no focus control";
                focusLocked = true;
                return;
            }

            if (!fc.isModeSupported(FocusControl.Mode.Fixed)) {
                if (fc.isModeSupported(FocusControl.Mode.Infinity)) {
                    fc.setMode(FocusControl.Mode.Infinity);
                    focusReport = "LOCKED at Infinity (Fixed unsupported)";
                } else {
                    focusReport = "CANNOT LOCK - mode " + fc.getMode() + ", ranges will drift";
                }
                focusLocked = true;
                return;
            }

            double target = TurretConfig.isSet(TurretConfig.FIXED_FOCUS_LENGTH)
                    ? TurretConfig.FIXED_FOCUS_LENGTH
                    : fc.getFocusLength();

            // A camera that does not report focus length returns -1. Setting
            // the lens to a focus length of -1 is meaningless, and we saw
            // exactly that: "LOCKED Fixed @ -1.0" while ranges kept drifting.
            if (target < 0 || Double.isNaN(target)) {
                if (fc.isModeSupported(FocusControl.Mode.Infinity)) {
                    fc.setMode(FocusControl.Mode.Infinity);
                    focusReport = "LOCKED at Infinity (camera reports no focus length)";
                } else {
                    fc.setMode(FocusControl.Mode.Fixed);
                    focusReport = "Fixed mode set, but camera reports no focus"
                                + " length - lock is unverified";
                }
                focusLocked = true;
                return;
            }

            fc.setMode(FocusControl.Mode.Fixed);
            fc.setFocusLength(target);

            lockedFocusLength = fc.getFocusLength();
            focusReport = String.format("LOCKED Fixed @ %.1f", lockedFocusLength);
            focusLocked = true;

        } catch (Exception e) {
            focusReport = "focus lock failed: " + e.getMessage();
            focusLocked = true;   // don't retry forever
        }
    }

    public String getFocusReport()       { return focusReport; }
    public double getLockedFocusLength() { return lockedFocusLength; }

    // -----------------------------------------------------------------
    // Polling
    // -----------------------------------------------------------------

    /**
     * Look for a fresh, trustworthy sighting of our tag.
     *
     * Returns null and sets a reject reason for anything we will not use.
     * Each captured frame is consumed at most once: we compare the frame's
     * acquisition timestamp against the last accepted one, so re-reading the
     * same detection on a later loop does not double-count.
     *
     * @param nowNanos System.nanoTime() from the caller's loop
     */
    public Observation poll(long nowNanos) {
        List<AprilTagDetection> detections = processor.getDetections();
        detectionCount = detections == null ? 0 : detections.size();

        if (detections == null || detections.isEmpty()) {
            lastReject = Reject.NO_DETECTION;
            return null;
        }

        AprilTagSingleDetection match = null;
        secondaryVisible = false;
        StringBuilder ids = new StringBuilder();

        for (AprilTagDetection d : detections) {
            // SDK 12: only single tags carry an id. Clusters are skipped.
            if (!(d instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection s = (AprilTagSingleDetection) d;

            lastSeenId = s.id;
            if (ids.length() > 0) ids.append(", ");
            ids.append(s.id);

            if (s.id == TurretConfig.SECONDARY_TAG_ID) secondaryVisible = true;
            if (s.id == TurretConfig.TARGET_TAG_ID && match == null) match = s;
        }
        visibleIds = ids.toString();

        if (match == null) { lastReject = Reject.WRONG_ID; return null; }
        if (match.ftcPose == null) { lastReject = Reject.NO_POSE; return null; }

        if (match.decisionMargin < TurretConfig.MIN_DECISION_MARGIN) {
            lastReject = Reject.LOW_MARGIN;
            return null;
        }

        if (match.ftcPose.range > TurretConfig.MAX_ACCEPT_RANGE_IN) {
            lastReject = Reject.TOO_FAR;
            return null;
        }

        // Timestamp sanity. frameAcquisitionNanoTime comes from the same
        // System.nanoTime() clock we use, but check rather than assume: an
        // observation we cannot place in time is useless on a moving robot.
        long capture = match.frameAcquisitionNanoTime;
        if (capture <= 0) { lastReject = Reject.NO_TIMESTAMP; return null; }

        long ageNanos = nowNanos - capture;
        if (ageNanos < 0) { lastReject = Reject.NO_TIMESTAMP; return null; }
        if (ageNanos > (long) (TurretConfig.MAX_FRAME_AGE_MS * 1e6)) {
            lastReject = Reject.STALE;
            return null;
        }

        if (capture == lastAcceptedCaptureNanos) {
            lastReject = Reject.ALREADY_SEEN;
            return null;
        }

        lastAcceptedCaptureNanos = capture;
        lastReject = Reject.NONE;

        // Apply the empirical focal-length correction. Scaling x, y and range
        // together preserves bearing exactly, which is what we want: the
        // bearing was already correct and must not be disturbed.
        double k = TurretConfig.RANGE_SCALE;

        return new Observation(capture,
                match.ftcPose.x * k, match.ftcPose.y * k,
                match.ftcPose.range * k, match.ftcPose.bearing,
                match.decisionMargin);
    }

    // -----------------------------------------------------------------
    // Status
    // -----------------------------------------------------------------

    public VisionPortal.CameraState getCameraState() { return portal.getCameraState(); }

    public boolean isStreaming() {
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public float   getFps()             { return portal.getFps(); }
    public Reject  getLastReject()      { return lastReject; }
    public int     getLastSeenId()      { return lastSeenId; }
    public int     getDetectionCount()  { return detectionCount; }

    /** Comma-separated IDs in the most recent frame, for telemetry. */
    public String  getVisibleIds()      { return visibleIds.isEmpty() ? "none" : visibleIds; }

    /** True when the neighbour tag is in frame. Never aimed at. */
    public boolean isSecondaryVisible() { return secondaryVisible; }

    /** Always call this from the OpMode's stop(). */
    public void close() {
        try { portal.close(); } catch (Exception ignored) { }
    }
}
