package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * One positional-servo turret.
 *
 * =====================================================================
 * THERE IS NO ANGLE FEEDBACK HERE. READ THIS.
 * =====================================================================
 * Servo.getPosition() in the FTC SDK returns THE LAST VALUE YOU WROTE. It is
 * not a measurement. It does not know if the horn is stalled, blocked, still
 * travelling, or has been knocked off its spline. Nothing in this class is
 * encoder feedback and nothing in it verifies physical alignment.
 *
 * What we have instead is a COMMAND-BASED ESTIMATE: we rate-limit our own
 * commands to a speed we believe the servo can keep up with, and then assume
 * the horn is roughly where we told it to be. That assumption is only
 * defensible because this test moves deliberately slowly with a bare horn and
 * no load. Under load, at speed, or against a hard stop it will be wrong, and
 * the aim will be wrong with it.
 *
 * The estimate is kept in a timestamped ring buffer, because a camera frame
 * is always a little bit old: when a frame arrives we need to know where this
 * turret was WHEN THE SHUTTER FIRED, not where it is now. That is the whole
 * reason for the buffer.
 *
 * Treat everything this class reports as "where we asked it to be", and
 * confirm real alignment with your eyes and a protractor.
 */
public class ServoTurret {

    /** Where the aim ended up relative to what was asked for. */
    public enum LimitState { OK, AT_MIN, AT_MAX }

    private final String name;
    private final Servo servo;

    // Calibration, all from TurretConfig.
    private final double forwardPos;
    private final double posPerDeg;
    private final double direction;
    private final double mountAngleRad;
    private final double minPos, maxPos;
    private final double maxRadPerSec;
    private final double deadbandRad;

    // Command-based state.
    private double commandedYaw;          // radians, robot frame
    private double commandedPos;
    private long   lastUpdateNanos;
    private LimitState limitState = LimitState.OK;
    private double lastRequestedYaw;
    private boolean everCommanded = false;

    // Timestamped estimate history, for capture-time lookup.
    private static final int HISTORY = 64;
    private final long[]   histNanos = new long[HISTORY];
    private final double[] histYaw   = new double[HISTORY];
    private int histCount = 0, histHead = 0;

    public ServoTurret(HardwareMap hw,
                       String hardwareName,
                       double forwardPos,
                       double posPerDeg,
                       double direction,
                       double mountAngleDeg,
                       double minPos,
                       double maxPos,
                       double maxDegPerSec,
                       double deadbandDeg) {
        this.name          = hardwareName;
        this.servo         = applyPwmRange(hw, hardwareName);
        this.forwardPos    = forwardPos;
        this.posPerDeg     = posPerDeg;
        this.direction     = direction;
        this.mountAngleRad = Math.toRadians(mountAngleDeg);
        this.minPos        = Math.min(minPos, maxPos);
        this.maxPos        = Math.max(minPos, maxPos);
        this.maxRadPerSec  = Math.toRadians(maxDegPerSec);
        this.deadbandRad   = Math.toRadians(deadbandDeg);
    }

    public String getName() { return name; }

    /**
     * Fetch the servo, widening its pulse range if configured.
     *
     * ServoImplEx is the same physical device as Servo with extra controls
     * exposed. If the cast fails for any reason we fall back to a plain Servo
     * rather than refusing to run, because a turret with default travel still
     * works; it just cannot reach as far.
     */
    static Servo applyPwmRange(HardwareMap hw, String name) {
        if (!TurretConfig.USE_WIDE_PWM) return hw.get(Servo.class, name);
        try {
            com.qualcomm.robotcore.hardware.ServoImplEx ex =
                    hw.get(com.qualcomm.robotcore.hardware.ServoImplEx.class, name);
            ex.setPwmRange(new com.qualcomm.robotcore.hardware.PwmControl.PwmRange(
                    TurretConfig.PWM_MIN_US, TurretConfig.PWM_MAX_US));
            return ex;
        } catch (Exception e) {
            return hw.get(Servo.class, name);
        }
    }

    // -----------------------------------------------------------------
    // Command mapping
    // -----------------------------------------------------------------

    /**
     * Robot-frame yaw -> servo command position.
     *
     * The turret sits at mountAngle when the servo is at forwardPos, and
     * moves posPerDeg of command per degree, in whichever rotational sense
     * `direction` says.
     *
     * Angle wrapping: we take the SHORTEST angular route from the mount angle
     * to the requested yaw. A limited-travel servo cannot spin the long way
     * round, so expressing the request as a delta from the mount angle is the
     * only representation that makes sense. If that delta is still outside
     * the travel limits, the clamp below catches it and we report LIMIT.
     */
    public double yawToPos(double yawRad) {
        double deltaDeg = Math.toDegrees(Geometry.angleDelta(mountAngleRad, yawRad));
        return forwardPos + direction * posPerDeg * deltaDeg;
    }

    /** Inverse of yawToPos, for reporting what a manual command corresponds to. */
    public double posToYaw(double pos) {
        double deltaDeg = (pos - forwardPos) / (direction * posPerDeg);
        return Geometry.normalize(mountAngleRad + Math.toRadians(deltaDeg));
    }

    // -----------------------------------------------------------------
    // Automatic aiming
    // -----------------------------------------------------------------

    /**
     * Ask the turret to point at a robot-frame yaw. Rate limited, deadbanded,
     * and clamped to the travel limits.
     *
     * @return the limit state after this command
     */
    public LimitState aimAt(double desiredYaw, long nowNanos) {
        lastRequestedYaw = desiredYaw;

        if (!everCommanded) {
            // We have no idea where the horn is until something commands it.
            // Do not invent a starting angle and do not sweep to find one.
            // The OpMode must send the turret to its forward position first.
            return limitState;
        }

        double dt = (nowNanos - lastUpdateNanos) / 1e9;
        if (dt <= 0 || dt > 0.5) dt = 0.02;   // first call, or a stalled loop

        double error = Geometry.angleDelta(commandedYaw, desiredYaw);

        if (Math.abs(error) < deadbandRad) {
            recordHistory(nowNanos);
            lastUpdateNanos = nowNanos;
            return limitState;
        }

        double maxStep = maxRadPerSec * dt;
        double step = Math.max(-maxStep, Math.min(maxStep, error));
        double target = Geometry.normalize(commandedYaw + step);

        writeYaw(target, nowNanos);
        return limitState;
    }

    /** Command a yaw directly, still clamped. Used for calibration moves. */
    public void writeYaw(double yawRad, long nowNanos) {
        double pos = yawToPos(yawRad);

        limitState = LimitState.OK;
        if (pos < minPos) { pos = minPos; limitState = LimitState.AT_MIN; }
        if (pos > maxPos) { pos = maxPos; limitState = LimitState.AT_MAX; }

        commandedPos = pos;
        commandedYaw = posToYaw(pos);   // what the clamp actually leaves us at
        servo.setPosition(pos);
        everCommanded = true;
        lastUpdateNanos = nowNanos;
        recordHistory(nowNanos);
    }

    /** Send the turret to its calibrated forward position. */
    public void goToForward(long nowNanos) {
        writeYaw(mountAngleRad, nowNanos);
    }

    /** Small manual step, in servo command units, for calibration. */
    public void jogPosition(double deltaPos, long nowNanos) {
        double base = everCommanded ? commandedPos : forwardPos;
        double pos = Math.max(minPos, Math.min(maxPos, base + deltaPos));
        commandedPos = pos;
        commandedYaw = posToYaw(pos);
        servo.setPosition(pos);
        everCommanded = true;
        lastUpdateNanos = nowNanos;
        recordHistory(nowNanos);
    }

    /** Stop updating. Leaves the servo holding whatever it was last sent. */
    public void hold() { /* deliberately nothing: no new setPosition call */ }

    // -----------------------------------------------------------------
    // Estimate history
    // -----------------------------------------------------------------

    private void recordHistory(long nanos) {
        histNanos[histHead] = nanos;
        histYaw[histHead]   = commandedYaw;
        histHead = (histHead + 1) % HISTORY;
        if (histCount < HISTORY) histCount++;
    }

    /**
     * Our best estimate of this turret's angle at a past instant.
     *
     * Again: this is the command history, not a measurement. Returns NaN if
     * we have nothing at all, so callers can refuse to use the observation
     * rather than silently substituting the current angle.
     */
    public double estimatedYawAt(long nanos) {
        if (histCount == 0) return Double.NaN;

        long bestDiff = Long.MAX_VALUE;
        double best = Double.NaN;
        for (int i = 0; i < histCount; i++) {
            int idx = (histHead - 1 - i + HISTORY * 2) % HISTORY;
            long d = Math.abs(histNanos[idx] - nanos);
            if (d < bestDiff) { bestDiff = d; best = histYaw[idx]; }
        }
        return best;
    }

    // -----------------------------------------------------------------
    // Reporting
    // -----------------------------------------------------------------

    public boolean    hasBeenCommanded() { return everCommanded; }
    public double     getCommandedYaw()  { return commandedYaw; }
    public double     getRequestedYaw()  { return lastRequestedYaw; }
    public double     getCommandedPos()  { return commandedPos; }
    public LimitState getLimitState()    { return limitState; }
    public double     getMountAngleRad() { return mountAngleRad; }

    /** How far the clamp forced us away from what was asked for, in degrees. */
    public double getLimitShortfallDeg() {
        if (limitState == LimitState.OK || !everCommanded) return 0;
        return Math.toDegrees(Geometry.angleDelta(commandedYaw, lastRequestedYaw));
    }
}
