package org.firstinspires.ftc.teamcode.manualturret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * MANUAL TURRET TEST
 *
 * One goBILDA 2000 Series 5-Turn Dual Mode servo (SKU 2000-0025-0503, the
 * SPEED variant) driving a goBILDA gear-driven turret through a 2.75:1
 * reduction. Left stick X slews the turret. Nothing else.
 *
 * This OpMode touches no drivetrain, no flywheel, no hood and no camera. It
 * lives in its own package so it cannot pick up constants from the dual-servo
 * aiming experiment in org.firstinspires.ftc.teamcode.turret, which is
 * different hardware with the same word in its name.
 *
 * ===================================================================
 * THE ONE THING TO UNDERSTAND BEFORE YOU RUN THIS
 * ===================================================================
 * A positional servo has no way to tell us where it is. getPosition() returns
 * the last value we wrote, not a measurement. On the first run after a power
 * cycle we have written nothing, so we know nothing.
 *
 * The consequence: the first target we send is not a small adjustment from
 * wherever the horn happens to be. It is an absolute command, and the servo
 * will travel to it at full speed, dragging 2.75:1 of geared turret with it.
 * Nothing in software can soften that, because softening it would require
 * knowing the starting point.
 *
 * That is why nothing is written to the servo until you press A, and why the
 * calibration procedure below has you find the zero with the drive gear
 * physically disengaged.
 *
 * ===================================================================
 * HOW MOVEMENT WORKS
 * ===================================================================
 * The stick sets a RATE, not a position. Each loop the commanded angle moves
 * by rate x elapsed time, and when you let go the angle stops where it is.
 * Mapping stick position straight to servo position would snap the turret back
 * to centre every time you released the stick.
 *
 * Elapsed time is clamped to MAX_LOOP_DT_S. If the loop stalls (first
 * iteration, a garbage collection, a telemetry hiccup) the raw elapsed time
 * can be large, and rate x that would be a jump rather than a slew.
 *
 * ===================================================================
 * CONTROLS - gamepad 1
 * ===================================================================
 *   A                  enable, and send the first target. Deliberate.
 *   left stick X       right moves the turret right, left moves it left.
 *                      Further over means faster. Release to hold.
 *   left bumper        hold for precision rate
 *   B                  stop updating the target
 *
 * B does NOT make the servo go limp. It keeps holding the last position it was
 * given, because that is what a servo does. B only stops this OpMode changing
 * the number.
 *
 * ===================================================================
 * CALIBRATION, THE FIRST TIME
 * ===================================================================
 * SERVO_POS_AT_TURRET_ZERO starts unset, and while it is unset this OpMode
 * runs in RAW CALIBRATION mode: the stick slews a raw servo position between
 * 0 and 1 and the telemetry prints it. Procedure:
 *
 *   1. Power off. Disengage the servo's drive gear from the turret ring, so
 *      the servo can turn freely without moving the turret. Back the motor
 *      pod off its mount, or lift the pinion out of mesh, whatever your build
 *      allows. The servo must be able to sweep its full five turns with
 *      nothing attached.
 *   2. Power on, run this OpMode, press PLAY, then press A. Telemetry says
 *      RAW CALIBRATION.
 *   3. The first A press commands RAW_START_POS, which is 0.5. This is safe
 *      ONLY because the gear is out of mesh. Do not do this step with the
 *      turret connected.
 *   4. Slew with the stick and watch the horn. Pick the position you want to
 *      call turret zero. Mid travel is the usual choice because it leaves
 *      equal rotation either way. Write the number down.
 *   5. Power off. Re-mesh the drive gear with the turret physically at the
 *      heading you want to call zero, and with the servo still at that
 *      number. The servo does not move while unpowered, so it will still be
 *      there.
 *   6. Put the number into SERVO_POS_AT_TURRET_ZERO below.
 *   7. Leave TURRET_MIN_DEG and TURRET_MAX_DEG at the deliberately small
 *      starting values until you have watched the turret reach both of them
 *      and checked cable routing, wire strain and anything the turret can
 *      hit. Widen in steps, re-checking each time.
 *
 * The servo's positional-versus-continuous mode is stored in the servo itself
 * and is changed with goBILDA's 3102 Series Dual Mode Servo Programmer. This
 * OpMode cannot read it or change it. If the turret creeps steadily instead of
 * holding a position, the servo is in continuous mode and no amount of code
 * here will fix that.
 */
@TeleOp(name = "Manual Turret Test", group = "Turret")
public class ManualTurretTest extends OpMode {

    /** Sentinel for a value that has to be measured before it can be used. */
    public static final double UNSET = Double.NaN;

    public static boolean isSet(double v) { return !Double.isNaN(v); }

    // =================================================================
    // 1. HARDWARE
    // =================================================================
    public static final String TURRET_SERVO_NAME = "turretServo";

    // =================================================================
    // 2. PWM RANGE
    // =================================================================
    /**
     * goBILDA specify 1800 degrees of travel across 500 to 2500 microseconds,
     * which is 0.90 degrees per microsecond.
     *
     * A REV hub defaults to 600 to 2400 instead. The SDK spreads servo
     * position 0.0 to 1.0 evenly across whatever range is configured, so the
     * default would give you 1800 x (1800/2000) = 1620 degrees, and every
     * position number would mean a different angle.
     *
     * WIDENING THE RANGE CHANGES THE MAPPING. If you calibrate
     * SERVO_POS_AT_TURRET_ZERO with one range and then change this flag, your
     * zero moves and your limits move with it. Change one or the other, never
     * both, and re-verify after.
     */
    public static final boolean USE_WIDE_PWM = true;
    public static final double  PWM_MIN_US   = 500.0;
    public static final double  PWM_MAX_US   = 2500.0;

    /**
     * Servo output travel, in degrees, across position 0.0 to 1.0 AT THE PWM
     * RANGE CONFIGURED ABOVE.
     *
     * 1800 with USE_WIDE_PWM true. If the wide range fails to apply, the
     * telemetry says so and this number is wrong by about 10%; the OpMode
     * reports that rather than silently carrying on.
     */
    public static final double SERVO_TRAVEL_DEG = 1800.0;

    // =================================================================
    // 3. MECHANISM
    // =================================================================
    /** Servo revolutions per turret revolution. goBILDA gear-driven turret. */
    public static final double GEAR_REDUCTION = 2.75;

    /**
     * Which way a rising servo position drives the turret.
     *
     * +1.0 means increasing servo position moves the turret in the direction
     * this OpMode calls positive, which is the driver's right.
     *
     * VERIFY THIS. goBILDA's datasheet says the output rotates clockwise as
     * the pulse width rises, but your pinion meshing the ring gear reverses
     * that, and which way the assembly faces decides the rest. Find out with
     * a small movement, not by reasoning about it.
     */
    public static final double SERVO_SIGN = 1.0;

    /**
     * The servo position at which the turret sits at zero degrees.
     *
     * UNSET until you measure it. See the calibration procedure in the class
     * comment. While this is unset the OpMode runs in RAW CALIBRATION mode.
     */
    public static final double SERVO_POS_AT_TURRET_ZERO = UNSET;

    /**
     * Hard bounds on the servo command itself, as a last line of defence
     * underneath the turret angle limits. Tighten these if you find the servo
     * has less usable travel than its full range.
     */
    public static final double SERVO_POS_MIN = 0.0;
    public static final double SERVO_POS_MAX = 1.0;

    // =================================================================
    // 4. TRAVEL LIMITS - START SMALL, WIDEN DELIBERATELY
    // =================================================================
    /**
     * Turret angle limits in TURRET degrees, measured from your zero.
     *
     * Deliberately tiny to begin with. The mechanism can physically do far
     * more: 1800 degrees of servo divided by 2.75 is about 654 turret degrees,
     * a little under two full turns. Getting there without first checking
     * where the wiring ends up is how a turret tears its own cable out.
     *
     * Widen these a bit at a time. After each widening, drive slowly to both
     * ends and watch the cable.
     */
    public static final double TURRET_MIN_DEG = -15.0;
    public static final double TURRET_MAX_DEG =  15.0;

    /**
     * The angle commanded the instant you press A.
     *
     * Zero means "go to the position you calibrated as turret zero". Change it
     * only if zero is not where you want the turret to start a match.
     */
    public static final double START_TURRET_DEG = 0.0;

    /** Raw servo position commanded by the first A press during calibration. */
    public static final double RAW_START_POS = 0.5;

    // =================================================================
    // 5. FEEL
    // =================================================================
    /**
     * How fast the COMMANDED angle changes, in turret degrees per second.
     *
     * This is a target-change rate, not a measurement of how fast the servo
     * actually moves. The servo follows at its own pace; goBILDA quote 0.09
     * s/60 degrees at 6 V unloaded, which through 2.75:1 is roughly 240 turret
     * degrees per second with nothing on it. Real loaded speed is lower. If
     * you set these rates above what the servo can deliver, the commanded
     * number runs away from the hardware and the telemetry stops describing
     * where the turret is.
     *
     * RAISED TO THE HARDWARE CEILING 2026-10-16, on request.
     *
     * Where 240 comes from: goBILDA quote 0.09 s per 60 degrees at 6 V with no
     * load, which is 667 servo degrees per second, and 667 / 2.75 is 242
     * turret degrees per second. That is the unloaded number. A turret with
     * mass on it, and a battery that sags under a flywheel, will be slower.
     *
     * The failure mode to watch for: if the commanded rate exceeds what the
     * servo can actually deliver, the commanded angle runs away from the
     * physical turret and the telemetry stops describing where the turret is.
     * You will see it as the turret still coasting into position after you let
     * go of the stick. If that happens, bring this number down until the
     * turret keeps up with the number.
     */
    public static final double NORMAL_DEG_PER_SEC    = 240.0;
    public static final double PRECISION_DEG_PER_SEC = 30.0;

    /**
     * Raw-mode slew rates, in servo position units per second.
     *
     * 0.35 crosses the servo's whole five turns in about three seconds, which
     * is roughly as fast as the servo can move. Precision stays slow because
     * raw mode is where you land on a specific number to write down.
     */
    public static final double RAW_NORMAL_POS_PER_SEC    = 0.35;
    public static final double RAW_PRECISION_POS_PER_SEC = 0.04;

    /**
     * Stick travel ignored around centre. Sticks do not rest at exactly zero,
     * and without this the turret drifts whenever nobody is touching it.
     */
    public static final double DEADBAND = 0.08;

    /**
     * Largest elapsed time a single loop may integrate over, in seconds.
     *
     * The first loop after start, and any loop that got delayed, can report a
     * long interval. Multiplying that by the slew rate would be a jump. 0.10 s
     * at 30 deg/s caps any single step at 3 turret degrees.
     */
    public static final double MAX_LOOP_DT_S = 0.10;

    // =================================================================

    private ServoImplEx servo;
    private String initError = null;
    private String pwmReport = "not attempted";

    private boolean calibrated = false;
    private boolean enabled = false;

    /** COMMANDED turret angle. Never a measurement. */
    private double commandedDeg = 0.0;
    /** COMMANDED servo position, NaN until we have written one. */
    private double commandedPos = UNSET;
    /** COMMANDED raw position, calibration mode only. */
    private double rawPos = RAW_START_POS;

    private long lastLoopNanos = 0;

    private boolean prevA, prevB;
    private String limitNote = "none";
    private String lastNote = "";

    /** Turret degrees spanned by the servo's full 0.0 to 1.0 travel. */
    private static double turretDegPerFullTravel() {
        return SERVO_TRAVEL_DEG / GEAR_REDUCTION;
    }

    // =================================================================

    @Override
    public void init() {
        // Sanity-check the constants before anything can be commanded. A bad
        // SERVO_SIGN or GEAR_REDUCTION would divide by zero further down and
        // hand the servo an Infinity, so these are gates, not warnings.
        boolean constantsSane =
                   (SERVO_SIGN == 1.0 || SERVO_SIGN == -1.0)
                && GEAR_REDUCTION > 0.0
                && SERVO_TRAVEL_DEG > 0.0
                && SERVO_POS_MIN >= 0.0 && SERVO_POS_MAX <= 1.0
                && SERVO_POS_MIN < SERVO_POS_MAX;

        if (!constantsSane) {
            initError = "constants invalid: SERVO_SIGN must be +1 or -1, "
                      + "GEAR_REDUCTION and SERVO_TRAVEL_DEG must be > 0, "
                      + "SERVO_POS_MIN/MAX must be a valid range inside 0..1";
        }

        calibrated = constantsSane
                && isSet(SERVO_POS_AT_TURRET_ZERO)
                && SERVO_POS_AT_TURRET_ZERO >= SERVO_POS_MIN
                && SERVO_POS_AT_TURRET_ZERO <= SERVO_POS_MAX
                && TURRET_MIN_DEG < TURRET_MAX_DEG;

        try {
            servo = hardwareMap.get(ServoImplEx.class, TURRET_SERVO_NAME);
            servo.setDirection(Servo.Direction.FORWARD);
        } catch (Exception e) {
            initError = "servo '" + TURRET_SERVO_NAME + "' not found: " + e.getMessage();
        }

        // Widening the pulse range does NOT move the servo. It changes what
        // future position values mean. Safe to do before anything is enabled.
        if (servo != null && USE_WIDE_PWM) {
            try {
                servo.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
                pwmReport = String.format("%.0f-%.0f us requested", PWM_MIN_US, PWM_MAX_US);
            } catch (Exception e) {
                pwmReport = "WIDE PWM REJECTED - travel is not "
                          + (int) SERVO_TRAVEL_DEG + " deg: " + e.getMessage();
            }
        } else if (servo != null) {
            pwmReport = "hub default (about 600-2400 us) - SERVO_TRAVEL_DEG must match";
        }

        commandedDeg = clamp(START_TURRET_DEG, TURRET_MIN_DEG, TURRET_MAX_DEG);

        telemetry.addLine("MANUAL TURRET TEST");
        telemetry.addLine();
        telemetry.addData("mode", calibrated ? "turret angle" : "RAW CALIBRATION");
        telemetry.addLine();
        telemetry.addLine("Nothing is written to the servo until you");
        telemetry.addLine("press A after PLAY. The first command is");
        telemetry.addLine("absolute and the servo WILL travel to it.");
        telemetry.update();
    }

    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        // Raw elapsed time, then clamped. See MAX_LOOP_DT_S.
        double rawDt = (now - lastLoopNanos) / 1e9;
        lastLoopNanos = now;
        double dt = clamp(rawDt, 0.0, MAX_LOOP_DT_S);

        readButtons();

        if (servo != null && enabled) {
            if (calibrated) updateTurret(dt);
            else            updateRaw(dt);
        }

        report(rawDt, dt);
    }

    @Override
    public void stop() {
        // Nothing to switch off. A servo keeps holding its last commanded
        // position after the OpMode ends; there is no "release" that is safe
        // to do on a geared turret without knowing what is holding it.
        enabled = false;
    }

    // =================================================================

    private void readButtons() {
        if (gamepad1.a && !prevA) {
            if (servo == null) {
                lastNote = "no servo in the configuration";
            } else if (initError != null) {
                lastNote = "refusing to enable - fix the constants first";
            } else if (!enabled) {
                enabled = true;
                if (calibrated) {
                    commandedDeg = clamp(START_TURRET_DEG, TURRET_MIN_DEG, TURRET_MAX_DEG);
                    writeAngle(commandedDeg);
                    lastNote = String.format("ENABLED - first target %.1f deg", commandedDeg);
                } else {
                    rawPos = clamp(RAW_START_POS, SERVO_POS_MIN, SERVO_POS_MAX);
                    commandedPos = rawPos;
                    servo.setPosition(rawPos);
                    lastNote = String.format("ENABLED - raw %.4f (gear must be disengaged)", rawPos);
                }
            }
        }
        prevA = gamepad1.a;

        if (gamepad1.b && !prevB && enabled) {
            enabled = false;
            lastNote = "target frozen - servo still holds its last position";
        }
        prevB = gamepad1.b;
    }

    /**
     * Stick to a signed 0..1 demand, with the deadband removed rather than
     * merely tested. Subtracting and rescaling means motion starts from zero
     * at the edge of the deadband instead of jumping to 8% rate.
     */
    private double stickDemand() {
        double x = gamepad1.left_stick_x;
        double mag = Math.abs(x);
        if (mag <= DEADBAND) return 0.0;
        double scaled = (mag - DEADBAND) / (1.0 - DEADBAND);
        return Math.signum(x) * clamp(scaled, 0.0, 1.0);
    }

    private boolean precision() { return gamepad1.left_bumper; }

    private void updateTurret(double dt) {
        double rate = (precision() ? PRECISION_DEG_PER_SEC : NORMAL_DEG_PER_SEC);
        commandedDeg += stickDemand() * rate * dt;

        // Angle limits first. No wrapping: this is a bounded multi-turn
        // mechanism, so 370 degrees is 370 degrees, not 10.
        limitNote = "none";
        if (commandedDeg <= TURRET_MIN_DEG) {
            commandedDeg = TURRET_MIN_DEG;
            limitNote = "AT MIN";
        } else if (commandedDeg >= TURRET_MAX_DEG) {
            commandedDeg = TURRET_MAX_DEG;
            limitNote = "AT MAX";
        }

        writeAngle(commandedDeg);
    }

    /**
     * Convert a turret angle to a servo position, clamp it, and write it.
     *
     * If the servo clamp bites, the commanded angle is pulled back to the
     * angle that position actually represents. Without that the angle keeps
     * integrating past what the servo can reach, and the telemetry starts
     * reporting an angle the turret is nowhere near.
     */
    private void writeAngle(double deg) {
        double pos = SERVO_POS_AT_TURRET_ZERO
                   + SERVO_SIGN * (deg / turretDegPerFullTravel());

        if (pos < SERVO_POS_MIN) {
            pos = SERVO_POS_MIN;
            commandedDeg = posToDeg(pos);
            limitNote = "SERVO AT MIN POSITION";
        } else if (pos > SERVO_POS_MAX) {
            pos = SERVO_POS_MAX;
            commandedDeg = posToDeg(pos);
            limitNote = "SERVO AT MAX POSITION";
        }

        commandedPos = pos;
        servo.setPosition(pos);
    }

    private static double posToDeg(double pos) {
        return (pos - SERVO_POS_AT_TURRET_ZERO) * turretDegPerFullTravel() / SERVO_SIGN;
    }

    private void updateRaw(double dt) {
        double rate = precision() ? RAW_PRECISION_POS_PER_SEC : RAW_NORMAL_POS_PER_SEC;
        rawPos = clamp(rawPos + stickDemand() * rate * dt, SERVO_POS_MIN, SERVO_POS_MAX);

        limitNote = "none";
        if (rawPos <= SERVO_POS_MIN) limitNote = "RAW AT MIN";
        if (rawPos >= SERVO_POS_MAX) limitNote = "RAW AT MAX";

        commandedPos = rawPos;
        servo.setPosition(rawPos);
    }

    // =================================================================

    private void report(double rawDt, double dt) {
        if (initError != null) telemetry.addData("ERROR", initError);
        if (!lastNote.isEmpty()) telemetry.addData("note", lastNote);

        telemetry.addData("state", enabled
                ? "ENABLED"
                : (servo == null ? "no servo" : "press A to enable"));
        telemetry.addData("mode", calibrated ? "TURRET ANGLE" : "RAW CALIBRATION");
        telemetry.addData("speed", precision() ? "PRECISION" : "normal");

        telemetry.addLine();
        telemetry.addData("stick X", "%.3f", gamepad1.left_stick_x);
        telemetry.addData("demand after deadband", "%.3f", stickDemand());

        telemetry.addLine();
        if (calibrated) {
            telemetry.addData("COMMANDED angle", "%.2f deg", commandedDeg);
            telemetry.addData("angle limits", "%.1f to %.1f deg", TURRET_MIN_DEG, TURRET_MAX_DEG);
            telemetry.addData("rate", "%.0f deg/s",
                    precision() ? PRECISION_DEG_PER_SEC : NORMAL_DEG_PER_SEC);
        } else {
            telemetry.addData("COMMANDED raw pos", isSet(commandedPos)
                    ? String.format("%.4f  <- record this as your zero", commandedPos)
                    : "nothing written yet");
            telemetry.addLine("SERVO_POS_AT_TURRET_ZERO is unset.");
            telemetry.addLine("DRIVE GEAR MUST BE DISENGAGED.");
        }

        telemetry.addData("COMMANDED servo pos", isSet(commandedPos)
                ? String.format("%.4f", commandedPos) : "nothing written yet");
        telemetry.addData("limit", limitNote);

        telemetry.addLine();
        telemetry.addData("PWM", pwmReport);
        telemetry.addData("servo travel", "%.0f deg over 0.0-1.0", SERVO_TRAVEL_DEG);
        telemetry.addData("reduction", "%.2f:1", GEAR_REDUCTION);
        telemetry.addData("turret span", "%.0f deg of turret over full servo travel",
                turretDegPerFullTravel());
        telemetry.addData("sign", "%+.0f", SERVO_SIGN);

        telemetry.addLine();
        telemetry.addData("loop dt", "%.1f ms%s", dt * 1000.0,
                rawDt > MAX_LOOP_DT_S ? String.format("  (clamped from %.0f)", rawDt * 1000.0) : "");

        telemetry.addLine();
        telemetry.addLine("Angles and positions above are COMMANDED.");
        telemetry.addLine("This servo cannot report where it actually is.");

        telemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
