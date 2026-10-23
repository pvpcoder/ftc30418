package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * SHOOTER BENCH TEST
 *
 * One flywheel motor, one positional hood servo. Nothing else. This OpMode
 * does not drive the robot and does not touch MecanumTeleOp, the Pedro
 * autonomous OpModes, or the turret package.
 *
 * ===================================================================
 * READ THIS BEFORE THE FIRST RUN
 * ===================================================================
 * A flywheel stores a lot of energy and does not care what is in front of it.
 * Run this with the robot on a stand, nothing loaded, and nobody reaching
 * across the wheel.
 *
 * The hood starts UNCALIBRATED on purpose. HOOD_MIN, HOOD_MAX and HOOD_START
 * are NaN until you measure them, and nothing is ever written to the servo
 * until you press a button. A servo told to go to an unverified position will
 * happily drive a linkage into its stop and stall there, which is how servos
 * and linkages get destroyed. See the calibration procedure below.
 *
 * ===================================================================
 * CONTROLS - gamepad 2 (gamepad 1 also works, so one controller is enough)
 * ===================================================================
 *   right trigger      HOLD to spin the flywheel. Release to stop.
 *   dpad up / down     target +/- one step, hold to repeat
 *   dpad right / left  hood +/- one small step, hold to repeat once
 *                      HOOD_MIN/MAX are set (single press before that)
 *   B                  send the hood to HOOD_START (only once calibrated)
 *   back               reset the target to zero
 *
 * A tap is always exactly one step. Holding pauses for HOLD_DELAY_MS and then
 * repeats, except on the hood while it is still uncalibrated, where there are
 * no software limits yet and a held button would walk the horn into whatever
 * the real mechanical stop turns out to be.
 *
 * ===================================================================
 * TWO MODES, AND WHY
 * ===================================================================
 * RPM mode needs two numbers this code refuses to guess: how many encoder
 * ticks the motor reports per revolution, and the ratio between the motor and
 * the flywheel. Fill those in and you get true closed-loop velocity control
 * through DcMotorEx.setVelocity().
 *
 * Leave them blank and the OpMode runs in POWER mode instead: dpad up/down
 * change motor power, bounded by MAX_TEST_POWER, and the telemetry says
 * "POWER" everywhere it would otherwise say RPM. Power is not RPM, it is not
 * proportional to RPM, and it sags with battery voltage. The two are never
 * mixed in the display.
 *
 * There is also a runtime check. If the constants are filled in but the
 * encoder is not actually plugged in, RUN_USING_ENCODER sees zero velocity,
 * concludes it is infinitely far from target and winds up to full power. That
 * is a real way to destroy a shooter. So the first time the flywheel is
 * commanded, this OpMode watches for encoder counts; if none arrive within
 * ENCODER_PROOF_MS it cuts the motor, drops to POWER mode and says so.
 */
@TeleOp(name = "Shooter Test", group = "Shooter")
public class ShooterTestTeleOp extends OpMode {

    /**
     * Sentinel for "you have not measured this yet".
     *
     * Declared first because Java forbids a static initialiser from referring
     * to a static field declared further down the file.
     */
    public static final double UNSET = Double.NaN;

    public static boolean isSet(double v) { return !Double.isNaN(v); }

    // =================================================================
    // 1. HARDWARE NAMES
    //    These must match the Robot Configuration on the Driver Station
    //    exactly, including capitalisation. Nothing in this project
    //    currently configures a shooter, so both of these are new entries
    //    you need to add. See the handoff notes at the bottom of this file.
    // =================================================================
    public static final String FLYWHEEL_NAME = "flywheel";   // Motor port
    public static final String HOOD_NAME     = "hood";       // Servo port

    /**
     * Flip if the flywheel spins the wrong way.
     * Set true 2026-10-12: it ran backwards on the first spin test.
     *
     * This reverses the encoder sense along with the motor, so measured RPM
     * stays positive. Do not "fix" a backwards motor by commanding negative
     * velocity instead; that leaves the sign conventions fighting each other
     * the moment anything else reads this motor.
     */
    public static final boolean FLYWHEEL_REVERSED = true;

    // =================================================================
    // 2. FLYWHEEL GEARING - FILL THESE IN FOR RPM CONTROL
    // =================================================================
    /**
     * Encoder ticks per ONE REVOLUTION OF THE MOTOR'S OUTPUT SHAFT.
     *
     * Take this from the motor manufacturer's spec sheet, not from a guess.
     * It already includes the internal gearbox: a goBILDA 5202 series motor
     * reports a different number for every gear ratio they sell, and two
     * motors that look identical can differ by 20x. A bare REV HD Hex with no
     * gearbox is not the same as the same motor with a 20:1 cartridge.
     *
     * OURS: goBILDA 5203 series Yellow Jacket, 1:1 ratio, part 5203-2402-0001.
     * goBILDA publish "28 PPR at the output shaft" for this one, and because
     * the gearbox is 1:1 the output shaft and the motor shaft are the same
     * thing. 28 is what the hub counts per revolution.
     *
     * Be aware this is a COARSE encoder. The 19.2:1 version of the same motor
     * reports 537.7 ticks per revolution; ours reports 28, because the
     * gearbox that multiplies the count is not there. Velocity readings will
     * be steppier than you would see on a geared motor. Fine for holding one
     * flywheel speed, not precise enough for fine position work.
     */
    public static final double TICKS_PER_MOTOR_REV = 28.0;

    /**
     * Flywheel revolutions per ONE motor output-shaft revolution.
     *
     *   direct drive, motor shaft bolted to the wheel      -> 1.0
     *   belt, 24 tooth motor pulley -> 16 tooth wheel      -> 24.0 / 16.0
     *   gears, 30 tooth on motor -> 60 tooth on wheel      -> 30.0 / 60.0
     *
     * Count the teeth. Do not estimate from pulley diameter.
     *
     * OURS: 1.0. The flywheel is mounted straight onto the motor's output
     * shaft, so flywheel RPM and motor RPM are the same number.
     */
    public static final double FLYWHEEL_REVS_PER_MOTOR_REV = 1.0;

    /**
     * Motor free speed in RPM at the output shaft, from the spec sheet.
     * Optional. If you fill it in, the telemetry warns when you ask for a
     * target the motor physically cannot hold.
     *
     * OURS: 6000 RPM no-load at 12 V, from goBILDA's spec sheet. That is
     * no-load. A flywheel with air resistance and a belt of friction on it
     * will top out meaningfully below that, and the number drops as the
     * battery sags, so treat 6000 as a ceiling you never reach rather than a
     * target.
     */
    public static final double MOTOR_FREE_SPEED_RPM = 6000.0;

    // =================================================================
    // 3. TARGETS AND LIMITS
    // =================================================================
    public static final double RPM_STEP       = 100.0;
    /**
     * Raised to 6000 on request, 2026-10-12.
     *
     * 6000 is the motor's NO-LOAD free speed. With a flywheel on the shaft and
     * a battery under load you will not reach it, so expect the top of the
     * range to be a target the loop chases without arriving. The telemetry
     * warns above 5400 for that reason; the warning is information, not a
     * block.
     */
    public static final double MAX_TARGET_RPM = 6000.0;

    /** Used only in POWER mode. Deliberately well short of 1.0. */
    public static final double POWER_STEP     = 0.05;
    public static final double MAX_TEST_POWER = 0.60;

    /** Trigger must pass this before the motor runs at all. */
    public static final double TRIGGER_THRESHOLD = 0.15;

    // ---- hold-to-repeat -------------------------------------------
    /**
     * How long a dpad button must be held before it starts repeating, in ms.
     *
     * Long enough that a deliberate single tap is still a single step. Below
     * about 300 ms you start getting two steps when you meant one.
     */
    public static final double HOLD_DELAY_MS = 400.0;

    /** Repeat interval for the target buttons once repeating starts, ms. */
    public static final double REPEAT_RPM_MS = 50.0;

    /**
     * Repeat interval for the hood, ms.
     *
     * At 50 ms and a 0.005 step that is 0.1 of servo travel per second, so the
     * 0.265 usable range crosses in about 2.7 seconds.
     *
     * Past roughly this rate the software stops being what limits you. The
     * servo has its own slew speed, quoted on its datasheet as seconds per 60
     * degrees, and once the commands arrive faster than the horn can follow it
     * simply moves continuously at its own top speed and lags the commanded
     * number. No amount of shortening this interval makes the hardware move
     * faster. If the hood still feels slow after this, the fix is a faster
     * servo or a shorter lever arm, not a smaller number here.
     */
    public static final double REPEAT_HOOD_MS = 50.0;

    /**
     * How long to wait for the encoder to prove it exists, in milliseconds,
     * once the flywheel has been commanded. Long enough that a heavy wheel has
     * begun to turn, short enough that a dead encoder cannot wind the
     * controller up to full power.
     */
    public static final double ENCODER_PROOF_MS = 1000.0;

    // =================================================================
    // 4. HOOD LIMITS - CALIBRATE BEFORE ATTACHING THE LINKAGE
    // =================================================================
    /**
     * Normalised servo positions, 0.0 to 1.0.
     *
     * These are NaN, not 0.0 and 1.0, because 0.0 and 1.0 would be a guess
     * about your mechanism and a wrong guess drives the hood into its stop.
     * With them unset the OpMode runs in HOOD CALIBRATION mode, which jogs the
     * raw servo and prints the number to paste back in here.
     *
     * CALIBRATION PROCEDURE, linkage DISCONNECTED from the servo horn:
     *   1. Run this OpMode. Telemetry will say HOOD CALIBRATION.
     *   2. Reconnect the linkage only after step 6.
     *   3. Press dpad right once. The horn moves 0.005. Watch it.
     *   4. Keep single-pressing toward one end. The instant the horn reaches
     *      the end of the SAFE travel for your mechanism, stop. Write that
     *      number down.
     *   5. Single-press the other way to find the other end. Write it down.
     *   6. Back each one off by about 0.02 as a margin, then put the smaller
     *      number in HOOD_MIN and the larger in HOOD_MAX.
     *   7. Pick a mid-travel value for HOOD_START.
     *   8. Re-deploy. Reattach the linkage. Press B and confirm the hood goes
     *      where you expect before touching anything else.
     *
     * Never hold the dpad to get there faster. Single presses only, which is
     * why the buttons are edge detected.
     */
    /*
     * MEASURED 2026-10-12, with the linkage attached.
     *
     * Direction, because "MIN" and "MAX" say nothing about the hood itself:
     *   LOWER servo number  = hood UP     (d-pad left)
     *   HIGHER servo number = hood DOWN   (d-pad right)
     *
     * Hard limits found by single-stepping until the hood stopped moving:
     *   0.2100  hood fully up
     *   0.5150  hood fully down
     *
     * Each is pulled in by 0.02 below. That margin is what keeps the servo
     * off its mechanical stop; a servo commanded past the stop cannot reach
     * the position, so it sits there at full torque drawing stall current
     * until something gives.
     *
     * 0.265 of usable travel is left, which is 53 fine steps end to end.
     */
    /*
     * HOOD REBUILT 2026-10-16. HOOD_MAX set to 0.5700 on request.
     *
     * The original calibration on 2026-10-12 found 0.2100 up and 0.5150 down,
     * and those were set to 0.2300 / 0.4950 with a 0.02 margin. That hood has
     * since been replaced, so the down-end measurement describes a part that
     * no longer exists. 0.5700 is a stated value for the new one, not a
     * measured one.
     *
     * HOOD_MIN is still the OLD measurement. It has not been re-checked
     * against the new hood either. It is the conservative direction, so it is
     * unlikely to damage anything, but it may now be stopping the hood short
     * of how far up the new mechanism can actually go.
     *
     * When there is time, recalibrate properly: blank all three values back to
     * UNSET, which puts the OpMode into HOOD CALIBRATION mode, and walk both
     * ends again. Until then, ease into the bottom the first time and listen
     * for the servo straining. A positional servo reports no load and no
     * position, so nothing in the code can catch a stall for you.
     */
    public static final double HOOD_MIN   = 0.2300;   // OLD HOOD measurement, unverified
    public static final double HOOD_MAX   = 0.5700;   // new hood, stated not measured
    public static final double HOOD_START = 0.3625;   // mid travel

    /** One dpad press of hood movement. */
    public static final double HOOD_STEP = 0.005;

    /**
     * Coarse jog multiplier, available ONLY while the hood is uncalibrated.
     *
     * 0.005 steps mean up to 200 presses to cross the full servo range, which
     * is enough tedium that people start holding the button down, which is the
     * exact thing the fine step exists to prevent. Holding the left bumper
     * makes each press 10x bigger so you can get into the neighbourhood, then
     * you release it and creep up on the limit properly.
     *
     * Deliberately dead once HOOD_MIN/MAX are filled in: by then the linkage
     * is attached and a 0.05 jump is a thing you do to a servo horn once.
     */
    public static final double HOOD_COARSE_MULTIPLIER = 10.0;

    // =================================================================
    // 5. OPTIONAL VELOCITY PIDF
    //    Leave unset to use the motor controller's defaults. The defaults are
    //    usually acceptable for a flywheel that only has to hold one speed;
    //    tune these only if the measured RPM will not settle.
    // =================================================================
    public static final double VEL_P = UNSET;
    public static final double VEL_I = UNSET;
    public static final double VEL_D = UNSET;
    public static final double VEL_F = UNSET;

    // =================================================================

    private enum Mode { RPM, POWER }

    private DcMotorEx flywheel;
    private Servo hood;

    private Mode mode = Mode.POWER;
    private String modeReason = "";
    private String initError = null;

    private boolean hoodCalibrated = false;
    private String hoodError = null;

    /** Target in RPM (RPM mode) or in motor power 0..1 (POWER mode). */
    private double targetRpm = 0.0;
    private double targetPower = 0.0;

    /** Commanded hood position. NaN until we have actually written one. */
    private double hoodPos = UNSET;

    private boolean running = false;

    // Encoder proof-of-life.
    private boolean encoderProven = false;
    private long    commandStartNanos = 0;
    private String  encoderNote = "not tested yet - hold the trigger";

    // Edge detection for the one-shot buttons.
    private boolean prevLeft, prevRight, prevB, prevBack;

    /**
     * Press-and-hold auto-repeat.
     *
     * Fires once the moment the button goes down, then goes quiet for
     * HOLD_DELAY_MS, then fires every intervalMs for as long as it is held.
     * The quiet gap is what keeps a quick tap to exactly one step.
     */
    private static final class Repeater {
        private final double delayMs, intervalMs;
        private boolean down;
        private long pressedAt, lastFire;

        Repeater(double delayMs, double intervalMs) {
            this.delayMs = delayMs;
            this.intervalMs = intervalMs;
        }

        boolean fired(boolean pressed, long now) {
            if (!pressed) { down = false; return false; }

            if (!down) {               // the initial press
                down = true;
                pressedAt = now;
                lastFire = now;
                return true;
            }

            if ((now - pressedAt) / 1e6 < delayMs) return false;

            if ((now - lastFire) / 1e6 >= intervalMs) {
                lastFire = now;
                return true;
            }
            return false;
        }
    }

    private final Repeater repUp    = new Repeater(HOLD_DELAY_MS, REPEAT_RPM_MS);
    private final Repeater repDown  = new Repeater(HOLD_DELAY_MS, REPEAT_RPM_MS);
    private final Repeater repLeft  = new Repeater(HOLD_DELAY_MS, REPEAT_HOOD_MS);
    private final Repeater repRight = new Repeater(HOLD_DELAY_MS, REPEAT_HOOD_MS);

    private String lastNote = "";

    // =================================================================

    @Override
    public void init() {
        // ---- motor -------------------------------------------------
        try {
            flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
            flywheel.setDirection(FLYWHEEL_REVERSED
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);

            // FLOAT, not BRAKE. Braking a loaded flywheel dumps its energy
            // back through the gearbox.
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setPower(0);
        } catch (Exception e) {
            initError = "flywheel '" + FLYWHEEL_NAME + "' not found: " + e.getMessage();
        }

        // ---- choose a mode -----------------------------------------
        if (flywheel == null) {
            mode = Mode.POWER;
            modeReason = "no motor";
        } else if (!isSet(TICKS_PER_MOTOR_REV) || !isSet(FLYWHEEL_REVS_PER_MOTOR_REV)) {
            mode = Mode.POWER;
            modeReason = "TICKS_PER_MOTOR_REV / FLYWHEEL_REVS_PER_MOTOR_REV not filled in";
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (FLYWHEEL_REVS_PER_MOTOR_REV <= 0 || TICKS_PER_MOTOR_REV <= 0) {
            mode = Mode.POWER;
            modeReason = "gearing constants must be greater than zero";
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            mode = Mode.RPM;
            modeReason = "gearing supplied";
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (isSet(VEL_P) && isSet(VEL_I) && isSet(VEL_D) && isSet(VEL_F)) {
                try {
                    flywheel.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
                } catch (Exception e) {
                    lastNote = "velocity PIDF rejected: " + e.getMessage();
                }
            }
        }

        // ---- servo --------------------------------------------------
        // Grabbed, but NOT commanded. Nothing moves at init.
        try {
            hood = hardwareMap.get(Servo.class, HOOD_NAME);
        } catch (Exception e) {
            hoodError = "hood '" + HOOD_NAME + "' not found: " + e.getMessage();
        }

        hoodCalibrated = isSet(HOOD_MIN) && isSet(HOOD_MAX) && isSet(HOOD_START)
                && HOOD_MIN >= 0.0 && HOOD_MAX <= 1.0
                && HOOD_MIN < HOOD_MAX
                && HOOD_START >= HOOD_MIN && HOOD_START <= HOOD_MAX;

        if (!hoodCalibrated && hoodError == null && isSet(HOOD_MIN)) {
            hoodError = "HOOD_MIN/MAX/START are set but inconsistent - check the ordering and the 0..1 range";
        }

        telemetry.addLine("SHOOTER TEST");
        telemetry.addLine();
        telemetry.addData("flywheel mode", mode + "  (" + modeReason + ")");
        telemetry.addData("hood", hoodCalibrated ? "calibrated" : "CALIBRATION MODE");
        telemetry.addLine();
        telemetry.addLine("Nothing moves until you press something.");
        telemetry.addLine("Robot on a stand, hands clear of the wheel.");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();

        readButtons(now);
        driveFlywheel(now);
        report();
    }

    @Override
    public void stop() {
        // Belt and braces: the SDK stops motors at the end of an OpMode, but
        // this OpMode is the one spinning a heavy wheel, so it says so itself.
        if (flywheel != null) {
            flywheel.setPower(0);
        }
        running = false;
    }

    // =================================================================
    // Buttons
    // =================================================================

    private void readButtons(long now) {
        boolean up    = gamepad2.dpad_up    || gamepad1.dpad_up;
        boolean down  = gamepad2.dpad_down  || gamepad1.dpad_down;
        boolean left  = gamepad2.dpad_left  || gamepad1.dpad_left;
        boolean right = gamepad2.dpad_right || gamepad1.dpad_right;
        boolean b     = gamepad2.b          || gamepad1.b;
        boolean back  = gamepad2.back       || gamepad1.back;

        // ---- target ------------------------------------------------
        // Hold to repeat. The target is a number on a screen; the worst a
        // runaway hold can do is set a speed you did not want, and it stops
        // at MAX_TARGET_RPM either way.
        if (repUp.fired(up, now))     stepTarget(+1);
        if (repDown.fired(down, now)) stepTarget(-1);
        if (back && !prevBack) {
            targetRpm = 0;
            targetPower = 0;
            lastNote = "target reset to zero";
        }

        // ---- hood ---------------------------------------------------
        // Coarse jog only exists during calibration. See HOOD_COARSE_MULTIPLIER.
        boolean coarse = !hoodCalibrated
                && (gamepad2.left_bumper || gamepad1.left_bumper);
        double step = coarse ? HOOD_STEP * HOOD_COARSE_MULTIPLIER : HOOD_STEP;

        // Hold to repeat ONLY once the hood is calibrated. Before that the
        // software limits do not exist, so a held button walks the horn into
        // whatever the mechanism's real stop is. During calibration you get
        // one step per press and you get to look at it first.
        boolean rightFire, leftFire;
        if (hoodCalibrated) {
            rightFire = repRight.fired(right, now);
            leftFire  = repLeft.fired(left, now);
        } else {
            rightFire = right && !prevRight;
            leftFire  = left  && !prevLeft;
        }

        if (rightFire) stepHood(+step);
        if (leftFire)  stepHood(-step);

        if (b && !prevB) {
            if (hood == null) {
                lastNote = "no hood servo in the configuration";
            } else if (!hoodCalibrated) {
                lastNote = "no HOOD_START yet - calibrate first";
            } else {
                hoodPos = HOOD_START;
                hood.setPosition(hoodPos);
                lastNote = String.format("hood -> start %.4f", hoodPos);
            }
        }

        prevLeft = left; prevRight = right;
        prevB = b; prevBack = back;
    }

    private void stepTarget(int dir) {
        if (mode == Mode.RPM) {
            targetRpm = clamp(targetRpm + dir * RPM_STEP, 0.0, MAX_TARGET_RPM);
            lastNote = String.format("target %.0f RPM", targetRpm);
        } else {
            targetPower = clamp(targetPower + dir * POWER_STEP, 0.0, MAX_TEST_POWER);
            lastNote = String.format("target POWER %.2f", targetPower);
        }
    }

    /**
     * Move the hood one step, clamped.
     *
     * Two different clamps on purpose. Once calibrated we clamp to the limits
     * you measured. Before that we clamp only to 0..1, because finding the
     * limits is the entire point and clamping to limits that do not exist yet
     * would make calibration impossible.
     */
    private void stepHood(double delta) {
        if (hood == null) { lastNote = "no hood servo in the configuration"; return; }

        double lo = hoodCalibrated ? HOOD_MIN : 0.0;
        double hi = hoodCalibrated ? HOOD_MAX : 1.0;

        // First ever command: start from the calibrated start, or from the
        // middle of the range while calibrating. We have not written to the
        // servo before now, so we do not know where the horn physically is.
        double from;
        if (isSet(hoodPos))            from = hoodPos;
        else if (hoodCalibrated)       from = HOOD_START;
        else                           from = 0.5;

        hoodPos = clamp(from + delta, lo, hi);
        hood.setPosition(hoodPos);

        if (hoodCalibrated) {
            lastNote = String.format("hood %.4f", hoodPos);
        } else {
            lastNote = String.format("CAL hood %.4f  <- record this", hoodPos);
        }
    }

    // =================================================================
    // Flywheel
    // =================================================================

    private void driveFlywheel(long now) {
        if (flywheel == null) return;

        double trigger = Math.max(gamepad2.right_trigger, gamepad1.right_trigger);
        boolean wantRun = trigger > TRIGGER_THRESHOLD;

        if (!wantRun) {
            if (running) {
                flywheel.setPower(0);
                running = false;
                commandStartNanos = 0;
            }
            return;
        }

        if (!running) {
            running = true;
            commandStartNanos = now;
        }

        if (mode == Mode.RPM) {
            flywheel.setVelocity(rpmToTicksPerSecond(targetRpm));
            checkEncoderIsReal(now);
        } else {
            flywheel.setPower(targetPower);
        }
    }

    /**
     * Catch a dead or unplugged encoder before the velocity controller winds
     * itself up to full power chasing a target it can never see.
     */
    private void checkEncoderIsReal(long now) {
        if (encoderProven || commandStartNanos == 0) return;
        if (targetRpm <= 0) return;   // nothing commanded, nothing to prove

        double movingTicksPerSec = Math.abs(flywheel.getVelocity());
        if (movingTicksPerSec > 1.0) {
            encoderProven = true;
            encoderNote = "encoder confirmed live";
            return;
        }

        double elapsedMs = (now - commandStartNanos) / 1e6;
        if (elapsedMs > ENCODER_PROOF_MS) {
            flywheel.setPower(0);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mode = Mode.POWER;
            modeReason = "no encoder counts after " + (int) ENCODER_PROOF_MS + " ms";
            targetPower = 0;
            running = false;
            encoderNote = "NO ENCODER SIGNAL - dropped to POWER mode, motor cut";
            lastNote = "check the encoder cable on the flywheel motor port";
        }
    }

    private double rpmToTicksPerSecond(double flywheelRpm) {
        double motorRpm = flywheelRpm / FLYWHEEL_REVS_PER_MOTOR_REV;
        return motorRpm * TICKS_PER_MOTOR_REV / 60.0;
    }

    private double measuredFlywheelRpm() {
        double ticksPerSec = flywheel.getVelocity();
        double motorRpm = ticksPerSec * 60.0 / TICKS_PER_MOTOR_REV;
        return motorRpm * FLYWHEEL_REVS_PER_MOTOR_REV;
    }

    // =================================================================
    // Telemetry
    // =================================================================

    private void report() {
        if (initError != null) telemetry.addData("MOTOR ERROR", initError);
        if (hoodError != null) telemetry.addData("HOOD ERROR", hoodError);
        if (!lastNote.isEmpty()) telemetry.addData("note", lastNote);

        telemetry.addLine();
        telemetry.addData("flywheel", running ? "RUNNING" : "stopped");
        telemetry.addData("control mode", mode == Mode.RPM ? "CLOSED LOOP RPM" : "OPEN LOOP POWER");

        if (mode == Mode.RPM) {
            telemetry.addData("target", "%.0f RPM", targetRpm);
            if (flywheel != null) {
                telemetry.addData("measured", "%.0f RPM", measuredFlywheelRpm());
                telemetry.addData("error", "%.0f RPM", targetRpm - measuredFlywheelRpm());
            } else {
                telemetry.addData("measured", "no motor");
            }
            telemetry.addData("encoder", encoderNote);

            if (isSet(MOTOR_FREE_SPEED_RPM)) {
                double motorRpmNeeded = targetRpm / FLYWHEEL_REVS_PER_MOTOR_REV;
                if (motorRpmNeeded > 0.9 * MOTOR_FREE_SPEED_RPM) {
                    telemetry.addData("WARNING",
                            "target needs %.0f motor RPM, free speed is %.0f",
                            motorRpmNeeded, MOTOR_FREE_SPEED_RPM);
                }
            }
        } else {
            // Never labelled RPM. Power is not speed.
            telemetry.addData("target", "%.2f POWER (0 to %.2f)", targetPower, MAX_TEST_POWER);
            telemetry.addData("measured", "NOT AVAILABLE - no RPM without encoder constants");
            telemetry.addData("why POWER mode", modeReason);
            telemetry.addLine("Power is not RPM. It sags with battery voltage");
            telemetry.addLine("and is not proportional to wheel speed.");
        }

        telemetry.addData("step", mode == Mode.RPM
                ? String.format("%.0f RPM", RPM_STEP)
                : String.format("%.2f power", POWER_STEP));

        // ---- hood ---------------------------------------------------
        telemetry.addLine();
        if (hoodCalibrated) {
            telemetry.addData("hood", "calibrated");
            telemetry.addData("hood commanded", isSet(hoodPos)
                    ? String.format("%.4f", hoodPos)
                    : "not commanded yet - press B");
            telemetry.addData("hood min / max", "%.4f / %.4f", HOOD_MIN, HOOD_MAX);
            telemetry.addData("hood start", "%.4f", HOOD_START);
        } else {
            telemetry.addData("hood", "HOOD CALIBRATION MODE");
            telemetry.addData("hood commanded", isSet(hoodPos)
                    ? String.format("%.4f  <- record this", hoodPos)
                    : "not commanded yet - dpad right/left to jog");
            telemetry.addData("hood min / max", "UNCALIBRATED - clamped to 0.0 / 1.0 only");
            telemetry.addData("hood coarse jog", (gamepad2.left_bumper || gamepad1.left_bumper)
                    ? String.format("ON - %.3f per press", HOOD_STEP * HOOD_COARSE_MULTIPLIER)
                    : "hold LB for 10x steps");
            telemetry.addLine("LINKAGE OFF. Single presses. Find both safe ends,");
            telemetry.addLine("back off 0.02, then fill in HOOD_MIN/MAX/START.");
        }
        telemetry.addData("hood step", "%.4f", HOOD_STEP);

        telemetry.addLine();
        telemetry.addLine("RT hold = spin   dpad U/D = target");
        telemetry.addLine("dpad R/L = hood  B = hood to start  BACK = zero");

        telemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
