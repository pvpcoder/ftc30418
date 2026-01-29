package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ZZ_Shooter_RPM_TeleOp", group = "Test")
public class ZZ_Shooter_RPM_TeleOp extends OpMode {

    // ===== Your robot hardware =====
    private DcMotorEx outakeLeft;
    private DcMotorEx outakeRight;

    private DcMotor intakeFrontMotor;
    private DcMotor intakeBackMotor;
    
    // ===== Shooter RPM control (like the other team's) =====
    // goBILDA 5203 6000RPM commonly ~28 ticks/rev
    private static final double TICKS_PER_REV = 28.0;

    // Pick your presets (change these later after testing)
    private final int[] rpmPresets = {4500, 5000, 5400};
    private int presetIndex = -1;     // -1 means OFF
    private double targetRPM = 0.0;

    private static final double RPM_TOLERANCE = 200.0; // "ready" window

    // ===== Feed / intake =====
    private static final double BACK_INTAKE_SCALE = 0.80;

    // Pulse feed to reduce constant contact
    private static final long FEED_ON_MS = 150;
    private static final long FEED_OFF_MS = 150;
    private boolean feedStateOn = false;
    private long lastFeedToggleMs = 0;

    // Edge detection
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean lastDL = false;

    @Override
    public void init() {
        // Names MUST match robot configuration exactly
        outakeLeft = hardwareMap.get(DcMotorEx.class, "outakeLeft");
        outakeRight = hardwareMap.get(DcMotorEx.class, "outakeRight");

        intakeFrontMotor = hardwareMap.get(DcMotor.class, "intakeFrontMotor");
        intakeBackMotor  = hardwareMap.get(DcMotor.class, "intakeBackMotor");

        // Match your BaseFullTeleOp directions
        outakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoders needed for velocity control
        outakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop everything
        outakeLeft.setPower(0);
        outakeRight.setPower(0);
        intakeFrontMotor.setPower(0);
        intakeBackMotor.setPower(0);

        telemetry.addLine("ZZ_Shooter_RPM_TeleOp READY");
        telemetry.addLine("RB: cycle RPM presets | LB: RPM OFF | DpadLeft: reverse RPM");
        telemetry.addLine("Y: spin shooter (uses target RPM)");
        telemetry.addLine("RT: feed (only feeds when RPM is READY)");
        telemetry.addLine("B: PURGE (reverses everything)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== Controls (gamepad2 like your BaseFullTeleOp) =====
        boolean purge = gamepad2.b;
        boolean shoot = gamepad2.y;
        boolean feed = gamepad2.right_trigger > 0.05;

        // ===== Presets (use gamepad2 bumpers like many teams) =====
        boolean rb = gamepad2.right_bumper;
        boolean lb = gamepad2.left_bumper;
        boolean dpadLeft = gamepad2.dpad_left;

        // RB cycles presets
        if (rb && !lastRB) {
            presetIndex = (presetIndex + 1) % rpmPresets.length;
            targetRPM = rpmPresets[presetIndex];
        }
        // LB turns RPM off
        if (lb && !lastLB) {
            presetIndex = -1;
            targetRPM = 0.0;
        }
        // DpadLeft sets reverse RPM (useful for unjamming)
        if (dpadLeft && !lastDL) {
            presetIndex = -1;
            targetRPM = -4000.0;
        }

        lastRB = rb;
        lastLB = lb;
        lastDL = dpadLeft;

        // ===== PURGE overrides everything =====
        if (purge) {
            // reverse shooter + reverse intakes
            outakeLeft.setPower(-1.0);
            outakeRight.setPower(-1.0);

            intakeFrontMotor.setPower(-1.0);
            intakeBackMotor.setPower(-BACK_INTAKE_SCALE);

            feedStateOn = false;
            lastFeedToggleMs = System.currentTimeMillis();

            telemetry.addLine("PURGE ACTIVE");
            telemetry.update();
            return;
        }

        // ===== Shooter velocity control =====
        // If shoot is held AND targetRPM != 0, command velocity. Otherwise stop.
        if (shoot && targetRPM != 0.0) {
            double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
            outakeLeft.setVelocity(targetTicksPerSec);
            outakeRight.setVelocity(targetTicksPerSec);
        } else {
            outakeLeft.setVelocity(0.0);
            outakeRight.setVelocity(0.0);
        }

        // ===== Compute "RPM ready" =====
        double leftRPM  = (outakeLeft.getVelocity()  / TICKS_PER_REV) * 60.0;
        double rightRPM = (outakeRight.getVelocity() / TICKS_PER_REV) * 60.0;
        double avgRPM = (leftRPM + rightRPM) / 2.0;

        boolean rpmReady = shoot
                && (targetRPM > 0)
                && (Math.abs(avgRPM - targetRPM) <= RPM_TOLERANCE);

        // ===== Feed logic (only when RPM ready) =====
        double frontIntake = 0.0;
        double backIntake  = 0.0;

        long now = System.currentTimeMillis();

        if (feed && rpmReady) {
            // pulse feed to prevent constant contact
            long interval = feedStateOn ? FEED_ON_MS : FEED_OFF_MS;
            if (now - lastFeedToggleMs >= interval) {
                feedStateOn = !feedStateOn;
                lastFeedToggleMs = now;
            }
            if (feedStateOn) {
                frontIntake = 1.0;
                backIntake  = BACK_INTAKE_SCALE;
            }
        } else if (feed && !shoot) {
            // allow normal intake when shooter is NOT on
            frontIntake = 1.0;
            backIntake  = BACK_INTAKE_SCALE;
            feedStateOn = false;
            lastFeedToggleMs = now;
        } else {
            // not feeding
            feedStateOn = false;
            lastFeedToggleMs = now;
        }

        intakeFrontMotor.setPower(frontIntake);
        intakeBackMotor.setPower(backIntake);

        // ===== Telemetry (simple + useful) =====
        telemetry.addData("TargetRPM", "%.0f", targetRPM);
        telemetry.addData("LeftRPM", "%.0f", leftRPM);
        telemetry.addData("RightRPM", "%.0f", rightRPM);
        telemetry.addData("RPM Ready", rpmReady ? "YES" : "NO");

        telemetry.addData("Shoot(Y)", shoot);
        telemetry.addData("Feed(RT)", feed);
        telemetry.addData("PulseOn", feedStateOn);

        telemetry.addLine("RB=next preset | LB=off | DpadLeft=reverse | B=purge");
        telemetry.update();
    }

    @Override
    public void stop() {
        outakeLeft.setVelocity(0.0);
        outakeRight.setVelocity(0.0);
        intakeFrontMotor.setPower(0.0);
        intakeBackMotor.setPower(0.0);
    }
}
