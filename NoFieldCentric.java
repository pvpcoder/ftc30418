package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "NoFieldCentric", group = "Test")
public class NoFieldCentric extends LinearOpMode {

    // ===== Drive motors =====
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // ===== Mechanisms =====
    private DcMotor intakeFrontMotor, intakeBackMotor;
    private DcMotorEx outakeLeft, outakeRight; // shooter flywheels
    private Servo hoodServo;

    // ===== IMU (kept mapped) =====
    private IMU imu;

    // ===== Vision =====
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ===== Tuning =====
    private static final double DEAD = 0.05;
    private static final double STRAFE_CORRECTION = 1.0;
    private static final double BACK_INTAKE_SCALE = 0.80;

    // ===== Hood =====
    private double hoodPos = 0.50;
    private static final double HOOD_MIN = 0.20;
    private static final double HOOD_MAX = 1.00;

    private boolean autoHood = true;
    private boolean lastA = false;

    // IMU mounting: Control Hub on LEFT side, REV logo OUT, USB UP
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // ===== Shooter encoder RPM control =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_TARGET_RPM = 5000.0;

    @Override
    public void runOpMode() {

        // ===== Hardware map =====
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        intakeFrontMotor = hardwareMap.get(DcMotor.class, "intakeFrontMotor");
        intakeBackMotor  = hardwareMap.get(DcMotor.class, "intakeBackMotor");

        outakeLeft  = hardwareMap.get(DcMotorEx.class, "outakeLeft");
        outakeRight = hardwareMap.get(DcMotorEx.class, "outakeRight");

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // ===== Directions (UNCHANGED) =====
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== Drive motor behavior + encoder mode =====
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== Shooter mode =====
        outakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== IMU (initialized but NOT used for driving) =====
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));

        // ===== Vision =====
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("NoFieldCentric loaded. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE (NORMAL / ROBOT-CENTRIC) =================
            double y  = -gamepad1.left_stick_y;  // forward (+)
            double x  =  gamepad1.left_stick_x;  // strafe right (+)
            double rx =  gamepad1.right_stick_x; // turn (+)

            if (Math.abs(y)  < DEAD) y = 0;
            if (Math.abs(x)  < DEAD) x = 0;
            if (Math.abs(rx) < DEAD) rx = 0;

            double rotX = x * STRAFE_CORRECTION;
            double rotY = y;

            double denom = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1.0);

            double fl = (rotY + rotX + rx) / denom;
            double bl = (rotY - rotX + rx) / denom;
            double fr = (rotY - rotX - rx) / denom;
            double br = (rotY + rotX - rx) / denom;

            setDrive(fl, bl, fr, br);

            // ================= MECHANISMS (GP2) =================
            boolean purge = gamepad2.b;
            boolean feed  = gamepad2.right_trigger > 0.05; // RT
            boolean shoot = gamepad2.y;                    // Y

            // AUTO HOOD TOGGLE
            if (gamepad2.a && !lastA) autoHood = !autoHood;
            lastA = gamepad2.a;

            // Shooter RPM readout (telemetry)
            double leftRPM  = ticksPerSecToRpm(outakeLeft.getVelocity(),  SHOOTER_TICKS_PER_REV);
            double rightRPM = ticksPerSecToRpm(outakeRight.getVelocity(), SHOOTER_TICKS_PER_REV);

            if (purge) {
                // Purge: reverse ONLY the intakes
                intakeFrontMotor.setPower(-1.0);
                intakeBackMotor.setPower(-BACK_INTAKE_SCALE);

                // Do NOT run shooter backwards during purge — stop it
                outakeLeft.setVelocity(0.0);
                outakeRight.setVelocity(0.0);

            } else {

                // Shooter velocity control
                if (shoot) {
                    double targetTicksPerSec = rpmToTicksPerSec(SHOOTER_TARGET_RPM, SHOOTER_TICKS_PER_REV);
                    outakeLeft.setVelocity(targetTicksPerSec);
                    outakeRight.setVelocity(targetTicksPerSec);
                } else {
                    outakeLeft.setVelocity(0.0);
                    outakeRight.setVelocity(0.0);
                }

                // Intake control (RT = both intakes)
                if (feed) {
                    intakeFrontMotor.setPower(1.0);
                    intakeBackMotor.setPower(BACK_INTAKE_SCALE);
                } else {
                    intakeFrontMotor.setPower(0.0);
                    intakeBackMotor.setPower(0.0);
                }
            }

            // ---------- HOOD ----------
            AprilTagDetection best = getBestOfIds(aprilTag.getDetections(), 20, 22);

            if (autoHood && best != null) {
                hoodPos = clamp(hoodFromRange(best.ftcPose.range), HOOD_MIN, HOOD_MAX);
                hoodServo.setPosition(hoodPos);
            } else if (!autoHood) {
                if (gamepad2.right_bumper) hoodPos += 0.01;
                if (gamepad2.left_bumper)  hoodPos -= 0.01;
                hoodServo.setPosition(clamp(hoodPos, HOOD_MIN, HOOD_MAX));
            }

            telemetry.addData("Drive", "Robot-Centric (no field centric)");
            telemetry.addData("Feed(RT)", feed);
            telemetry.addData("Purge(B)", purge);
            telemetry.addData("Shoot(Y)", shoot);
            telemetry.addData("BackIntakePower", "%.2f", purge ? -BACK_INTAKE_SCALE : (feed ? BACK_INTAKE_SCALE : 0.0));
            telemetry.addData("Shooter RPM L/R", "%.0f / %.0f", leftRPM, rightRPM);
            telemetry.update();
        }
    }

    // ===== Helpers =====
    private void setDrive(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private double hoodFromRange(double r) {
        return clamp(0.35 + (r - 60) * 0.0025, HOOD_MIN, HOOD_MAX);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private AprilTagDetection getBestOfIds(List<AprilTagDetection> dets, int a, int b) {
        AprilTagDetection best = null;
        for (AprilTagDetection d : dets) {
            if (d.id == a || d.id == b) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
            }
        }
        return best;
    }

    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return (rpm / 60.0) * ticksPerRev;
    }

    private double ticksPerSecToRpm(double ticksPerSec, double ticksPerRev) {
        if (ticksPerRev <= 0.0) return 0.0;
        return (ticksPerSec / ticksPerRev) * 60.0;
    }
}
