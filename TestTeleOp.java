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

@TeleOp(name = "TestTeleOp", group = "Test")
public class TestTeleOp extends LinearOpMode {

    // ===== Drive motors =====
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // ===== Mechanisms =====
    private DcMotor intakeFrontMotor, intakeBackMotor;
    private DcMotorEx outakeLeft, outakeRight; // shooter flywheels (DcMotorEx for setVelocity)
    private Servo hoodServo;

    // ===== IMU =====
    private IMU imu;

    // ===== Vision =====
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ===== Tuning =====
    private static final double DEAD = 0.05;
    private static final double STRAFE_CORRECTION = 1.0;
    private static final double BACK_INTAKE_SCALE = 0.80;

    // If yaw goes the "wrong way" (turn left makes yaw decrease), set to true.
    private static final boolean INVERT_IMU_YAW = false;

    // ===== Field-centric =====
    private double yawZero = 0.0;
    private boolean lastStart = false;

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

        outakeLeft.setDirection(DcMotorSimple.Direction.REVERSE); // keep as-is
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

        // ===== Shooter: encoder velocity mode =====
        outakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== IMU =====
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

        telemetry.addLine("TestTeleOp loaded. Press START.");
        telemetry.update();

        waitForStart();

        yawZero = getImuYawRad();

        while (opModeIsActive()) {

            // ================= DRIVE (GP1) =================
            double y  = -gamepad1.left_stick_y;  // forward (+)
            double x  =  gamepad1.left_stick_x;  // strafe right (+)
            double rx =  gamepad1.right_stick_x; // turn (+)

            if (Math.abs(y)  < DEAD) y = 0;
            if (Math.abs(x)  < DEAD) x = 0;
            if (Math.abs(rx) < DEAD) rx = 0;

            boolean start = gamepad1.start;
            if (start && !lastStart) yawZero = getImuYawRad();
            lastStart = start;

            double imuYaw = getImuYawRad();
            double heading = (imuYaw - yawZero) + 0.0; // hardcoded alliance offset for this test
            heading = normalizeRadians(heading);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            rotX *= STRAFE_CORRECTION;

            double denom = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1.0);

            double fl = (rotY + rotX + rx) / denom;
            double bl = (rotY - rotX + rx) / denom;
            double fr = (rotY - rotX - rx) / denom;
            double br = (rotY + rotX - rx) / denom;

            setDrive(fl, bl, fr, br);

            // ================= MECHANISMS (GP2) =================
            boolean purge = gamepad2.b;

            // RT = intake feed (front + back)
            boolean feed  = gamepad2.right_trigger > 0.05;

            // Y = shooter
            boolean shoot = gamepad2.y;

            // AUTO HOOD TOGGLE
            if (gamepad2.a && !lastA) autoHood = !autoHood;
            lastA = gamepad2.a;

            // Shooter RPM readout (just for telemetry)
            double leftRPM  = ticksPerSecToRpm(outakeLeft.getVelocity(),  SHOOTER_TICKS_PER_REV);
            double rightRPM = ticksPerSecToRpm(outakeRight.getVelocity(), SHOOTER_TICKS_PER_REV);

            if (purge) {
                // Purge: reverse both intakes + reverse shooter
                intakeFrontMotor.setPower(-1.0);
                intakeBackMotor.setPower(-BACK_INTAKE_SCALE);

                outakeLeft.setPower(-1.0);
                outakeRight.setPower(-1.0);

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

                // Intake control (continuous, no pulsing)
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

            telemetry.addData("Shoot(Y)", shoot);
            telemetry.addData("Feed(RT)", feed);
            telemetry.addData("Purge(B)", purge);
            telemetry.addData("Intake Front Pwr", "%.2f", purge ? -1.0 : (feed ? 1.0 : 0.0));
            telemetry.addData("Intake Back  Pwr", "%.2f", purge ? -BACK_INTAKE_SCALE : (feed ? BACK_INTAKE_SCALE : 0.0));
            telemetry.addData("AutoHood(A toggles)", autoHood);
            telemetry.addData("HoodPos", "%.3f", hoodPos);
            telemetry.addData("Shooter RPM L/R", "%.0f / %.0f", leftRPM, rightRPM);
            telemetry.update();
        }
    }

    // ===== Helpers =====
    private double getImuYawRad() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return INVERT_IMU_YAW ? -yaw : yaw;
    }

    private double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private double hoodFromRange(double r) {
        return clamp(0.35 + (r - 60) * 0.0025, HOOD_MIN, HOOD_MAX);
    }

    private void setDrive(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
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
