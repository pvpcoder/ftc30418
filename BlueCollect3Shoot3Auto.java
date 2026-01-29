package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Blue: 3x(Collect3 + Shoot3)", group="Main")
public class BlueCollect3Shoot3Auto extends LinearOpMode {

    // ===== Drive motors =====
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // ===== Mechanisms =====
    private DcMotor intakeFrontMotor, intakeBackMotor;
    private DcMotor outakeLeft, outakeRight;
    private Servo hoodServo;

    // ===== IMU =====
    private IMU imu;

    // ===== DISTANCES YOU MUST TUNE =====
    // Collect path (through each 3-ball row)
    private static final double ROW_FORWARD_IN = 24.0; // put robot at the start of Row 1, then measure from that point to a point just past the 3rd ball where you’re sure all 3 are collected. Use a tape measure.
    private static final double ROW_SPACING_IN = 12.0; // measure the center-to-center spacing between the rows (or measure from your robot centerline at row 1 to where it needs to be for row 2).

    // From end of a row to shooting spot (tune for your robot/field)
    private static final double TO_SHOOT_FWD_IN = 18.0;
    private static final double TO_SHOOT_STRAFE_IN = 0.0;

    // ===== Drivetrain constants =====
    private static final double WHEEL_DIAMETER_IN = 96.0 / 25.4;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 383.6; // 435rpm typical; verify with getTicksPerRev() if you want
    private static final double COUNTS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    private static final double DRIVE_POWER = 0.60;
    private static final double STRAFE_POWER = 0.60;

    // Heading hold
    private static final double kP_TURN = 2.5;

    // Timeouts (safety)
    private static final double TIMEOUT_FWD_S = 2.5;
    private static final double TIMEOUT_STRAFE_S = 2.5;

    // Intake
    private static final double BACK_INTAKE_SCALE = 0.80;

    // IMU mounting: hub on LEFT, logo out, USB up
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // If yaw sign is flipped, set true
    private static final boolean INVERT_IMU_YAW = false;

    // ===== SHOOTING constants (same as earlier block) =====
    private static final long HOLD_BACK_MS = 180;
    private static final double HOLD_BACK_PWR_FRONT = -0.25;
    private static final double HOLD_BACK_PWR_BACK  = -0.20;

    private static final long SHOOTER_SPINUP_MS = 500;
    private static final long FEED_ON_MS  = 150;
    private static final long FEED_OFF_MS = 170;
    private static final double BACK_HOLD_DURING_SHOOT = 0.10;

    @Override
    public void runOpMode() {
        // Hardware map
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");

        intakeFrontMotor = hardwareMap.get(DcMotor.class, "intakeFrontMotor");
        intakeBackMotor  = hardwareMap.get(DcMotor.class, "intakeBackMotor");

        outakeLeft  = hardwareMap.get(DcMotor.class, "outakeLeft");
        outakeRight = hardwareMap.get(DcMotor.class, "outakeRight");

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Directions (match your TeleOp)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));

        telemetry.addData("CPI", COUNTS_PER_INCH);
        telemetry.addLine("Ready: start at S, facing straight for your lane.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        final double holdHeading = getYawRad(); // lock heading at start

        // ===== Cycle 1: Collect row 1 -> Shoot 3 =====
        setIntake(true);
        driveForwardInches(ROW_FORWARD_IN, DRIVE_POWER, holdHeading, TIMEOUT_FWD_S);
        setIntake(false);

        goToShootSpot(holdHeading);
        shootN(3);

        // ===== Move to row 2 =====
        strafeInches(ROW_SPACING_IN, STRAFE_POWER, holdHeading, TIMEOUT_STRAFE_S);

        // ===== Cycle 2 =====
        setIntake(true);
        driveForwardInches(ROW_FORWARD_IN, DRIVE_POWER, holdHeading, TIMEOUT_FWD_S);
        setIntake(false);

        goToShootSpot(holdHeading);
        shootN(3);

        // ===== Move to row 3 =====
        strafeInches(ROW_SPACING_IN, STRAFE_POWER, holdHeading, TIMEOUT_STRAFE_S);

        // ===== Cycle 3 =====
        setIntake(true);
        driveForwardInches(ROW_FORWARD_IN, DRIVE_POWER, holdHeading, TIMEOUT_FWD_S);
        setIntake(false);

        goToShootSpot(holdHeading);
        shootN(3);

        stopAll();
    }

    // ===== Navigation to shoot spot (simple placeholder) =====
    private void goToShootSpot(double holdHeading) {
        if (Math.abs(TO_SHOOT_STRAFE_IN) > 0.1) {
            strafeInches(TO_SHOOT_STRAFE_IN, STRAFE_POWER, holdHeading, TIMEOUT_STRAFE_S);
        }
        if (Math.abs(TO_SHOOT_FWD_IN) > 0.1) {
            driveForwardInches(TO_SHOOT_FWD_IN, DRIVE_POWER, holdHeading, TIMEOUT_FWD_S);
        }
    }

    // ===== Intake =====
    private void setIntake(boolean on) {
        if (on) {
            intakeFrontMotor.setPower(1.0);
            intakeBackMotor.setPower(BACK_INTAKE_SCALE);
        } else {
            intakeFrontMotor.setPower(0.0);
            intakeBackMotor.setPower(0.0);
        }
    }

    // ===== Shoot =====
    private void shootN(int shots) {
        intakeFrontMotor.setPower(HOLD_BACK_PWR_FRONT);
        intakeBackMotor.setPower(HOLD_BACK_PWR_BACK);
        sleep(HOLD_BACK_MS);

        intakeFrontMotor.setPower(0.0);
        intakeBackMotor.setPower(0.0);
        sleep(60);

        outakeLeft.setPower(1.0);
        outakeRight.setPower(1.0);
        sleep(SHOOTER_SPINUP_MS);

        for (int i = 0; i < shots && opModeIsActive(); i++) {
            intakeBackMotor.setPower(BACK_HOLD_DURING_SHOOT);
            intakeFrontMotor.setPower(1.0);
            sleep(FEED_ON_MS);

            intakeFrontMotor.setPower(0.0);
            sleep(FEED_OFF_MS);
        }

        intakeFrontMotor.setPower(0.0);
        intakeBackMotor.setPower(0.0);
        outakeLeft.setPower(0.0);
        outakeRight.setPower(0.0);
    }

    private void stopAll() {
        setDrivePower(0,0,0,0);
        setIntake(false);
        outakeLeft.setPower(0.0);
        outakeRight.setPower(0.0);
    }

    // ===== Drive primitives (encoderless, using encoder delta + IMU hold) =====
    private void driveForwardInches(double inches, double power, double targetHeading, double timeoutS) {
        int targetCounts = (int) Math.round(Math.abs(inches) * COUNTS_PER_INCH);
        int direction = (inches >= 0) ? 1 : -1;

        int fl0 = frontLeftMotor.getCurrentPosition();
        int bl0 = backLeftMotor.getCurrentPosition();
        int fr0 = frontRightMotor.getCurrentPosition();
        int br0 = backRightMotor.getCurrentPosition();

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutS) {
            int fl = Math.abs(frontLeftMotor.getCurrentPosition() - fl0);
            int bl = Math.abs(backLeftMotor.getCurrentPosition() - bl0);
            int fr = Math.abs(frontRightMotor.getCurrentPosition() - fr0);
            int br = Math.abs(backRightMotor.getCurrentPosition() - br0);

            int avg = (fl + bl + fr + br) / 4;
            if (avg >= targetCounts) break;

            double err = angleWrap(targetHeading - getYawRad());
            double rx = clamp(kP_TURN * err, -0.35, 0.35);

            double y = direction * power;
            double flp = y + rx, blp = y + rx, frp = y - rx, brp = y - rx;

            setDrivePower(normalize(flp, blp, frp, brp));
        }
        setDrivePower(0,0,0,0);
        sleep(40);
    }

    private void strafeInches(double inches, double power, double targetHeading, double timeoutS) {
        int targetCounts = (int) Math.round(Math.abs(inches) * COUNTS_PER_INCH);
        int direction = (inches >= 0) ? 1 : -1; // + right

        int fl0 = frontLeftMotor.getCurrentPosition();
        int bl0 = backLeftMotor.getCurrentPosition();
        int fr0 = frontRightMotor.getCurrentPosition();
        int br0 = backRightMotor.getCurrentPosition();

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutS) {
            int fl = Math.abs(frontLeftMotor.getCurrentPosition() - fl0);
            int bl = Math.abs(backLeftMotor.getCurrentPosition() - bl0);
            int fr = Math.abs(frontRightMotor.getCurrentPosition() - fr0);
            int br = Math.abs(backRightMotor.getCurrentPosition() - br0);

            int avg = (fl + bl + fr + br) / 4;
            if (avg >= targetCounts) break;

            double err = angleWrap(targetHeading - getYawRad());
            double rx = clamp(kP_TURN * err, -0.35, 0.35);

            double x = direction * power;
            double flp = x + rx;
            double blp = -x + rx;
            double frp = -x - rx;
            double brp = x - rx;

            setDrivePower(normalize(flp, blp, frp, brp));
        }
        setDrivePower(0,0,0,0);
        sleep(40);
    }

    private double[] normalize(double fl, double bl, double fr, double br) {
        double denom = Math.max(1.0, Math.abs(fl) + Math.abs(bl) + Math.abs(fr) + Math.abs(br));
        return new double[]{fl/denom, bl/denom, fr/denom, br/denom};
    }

    private void setDrivePower(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private void setDrivePower(double[] p) {
        setDrivePower(p[0], p[1], p[2], p[3]);
    }

    // ===== IMU helpers =====
    private double getYawRad() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return INVERT_IMU_YAW ? -yaw : yaw;
    }

    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
