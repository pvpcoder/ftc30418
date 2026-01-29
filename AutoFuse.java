package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutoFuse_1.5m_FullSpeed", group="Test")
public class AutoFuse extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // ===== Constants for 435RPM Motors & 96mm Wheels =====
    private static final double WHEEL_DIAMETER_IN = 96.0 / 25.4;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 383.6;
    private static final double COUNTS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    // 1.5 meters = 59.055 inches
    private static final double DIST_IN = 59.055;
    private static final double DRIVE_POWER = 1.0; // FULL SPEED

    @Override
    public void runOpMode() {

        // 1. Hardware Map
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");

        // 2. Directions - Reversing RIGHT side stops the spinning 180s
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Brake - Stops momentum so it doesn't overshoot 1.5m
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 4. Reset Encoders
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to Launch - 1.5m Full Speed");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Move 1.5 Meters Forward
        encoderDrive(DRIVE_POWER, DIST_IN, DIST_IN, DIST_IN, DIST_IN);
    }

    /**
     * Helper method to drive using encoders.
     * Each parameter is inches for that specific wheel.
     */
    public void encoderDrive(double power, double flIn, double blIn, double frIn, double brIn) {
        int flTarget = frontLeftMotor.getCurrentPosition() + (int)(flIn * COUNTS_PER_INCH);
        int blTarget = backLeftMotor.getCurrentPosition() + (int)(blIn * COUNTS_PER_INCH);
        int frTarget = frontRightMotor.getCurrentPosition() + (int)(frIn * COUNTS_PER_INCH);
        int brTarget = backRightMotor.getCurrentPosition() + (int)(brIn * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(flTarget);
        backLeftMotor.setTargetPosition(blTarget);
        frontRightMotor.setTargetPosition(frTarget);
        backRightMotor.setTargetPosition(brTarget);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);

        while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy())) {
            telemetry.addData("Path", "Driving to 1.5m");
            telemetry.update();
        }

        // Stop
        setPower(0, 0, 0, 0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    private void setPower(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }
}