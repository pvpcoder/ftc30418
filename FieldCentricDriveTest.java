package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="FieldCentric Drive Test", group="Test")
public class FieldCentricDriveTest extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // IMU
    private IMU imu;

    // Tuning
    private static final double DEAD = 0.05;
    private static final double STRAFE_CORRECTION = 1.0;

    // If turning left (CCW) makes yaw decrease, set true
    private static final boolean INVERT_IMU_YAW = false;

    // Hub mounting: Control Hub on LEFT side, REV logo faces OUT, USB up
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // Field-centric zero (auto at start)
    private double yawZero = 0.0;

    @Override
    public void runOpMode() {

        // Hardware map
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");

        // Directions (match your TeleOp)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU init
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));

        telemetry.addLine("FieldCentric Drive Test ready.");
        telemetry.addLine("Robot should be placed facing your intended 'forward' before start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Auto-zero at start (no button required)
        yawZero = getYawRad();

        while (opModeIsActive()) {

            // Stick conventions
            double y  = -gamepad1.left_stick_y;  // forward +
            double x  =  gamepad1.left_stick_x;  // right +
            double rx =  gamepad1.right_stick_x; // CCW +

            if (Math.abs(y) < DEAD)  y = 0;
            if (Math.abs(x) < DEAD)  x = 0;
            if (Math.abs(rx) < DEAD) rx = 0;

            // Slow mode while holding left bumper (optional)
            double speedScale = gamepad1.left_bumper ? 0.35 : 1.0;
            y *= speedScale;
            x *= speedScale;
            rx *= speedScale;

            // Driver-relative field-centric (Option B): field frame is defined at start
            double imuYaw = getYawRad();
            double heading = normalizeRadians(imuYaw - yawZero);

            // Rotate joystick by -heading (field -> robot)
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            rotX *= STRAFE_CORRECTION;

            double denom = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1.0);

            double fl = (rotY + rotX + rx) / denom;
            double bl = (rotY - rotX + rx) / denom;
            double fr = (rotY - rotX - rx) / denom;
            double br = (rotY + rotX - rx) / denom;

            setDrive(fl, bl, fr, br);

            telemetry.addData("imuYaw(rad)", imuYaw);
            telemetry.addData("yawZero(rad)", yawZero);
            telemetry.addData("heading(rad)", heading);
            telemetry.addData("SlowMode(LB)", gamepad1.left_bumper);
            telemetry.update();
        }
    }

    private void setDrive(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private double getYawRad() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return INVERT_IMU_YAW ? -yaw : yaw;
    }

    private double normalizeRadians(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
