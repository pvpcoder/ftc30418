package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drive, FIELD CENTRIC.
 *
 * Push the left stick forward and the robot travels in one fixed field
 * direction no matter which way its nose is pointing. The driver no longer has
 * to mentally rotate with the robot.
 *
 * WHERE "FORWARD" COMES FROM
 * --------------------------
 * The Control Hub IMU's yaw. Yaw is zeroed when this OpMode initialises, so
 * whichever way the robot is physically pointing at INIT becomes stick-forward.
 * Put the robot down facing away from the driver station before pressing INIT.
 *
 * This OpMode reads the IMU directly rather than going through Pedro. It is
 * deliberately standalone: it is the OpMode you run to answer "is the
 * drivetrain wired and working", so it should not be able to break because of
 * an odometry or path-following problem.
 *
 * CONTROLS (gamepad 1)
 *   left stick       translate, in field coordinates
 *   right stick X    turn
 *   left bumper      hold for 35% speed
 *   OPTIONS          re-zero forward to whichever way the robot faces now
 *   BACK / SHARE     toggle field centric <-> robot centric
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    /** Speed multiplier while the left bumper is held. */
    private static final double SLOW_SCALE = 0.35;

    /**
     * Strafing is less efficient than driving straight, because the rollers
     * waste some of the push sideways. Scaling the strafe component up a little
     * makes a diagonal stick input actually travel on the diagonal. 1.1 is the
     * usual starting value; raise it if the robot still falls short sideways.
     */
    private static final double STRAFE_GAIN = 1.1;

    @Override
    public void runOpMode() {
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        // Verified on our robot: left side reversed, right side forward.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake rather than coast, so letting go of the stick actually stops
        // the robot instead of letting it drift.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Same hub orientation as Constants.java. If you ever re-mount the
        // Control Hub, change it in BOTH places or heading goes wrong here
        // while staying right in autonomous, which is a miserable bug to find.
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        boolean fieldCentric = true;
        boolean lastOptions  = false;
        boolean lastBack     = false;

        telemetry.addLine("Mecanum TeleOp -- FIELD CENTRIC");
        telemetry.addLine();
        telemetry.addLine("Point the robot AWAY FROM THE DRIVER STATION");
        telemetry.addLine("before pressing play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---- buttons, edge detected so one press is one action ----
            if (gamepad1.options && !lastOptions) {
                imu.resetYaw();
            }
            lastOptions = gamepad1.options;

            if (gamepad1.back && !lastBack) {
                fieldCentric = !fieldCentric;
            }
            lastBack = gamepad1.back;

            // ---- sticks ----------------------------------------------
            // left_stick_y reads negative when pushed forward, so negate it.
            double scale  = gamepad1.left_bumper ? SLOW_SCALE : 1.0;
            double drive  = -gamepad1.left_stick_y * scale;   // forward
            double strafe =  gamepad1.left_stick_x * scale;   // right
            double turn   =  gamepad1.right_stick_x * scale;  // clockwise

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the driver's vector from field frame into robot frame.
            // Rotating by MINUS the heading is what converts "where the driver
            // wants it to go on the field" into "which way that is relative to
            // the robot's own nose".
            double rotStrafe = strafe;
            double rotDrive  = drive;

            if (fieldCentric) {
                rotStrafe = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
                rotDrive  = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
            }

            rotStrafe *= STRAFE_GAIN;

            // Mecanum mixing. Dividing by the largest magnitude keeps the ratio
            // between the four wheels intact when the sum would exceed 1.0 --
            // clipping each wheel separately would bend the robot's path.
            double denom = Math.max(
                    Math.abs(rotDrive) + Math.abs(rotStrafe) + Math.abs(turn), 1.0);

            leftFront.setPower( (rotDrive + rotStrafe + turn) / denom);
            leftRear.setPower(  (rotDrive - rotStrafe + turn) / denom);
            rightFront.setPower((rotDrive - rotStrafe - turn) / denom);
            rightRear.setPower( (rotDrive + rotStrafe - turn) / denom);

            // ---- telemetry --------------------------------------------
            telemetry.addData("mode", fieldCentric ? "FIELD CENTRIC" : "robot centric");
            telemetry.addData("speed", gamepad1.left_bumper ? "SLOW" : "full");
            telemetry.addData("heading", "%.1f deg", Math.toDegrees(heading));
            telemetry.addLine();
            telemetry.addLine("OPTIONS = re-zero forward");
            telemetry.addLine("BACK    = toggle field/robot centric");
            telemetry.addLine("LB      = slow mode");
            telemetry.update();
        }
    }
}
