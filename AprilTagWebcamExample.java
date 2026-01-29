package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vision.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "ZZ_AprilTag_Test", group = "Test")
public class  AprilTagWebcamExample extends OpMode {
    // ===== Drive motors =====
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private double startTime;
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        startTime= getRuntime();
        double y = 0;
        double x = -1;
        double rx = 0;

//        if (Math.abs(y) < DEAD) y = 0;
//        if (Math.abs(x) < DEAD) x = 0;
//        if (Math.abs(rx) < DEAD) rx = 0;

//        if (gamepad1.start) imu.resetYaw();

//        double yaw = imu.getRobotYawPitchRollAngles()
//                .getYaw(AngleUnit.RADIANS);

//        double heading = yaw + allianceHeadingOffsetRadians();

//        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
//        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

//        double denom = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
        setDrive(
                -1,
                1,
                1,
                -1
        );
    }

    @Override
    public void loop() {
        if (getRuntime()-startTime>5)
        {

            setDrive(0, 0, 0, 0);
        }
    }
    private void setDrive(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

}
