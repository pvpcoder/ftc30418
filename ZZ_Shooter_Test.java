package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ZZ_Shooter_Test", group = "Test")
public class ZZ_Shooter_Test extends OpMode {

    private DcMotor outakeLeft;
    private DcMotor outakeRight;

    // Starting power
    private double shooterPower = 1;

    @Override
    public void init() {
        outakeLeft  = hardwareMap.get(DcMotor.class, "outakeLeft");
        outakeRight = hardwareMap.get(DcMotor.class, "outakeRight");

        // Make both shoot forward (match your BaseFullTeleOp)
        outakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        outakeLeft.setPower(0);
        outakeRight.setPower(0);

        telemetry.addLine("ZZ_Shooter_Test READY");
        telemetry.addLine("A = Left | B = Right | Y = Both");
        telemetry.addLine("D-Pad Up/Down = Power +/-");
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        outakeLeft.setPower(0);
        outakeRight.setPower(0);
    }

    @Override
    public void loop() {

        // Adjust power
        if (gamepad1.dpad_up) shooterPower += 0.02;
        if (gamepad1.dpad_down) shooterPower -= 0.02;
        shooterPower = clamp(shooterPower, 0.0, 1.0);

        double leftPower = 0.0;
        double rightPower = 0.0;

        if (gamepad1.a) {
            leftPower = shooterPower;
        } else if (gamepad1.b) {
            rightPower = shooterPower;
        } else if (gamepad1.y) {
            leftPower = shooterPower;
            rightPower = shooterPower;
        }

        outakeLeft.setPower(leftPower);
        outakeRight.setPower(rightPower);

        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        outakeLeft.setPower(0);
        outakeRight.setPower(0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
