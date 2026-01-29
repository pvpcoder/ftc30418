package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="ZZ_IntakeFront_Test", group="Test")
public class ZZ_IntakeFront_Test extends OpMode {

    private DcMotor intakeFrontMotor;
    private DcMotor intakeBackMotor;

    // Change these if you want
    private static final double SLOW_BACK  = 0.9;
    private static final double FAST_BACK  = 1.00;

    @Override
    public void init() {
        intakeFrontMotor = hardwareMap.get(DcMotor.class, "intakeFrontMotor");
        intakeBackMotor  = hardwareMap.get(DcMotor.class, "intakeBackMotor");

        // Match your BaseFullTeleOp directions (change only if needed)
        intakeFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Make sure front is off from the start
        intakeFrontMotor.setPower(0);

        telemetry.addLine("ZZ_IntakeBack_Only_Test READY");
        telemetry.addLine("Back runs constantly");
        telemetry.addLine("Hold Y = FAST (back only)");
        telemetry.addData("SLOW Back", "%.2f", SLOW_BACK);
        telemetry.addData("FAST Back", "%.2f", FAST_BACK);
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean fast = gamepad1.y; // hold to go max (back only)

        double backPower = fast ? FAST_BACK : SLOW_BACK;

        // Front stays off
        intakeFrontMotor.setPower(0);
        intakeBackMotor.setPower(backPower);

        telemetry.addData("FAST (hold Y)", fast);
        telemetry.addData("Front Power", "%.2f", 0.0);
        telemetry.addData("Back Power", "%.2f", backPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeFrontMotor.setPower(0);
        intakeBackMotor.setPower(0);
    }
}
