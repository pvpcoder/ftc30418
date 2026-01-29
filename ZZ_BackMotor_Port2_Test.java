package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ZZ_BackMotor_Port2_Test", group = "Test")
public class ZZ_BackMotor_Port2_Test extends OpMode {

    private DcMotor intakeBackMotor;

    @Override
    public void init() {
        // This MUST match the Control Hub motor name
        intakeBackMotor = hardwareMap.get(DcMotor.class, "intakeBackMotor");
        intakeBackMotor.setPower(0);

        telemetry.addLine("TEST: Control Hub Port 2");
        telemetry.addLine("Hold Y to run motor");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            intakeBackMotor.setPower(1.0);
        } else {
            intakeBackMotor.setPower(0);
        }

        telemetry.addData("Motor Power", gamepad1.y ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeBackMotor.setPower(0);
    }
}
