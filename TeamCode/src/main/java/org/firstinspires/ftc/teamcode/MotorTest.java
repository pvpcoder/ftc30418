package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * STEP 1. Run this before anything else.
 *
 * One button, one motor. This tells you two things you cannot find out any
 * other way: whether the name in your Robot Configuration matches the motor
 * you think it does, and which way that motor spins.
 *
 * HOW TO USE
 *   A  -> leftFront
 *   B  -> rightFront
 *   X  -> leftRear
 *   Y  -> rightRear
 *
 * Put the robot up on a block so the wheels spin free. Hold a button. Watch
 * which wheel turns and which way the TOP of that wheel travels.
 *
 * WHAT YOU WANT
 *   Every wheel's top surface should travel FORWARD (toward the front of the
 *   robot) when you hold its button. All four. No exceptions.
 *
 * WHAT TO FIX
 *   Wrong wheel spins   -> your Robot Configuration has the names on the
 *                          wrong ports. Fix it on the Driver Station.
 *   Wheel spins backward -> add that motor to the REVERSED list at the bottom
 *                          of setDirections() below, and remember it, because
 *                          the same direction has to go into Constants.java.
 *
 * Also watch the ENCODER numbers at the bottom of the telemetry. If an
 * odometry pod is plugged into that motor's port, the number you see is the
 * POD counting, not the motor. Push the robot by hand and check that three of
 * the four numbers change. That confirms all three pods are alive and wired.
 */
@TeleOp(name = "1. Motor Test", group = "Setup")
public class MotorTest extends OpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private static final double TEST_POWER = 0.3;

    @Override
    public void init() {
        // These four strings MUST exactly match your Robot Configuration.
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        setDirections();

        // IMPORTANT: RUN_WITHOUT_ENCODER, not RUN_USING_ENCODER.
        // Your odometry pods occupy the encoder ports, so the encoder on a
        // given port is NOT measuring that motor. Asking the motor to close a
        // loop on a pod's counts makes it fight itself.
        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Wheels off the ground. Hold A / B / X / Y.");
        telemetry.update();
    }

    private void setDirections() {
        // CONFIRMED ON OUR ROBOT, 2026-08-28:
        // With everything set FORWARD, the right side ran correctly and the
        // left side ran backwards. So the left side is reversed here.
        //
        // These same four directions are already in MecanumTeleOp.java and
        // Constants.java. If you ever swap a motor or rewire a port, change
        // the directions in ALL THREE files or autonomous and TeleOp will
        // disagree with each other.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double lf = gamepad1.a ? TEST_POWER : 0;
        double rf = gamepad1.b ? TEST_POWER : 0;
        double lr = gamepad1.x ? TEST_POWER : 0;
        double rr = gamepad1.y ? TEST_POWER : 0;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);

        telemetry.addLine("A=leftFront  B=rightFront  X=leftRear  Y=rightRear");
        telemetry.addLine("Every wheel top should travel FORWARD.");
        telemetry.addLine();
        telemetry.addData("running", lf + rf + lr + rr > 0 ? "yes" : "-");
        telemetry.addLine();
        telemetry.addLine("--- encoder counts (these are your PODS) ---");
        telemetry.addData("port leftFront", leftFront.getCurrentPosition());
        telemetry.addData("port rightFront", rightFront.getCurrentPosition());
        telemetry.addData("port leftRear", leftRear.getCurrentPosition());
        telemetry.addData("port rightRear", rightRear.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
