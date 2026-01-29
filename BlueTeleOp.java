package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlueTeleOp", group="Main")
public class BlueTeleOp extends BaseFullTeleOp {
    @Override
    protected double allianceHeadingOffsetRadians() {
        return -Math.PI / 2.0; // Right wall driver station
    }
}
