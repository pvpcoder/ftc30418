package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RedTeleOp", group="Main")
public class RedTeleOp extends BaseFullTeleOp {
    @Override
    protected double allianceHeadingOffsetRadians() {
        return Math.PI / 2.0; // Left wall driver station
    }
}
