package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FullTeleOp", group="Main")
public class FullTeleOp extends BaseFullTeleOp {
    @Override
    protected double allianceHeadingOffsetRadians() {
        return 0.0; // for the auto-zero Option B approach
    }
}
