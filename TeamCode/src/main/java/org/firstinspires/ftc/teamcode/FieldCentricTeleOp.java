package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Field-centric mecanum driving.
 *
 * WHAT "FIELD CENTRIC" MEANS
 * --------------------------
 * Robot centric: push the stick forward, the robot drives out of its own nose,
 * wherever that happens to be pointing. If the robot has spun 180 degrees,
 * forward on the stick drives it back toward you.
 *
 * Field centric: push the stick forward, the robot moves in one fixed field
 * direction no matter which way it is facing. The driver stops having to
 * mentally rotate with the robot.
 *
 * HOW IT KNOWS WHICH WAY IS FORWARD
 * ---------------------------------
 * From the odometry heading, same source autonomous uses. Pedro rotates the
 * stick vector by the robot's heading before handing it to the drivetrain.
 * That means field-centric driving is only as good as the heading estimate,
 * which is why the IMU and the pods have to be right. They are: we tuned them.
 *
 * ZERO POINT
 * ----------
 * "Field forward" is whichever way the robot was pointing when this OpMode
 * was initialised. So place the robot facing away from the driver station
 * before pressing INIT, and stick-forward will mean away-from-you all match.
 *
 * If the heading ever drifts, or you pick the robot up and set it down turned,
 * press OPTIONS to re-zero: the direction the robot is facing at that moment
 * becomes the new stick-forward.
 *
 * CONTROLS (gamepad 1)
 *   left stick       translate (field frame)
 *   right stick X    turn
 *   left bumper      hold for 35% speed, for lining up
 *   OPTIONS          re-zero field forward to current heading
 *   BACK / SHARE     toggle field centric <-> robot centric
 */
@TeleOp(name = "Field Centric TeleOp", group = "Drive")
public class FieldCentricTeleOp extends OpMode {

    /** Speed while the left bumper is held. */
    private static final double SLOW_SCALE = 0.35;

    /** Turning is geared down a little; full-rate spin is hard to drive. */
    private static final double TURN_SCALE = 0.80;

    private Follower follower;

    /**
     * Rotation applied to the driver's stick vector, in radians.
     *
     * Pedro rotates the commanded vector by this before using it, so setting it
     * to the robot's current heading makes "stick forward" mean "the way the
     * robot is pointing right now". It starts at 0, which is the heading the
     * robot had at INIT.
     */
    private double fieldOffset = 0.0;

    private boolean fieldCentric = true;

    // Edge detection, so one press is one action rather than one per loop.
    private boolean lastOptions = false;
    private boolean lastBack = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Heading 0 = whichever way the robot is physically facing right now.
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Field Centric TeleOp");
        telemetry.addLine();
        telemetry.addLine("Point the robot AWAY FROM THE DRIVER STATION");
        telemetry.addLine("before pressing play. That direction becomes");
        telemetry.addLine("'forward' on the stick.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Keep the localizer ticking so heading is live the moment we start.
        follower.update();

        telemetry.addLine("Field Centric TeleOp -- ready");
        telemetry.addData("heading now", "%.1f deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // ---- re-zero -------------------------------------------------
        boolean options = gamepad1.options;
        if (options && !lastOptions) {
            fieldOffset = follower.getPose().getHeading();
        }
        lastOptions = options;

        // ---- mode toggle ---------------------------------------------
        boolean back = gamepad1.back;
        if (back && !lastBack) {
            fieldCentric = !fieldCentric;
        }
        lastBack = back;

        // ---- sticks ---------------------------------------------------
        // Gamepad Y is negative when pushed forward, and X is positive to the
        // right. Pedro wants +X forward and +Y to the robot's left, so both
        // axes get negated.
        double scale = gamepad1.left_bumper ? SLOW_SCALE : 1.0;

        double forward = -gamepad1.left_stick_y * scale;
        double strafe  = -gamepad1.left_stick_x * scale;
        double turn    = -gamepad1.right_stick_x * scale * TURN_SCALE;

        // Pedro's flag is named for the input frame: true means "these numbers
        // are in the robot's frame, rotate them by heading". false means the
        // numbers are already field-frame, which is what we want here.
        if (fieldCentric) {
            follower.setTeleOpDrive(forward, strafe, turn, false, fieldOffset);
        } else {
            follower.setTeleOpDrive(forward, strafe, turn, true);
        }

        follower.update();

        // ---- telemetry -------------------------------------------------
        double headingDeg = Math.toDegrees(follower.getPose().getHeading());

        telemetry.addData("mode", fieldCentric ? "FIELD CENTRIC" : "robot centric");
        telemetry.addData("speed", gamepad1.left_bumper ? "SLOW" : "full");
        telemetry.addLine();
        telemetry.addData("heading", "%.1f deg", headingDeg);
        telemetry.addData("stick-forward points to", "%.1f deg", Math.toDegrees(fieldOffset));
        telemetry.addData("pose", "x %.1f  y %.1f", follower.getPose().getX(), follower.getPose().getY());
        telemetry.addLine();
        telemetry.addLine("OPTIONS = re-zero forward");
        telemetry.addLine("BACK    = toggle field/robot centric");
        telemetry.addLine("LB      = slow mode");
        telemetry.update();
    }
}
