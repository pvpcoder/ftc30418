package org.firstinspires.ftc.teamcode.turret;

/**
 * Pure coordinate maths. No hardware, no state, no SDK types.
 *
 * Everything here is a static function of its arguments, which is what makes
 * it testable off the robot (see GeometrySelfTest).
 *
 * ANGLE CONVENTION, used everywhere in this package:
 *   radians, counterclockwise positive, seen from above, 0 = robot +X
 *
 * FRAMES, innermost first:
 *
 *   CAMERA   what the lens sees. The SDK hands us ftcPose with
 *            x = RIGHT, y = FORWARD, z = UP. Different handedness from ours,
 *            so fromFtcPose() converts it once and nothing else touches it.
 *
 *   CAMTURRET  the camera turret's rotating frame. The lens sits at a fixed
 *            offset inside it and may be cocked at a fixed yaw.
 *
 *   ROBOT    X forward, Y left, origin at the centre of rotation. The camera
 *            turret's pivot sits at a fixed offset here, and the turret's
 *            rotation is what relates CAMTURRET to ROBOT.
 *
 *   FIELD    Pedro's frame. Related to ROBOT by the robot's pose.
 *
 * The whole point of the chain is that a rotating camera does not just change
 * where it LOOKS, it changes where the lens IS. Both effects are handled.
 */
public final class Geometry {

    private Geometry() {}

    /** A 2D point. Meaning depends on which frame you are in. */
    public static final class Vec {
        public final double x, y;
        public Vec(double x, double y) { this.x = x; this.y = y; }
        public Vec plus(Vec o)  { return new Vec(x + o.x, y + o.y); }
        public Vec minus(Vec o) { return new Vec(x - o.x, y - o.y); }
        public double norm()    { return Math.hypot(x, y); }
        @Override public String toString() { return String.format("(%.2f, %.2f)", x, y); }
    }

    /** Rotate a vector counterclockwise by theta radians. */
    public static Vec rotate(Vec v, double theta) {
        double c = Math.cos(theta), s = Math.sin(theta);
        return new Vec(v.x * c - v.y * s, v.x * s + v.y * c);
    }

    /** Wrap to (-PI, PI]. */
    public static double normalize(double radians) {
        double a = radians % (2 * Math.PI);
        if (a <= -Math.PI) a += 2 * Math.PI;
        if (a > Math.PI)   a -= 2 * Math.PI;
        return a;
    }

    /** Smallest signed angle to get from a to b. */
    public static double angleDelta(double a, double b) { return normalize(b - a); }

    /**
     * Convert the SDK's ftcPose x/y into our camera frame.
     *
     * SDK: x is RIGHT of the optical axis, y is FORWARD along it.
     * Ours: x is FORWARD, y is LEFT.
     *
     * So forward comes straight across, and left is the negation of right.
     * This is the ONLY place that conversion happens.
     */
    public static Vec fromFtcPose(double ftcPoseX, double ftcPoseY) {
        return new Vec(ftcPoseY, -ftcPoseX);
    }

    /**
     * Tag position in CAMERA frame -> tag position in ROBOT frame.
     *
     * @param tagInCamera     tag as seen by the lens, our convention
     * @param camMountYaw     lens optical axis yaw within the turret frame
     * @param lensOffset      lens position relative to the turret pivot,
     *                        expressed in the turret's rotating frame
     * @param camTurretAngle  the camera turret's angle in the ROBOT frame AT
     *                        THE MOMENT THE FRAME WAS CAPTURED, not now
     * @param camPivot        turret pivot position in the ROBOT frame
     */
    public static Vec tagToRobotFrame(Vec tagInCamera,
                                      double camMountYaw,
                                      Vec lensOffset,
                                      double camTurretAngle,
                                      Vec camPivot) {
        // Into the turret's rotating frame: account for how the lens is
        // aimed within it, then for where the lens sits within it.
        Vec inTurret = rotate(tagInCamera, camMountYaw).plus(lensOffset);

        // Into the robot frame: the turret's rotation, then its pivot.
        return rotate(inTurret, camTurretAngle).plus(camPivot);
    }

    /** Tag in ROBOT frame -> tag in FIELD frame, using the pose at capture. */
    public static Vec robotToField(Vec inRobot, Vec robotPos, double robotHeading) {
        return rotate(inRobot, robotHeading).plus(robotPos);
    }

    /** Tag in FIELD frame -> tag in ROBOT frame, using the pose right now. */
    public static Vec fieldToRobot(Vec inField, Vec robotPos, double robotHeading) {
        return rotate(inField.minus(robotPos), -robotHeading);
    }

    /**
     * The angle, in the ROBOT frame, from a turret's pivot to the target.
     *
     * This is the line the user asked for, and it is the reason we never copy
     * one turret's angle to the other: each turret subtracts ITS OWN pivot,
     * so two turrets in different places get different answers, and the
     * difference between them correctly shrinks as the target gets further
     * away.
     */
    public static double desiredYaw(Vec targetInRobot, Vec pivot) {
        return Math.atan2(targetInRobot.y - pivot.y, targetInRobot.x - pivot.x);
    }

    /** Distance from a turret's pivot to the target, for telemetry. */
    public static double rangeFromPivot(Vec targetInRobot, Vec pivot) {
        return targetInRobot.minus(pivot).norm();
    }
}
