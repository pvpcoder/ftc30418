package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // ------------------------------------------------------------------
    // 1. FOLLOWER CONSTANTS
    // ------------------------------------------------------------------
    public static FollowerConstants followerConstants = new FollowerConstants()

            .mass(5.17)                                        // measured 2026-08-28: 5.170953 kg
                                                               // RE-WEIGH once arms/intake are mounted.

            // Zero-power deceleration, measured 2026-09-18. Both depend on
            // weight and on how much the wheels grip, so re-run both once the
            // arm and intake are mounted.
            .forwardZeroPowerAcceleration(-52.58)
            .lateralZeroPowerAcceleration(-59.11)

            // From the PIDF tuners. The defaults below are a reasonable
            // starting point for a ~12 kg mecanum bot but will need adjusting.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.015))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.005, 0.0006))

            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.1, 0.0005))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01))
            .drivePIDFSwitch(15)

            .centripetalScaling(0.0005)

            // SAFETY NET, added 2026-09-18.
            // Pedro arms its stuck-detection timer only while path progress t
            // is between stuckTValueLow (0.1) and stuckTValueHigh, which
            // defaults to 0.8. Our square test stalled at t = 0.9917, past that
            // window, so the timer never armed and the robot sat there forever.
            // Lifting the robot by hand nudged the pose enough to finish.
            //
            // Widening the top of the window to 0.999 means a robot that stops
            // moving near the end of a path gives up after 500 ms instead of
            // hanging. Worst case it ends a path a fraction early, which is
            // always better than an autonomous that freezes mid-match.
            .stuckTValueHigh(0.999)

            // PredictiveBrakingTuner, 2026-09-18.
            //
            // WATCH THE ARGUMENT ORDER. The tuner prints kQuadratic first, but
            // the constructor takes (P, linearBraking, quadraticFriction).
            //   tuner kLinear    = 0.042345   -> 2nd argument
            //   tuner kQuadratic = 0.0023819  -> 3rd argument
            //
            // P is NOT from the tuner. It's the one value you tune by hand,
            // using Tuning -> Tests -> Line. Start at 0.15 (Pedro's default)
            // and raise it as high as it goes without the robot jittering.
            // Typical range is 0.1 to 0.3.
            //
            // Calling this setter also flips usePredictiveBraking to true, so
            // the drive PIDF coefficients above are no longer what stops the
            // robot at the end of a path. This does it instead.
            // P history (first argument):
            //   0.15 - stopped accurately but vibrated noticeably at the stop
            //   0.10 - trying this
            .predictiveBrakingCoefficients(
                    new PredictiveBrakingCoefficients(0.10, 0.042345, 0.0023819));


    // ------------------------------------------------------------------
    // 2. DRIVETRAIN
    // ------------------------------------------------------------------
    // The four motor names MUST match your Robot Configuration on the Driver
    // Station exactly, including capitalisation.
    //
    // Directions: the usual mecanum result is left side REVERSE, right side
    // FORWARD, but that depends on how your motors are mounted. Verify in
    // TeleOp before you touch autonomous -- pushing the left stick forward
    // must drive the robot forward, not spin it.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            // Velocity tuners, re-run 2026-09-18 after fixing the wheels that
            // were making the robot veer right. Both jumped a lot, because the
            // old runs travelled diagonally and the tuner only ever saw the
            // component along the measured axis.
            //   forward: 63.31 -> 80.39  (+27%)
            //   strafe:  61.70 -> 64.84
            // Strafe is now 80.7% of forward, normal for mecanum. It was 97.5%
            // before, which was the veer flattening the difference.
            .xVelocity(80.39)
            .yVelocity(64.84);


    // ------------------------------------------------------------------
    // 3. LOCALIZER -- three pods + Control Hub IMU
    // ------------------------------------------------------------------
    // WIRING NOTE: odometry pods plug into MOTOR ENCODER ports. A Control Hub
    // has four, and your four drive motors already occupy them. So each pod
    // physically shares a port with a drive motor, and in code you refer to
    // the pod by THAT MOTOR'S name. That is why the names below are motor
    // names and not "leftPod" etc. If you have an Expansion Hub you can use
    // its ports instead and name them accordingly.
    //
    // POD PLACEMENT:
    //   left pod    -- parallel to forward, on the left side
    //   right pod   -- parallel to forward, on the right side
    //   strafe pod  -- perpendicular, pointing sideways
    //
    // OFFSETS are measured in inches from the robot's centre of rotation
    // (roughly the middle of the drivetrain) to the CONTACT PATCH of the pod
    // wheel. Sign convention: +X is forward, +Y is to the robot's left.
    //   leftPodY   -> +, because the left pod sits to the left of centre
    //   rightPodY  -> -, because the right pod sits to the right of centre
    //   strafePodX -> + if the strafe pod is ahead of centre, - if behind
    // Measure these with a tape measure to the nearest 1/8 inch. Getting them
    // wrong is the single most common reason odometry "drifts on turns".
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()

            // goBILDA odometry pod: 48 mm wheel, 2000 ticks per revolution.
            //   (pi * 48 mm) / 25.4 mm-per-inch / 2000 ticks = 0.0029685 in/tick
            // This is a STARTING value. The Forward/Lateral/Turn tuners will
            // give you a multiplier to correct it -- use their output.
            // ForwardTuner, 2026-08-28, average of two 48 in runs.
            //   run 1: 0.002990195  (odometry read 47.652)
            //   run 2: 0.003038     (odometry read 47.23)
            // 1.6% spread between runs, which is push accuracy, not the pods.
            .forwardTicksToInches(0.003014098)
            // LateralTuner, 2026-08-28. Pushed 48.0 in left, odometry read
            // 48.066. Only a 0.14% correction, so the strafe pod was already
            // essentially perfect.
            .strafeTicksToInches(0.002964424)
            // TurnTuner, 2026-08-28. One full CCW revolution: target 6.28319 rad,
            // odometry read 6.2996, a 0.26% overshoot.
            .turnTicksToInches(0.0029607)

            // MEASURED 2026-08-28
            // Parallel pods are 9.75 in apart, so each sits 4.875 in from centre.
            // Sideways pod is 4.5 in BEHIND the middle, hence negative.
            .leftPodY(4.875)
            .rightPodY(-4.875)
            .strafePodX(-4.5)

            // OUR WIRING, CONFIRMED 2026-08-28
            // Motor ports:   0=leftFront  1=leftRear  2=rightFront  3=rightRear
            // Encoder ports: 0=empty      1=LEFT pod  2=SIDEWAYS pod 3=RIGHT pod
            //
            // A pod is read under the name of the motor on its encoder port,
            // so the three names below are motor names. That is correct.
            .leftEncoder_HardwareMapName("leftRear")     // encoder port 1
            .rightEncoder_HardwareMapName("rightRear")   // encoder port 3
            .strafeEncoder_HardwareMapName("rightFront") // encoder port 2

            // Flip any of these to Encoder.REVERSE if that pod counts backwards.
            // Test: push the robot forward by hand, both parallel pods must
            // count UP. Push it left, the strafe pod must count UP.
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)

            .IMU_HardwareMapName("imu")

            // Change these two to match how your Control Hub is physically
            // bolted to the robot. Wrong orientation = heading is nonsense.
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));


    // ------------------------------------------------------------------
    // 4. PATH CONSTRAINTS
    // ------------------------------------------------------------------
    // Leave these alone until everything else works.
    // LOOSENED 2026-09-18 after the square test hung in state 2.
    //
    // tValueConstraint was 0.995, which on a 24 in path means the robot must
    // finish within 0.12 in of the endpoint before Pedro marks the path done.
    // Leg 1 stopped at 23.9 (t = 0.9958) and just passed. Leg 2 stopped at
    // 23.8 (t = 0.9917) and failed, so isBusy never cleared and the third path
    // was never commanded. The timeout does not rescue this: in Follower.java
    // that check sits after the parametric-end gate, not beside it.
    //
    // 0.98 allows 0.48 in on a 24 in path, which is a sane FTC tolerance.
    // Translational and heading were also absurdly tight for a real robot.
    public static PathConstraints pathConstraints = new PathConstraints(
            0.98,    // t-value end constraint      (was 0.995)
            0.1,     // velocity end constraint
            0.5,     // translational end constraint, inches  (was 0.1)
            0.02,    // heading end constraint, radians ~1.1 deg  (was 0.009)
            100,     // path end timeout, ms        (was 50; Pedro's default)
            1.25,    // braking strength
            10,      // BEZIER_CURVE_SEARCH_LIMIT
            1        // braking start
    );


    // ------------------------------------------------------------------
    // 5. BUILD THE FOLLOWER
    // ------------------------------------------------------------------
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
