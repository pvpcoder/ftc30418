package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A deliberately boring first autonomous: drive forward 24 inches, strafe left
 * 24 inches, come back to the start. A square-ish trip with no mechanisms.
 *
 * The point is to prove the follower actually works before you build a real
 * auto on top of it. If the robot ends within an inch or two of where it
 * started, your localization and PIDFs are good enough to build on. If it
 * spirals, oscillates, or overshoots, go back to tuning -- do NOT start adding
 * arm and claw code on top of a follower that can't drive straight.
 *
 * Run this with the robot on an open patch of field and a hand on the stop
 * button the first few times.
 */
@Autonomous(name = "First Auto (square test)", group = "Auto")
public class FirstAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Everything is relative to wherever the robot starts, so this test does
    // not depend on you having field coordinates figured out yet.
    private final Pose startPose  = new Pose(0,  0,  Math.toRadians(0));
    private final Pose forwardPose = new Pose(24, 0,  Math.toRadians(0));
    private final Pose leftPose    = new Pose(24, 24, Math.toRadians(0));
    private final Pose homePose    = new Pose(0,  0,  Math.toRadians(0));

    private PathChain goForward, goLeft, goHome;

    private void buildPaths() {
        goForward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, forwardPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading())
                .build();

        goLeft = follower.pathBuilder()
                .addPath(new BezierLine(forwardPose, leftPose))
                .setLinearHeadingInterpolation(forwardPose.getHeading(), leftPose.getHeading())
                .build();

        goHome = follower.pathBuilder()
                .addPath(new BezierLine(leftPose, homePose))
                .setLinearHeadingInterpolation(leftPose.getHeading(), homePose.getHeading())
                .build();
    }

    /**
     * The state machine. Each case starts a path and bumps the state. The
     * "!follower.isBusy()" check is how you wait for a path to finish without
     * ever calling sleep(), which would freeze the follower.
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(goForward);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(goLeft);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(goHome);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);   // done
                }
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        // These two lines are the whole engine. update() runs the follower,
        // autonomousPathUpdate() advances the state machine.
        follower.update();
        autonomousPathUpdate();

        Pose pose = follower.getPose();
        telemetry.addData("state", pathState);
        telemetry.addData("time in state", "%.1f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("x", "%.1f", pose.getX());
        telemetry.addData("y", "%.1f", pose.getY());
        telemetry.addData("heading", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }
}
