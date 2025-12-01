package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RedGoalAutoPaths {

    public final Follower follower;

    public final Pose startPose = new Pose(110, 134, Math.toRadians(270));
    public final Pose launchPose = new Pose(90, 107, Math.toRadians(213));
    public final Pose startPickup1 = new Pose(92, 82, Math.toRadians(0));
    public final Pose finishPickup1 = new Pose(120, 82, Math.toRadians(0));
    public final Pose gatePose = new Pose(124, 73, Math.toRadians(0));
    public final Pose gateControlPoint = new Pose(115, 76, Math.toRadians(0));
    public final Pose startPickup2 = new Pose(92, 58, Math.toRadians(0));
    public final Pose finishPickup2 = new Pose(120, 58, Math.toRadians(0));
    public final Pose controlPoint = new Pose (90, 75, Math.toRadians(105));
    public final Pose startPickup3 = new Pose(92, 34, Math.toRadians(0));
    public final Pose finishPickup3 = new Pose(120, 34, Math.toRadians(0));
    public final Pose finishPose = new Pose(100, 76, Math.toRadians(90));
    public final Pose basePose = new Pose(106.5, 32, Math.toRadians(90));

    public final double correctHeading = Math.toDegrees(0);

    public PathChain start_launch, launch_startPickup1, startPickup1_finishPickup1, finishPickup1_launch, finishPickup1_gatePose, gatePose_launch, launch_startPickup2, startPickup2_finishPickup2, finishPickup2_launch, launch_startPickup3, startPickup3_finishPickup3, finishPickup3_launch, launch_finishPose;

    public RedGoalAutoPaths(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
    }

    public void buildPaths() {
        start_launch = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        launch_startPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startPickup1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startPickup1.getHeading())
                .build();
        startPickup1_finishPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPickup1, finishPickup1))
                .setConstantHeadingInterpolation(startPickup1.getHeading())
                .setGlobalDeceleration(0.1)
                .build();
        finishPickup1_launch = follower.pathBuilder()
                .addPath(new BezierLine(finishPickup1, launchPose))
                .setLinearHeadingInterpolation(finishPickup1.getHeading(), launchPose.getHeading())
                .setGlobalDeceleration(0.3)
                .build();
        finishPickup1_gatePose = follower.pathBuilder()
                .addPath(new BezierCurve(finishPickup1, gateControlPoint, gatePose))
                .setLinearHeadingInterpolation(finishPickup1.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration(0.1)
                .build();
        gatePose_launch = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, launchPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), launchPose.getHeading())
                .setGlobalDeceleration(0.3)
                .build();
        launch_startPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startPickup2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startPickup2.getHeading())
                .build();
        startPickup2_finishPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(startPickup2, finishPickup2))
                .setConstantHeadingInterpolation(startPickup2.getHeading())
                .setGlobalDeceleration(0.1)
                .build();
        finishPickup2_launch = follower.pathBuilder()
                .addPath(new BezierCurve(finishPickup2, controlPoint, launchPose))
                .setLinearHeadingInterpolation(finishPickup2.getHeading(), launchPose.getHeading())
                .setGlobalDeceleration(0.3)
                .build();
        launch_startPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startPickup3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startPickup3.getHeading())
                .build();
        startPickup3_finishPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(startPickup3, finishPickup3))
                .setConstantHeadingInterpolation(startPickup3.getHeading())
                .setGlobalDeceleration(0.1)
                .build();
        finishPickup3_launch = follower.pathBuilder()
                .addPath(new BezierLine(finishPickup3, launchPose))
                .setLinearHeadingInterpolation(finishPickup3.getHeading(), launchPose.getHeading())
                .setGlobalDeceleration(0.3)
                .build();
        launch_finishPose = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, finishPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), finishPose.getHeading())
                .setGlobalDeceleration(0.3)
                .build();
    }
}
