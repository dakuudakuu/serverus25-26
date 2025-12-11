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
    public final Pose pickup1 = new Pose(120, 82, Math.toRadians(0));
    public final Pose pickup1Control = new Pose(70, 72, Math.toRadians(0));
    public final Pose gatePose = new Pose(126, 73, Math.toRadians(0));
    public final Pose gateControlPoint = new Pose(115, 76, Math.toRadians(0));
    public final Pose gatePickup = new Pose(133, 57, Math.toRadians(30));
    public final Pose gatePickupControl = new Pose(90, 47, Math.toRadians(30));
    public final Pose pickup2 = new Pose(120, 58, Math.toRadians(0));
    public final Pose pickup2Control = new Pose(70, 48, Math.toRadians(0));
    public final Pose controlPoint = new Pose (90, 75, Math.toRadians(105));
    public final Pose pickup3 = new Pose(120, 34, Math.toRadians(0));
    public final Pose pickup3Control = new Pose(70, 24, Math.toRadians(0));
    public final Pose finishPose = new Pose(29, 80, Math.toRadians(213));
    public final Pose basePose = new Pose(37.5, 32, Math.toRadians(90));

    public final double correctHeading = Math.toDegrees(0);

    public PathChain start_launch, launch_pickup1, pickup1_launch, pickup1_gatePose, gatePose_launch, launch_gatePickup, gatePickup_launch, launch_pickup2, pickup2_launch, launch_pickup3, pickup3_launch, launch_finishPose;

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
        launch_pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, pickup1Control, pickup1))
                .setConstantHeadingInterpolation(pickup1.getHeading())
                .build();
        pickup1_launch = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, launchPose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), launchPose.getHeading())
                .build();
        pickup1_gatePose = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1, gateControlPoint, gatePose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), gatePose.getHeading())
                .build();
        gatePose_launch = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, launchPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), launchPose.getHeading())
                .build();
        launch_gatePickup = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, gatePickupControl, gatePickup))
                .setConstantHeadingInterpolation(gatePickup.getHeading())
                .build();
        gatePickup_launch = follower.pathBuilder()
                .addPath(new BezierCurve(gatePickup, gatePickupControl, launchPose))
                .setLinearHeadingInterpolation(gatePickup.getHeading(), launchPose.getHeading())
                .build();
        launch_pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, pickup2Control, pickup2))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .build();
        pickup2_launch = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2, controlPoint, launchPose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), launchPose.getHeading())
                .build();
        launch_pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, pickup3Control, pickup3))
                .setConstantHeadingInterpolation(pickup3.getHeading())
                .build();
        pickup3_launch = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, launchPose))
                .setLinearHeadingInterpolation(pickup3.getHeading(), launchPose.getHeading())
                .build();
        launch_finishPose = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, finishPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), finishPose.getHeading())
                .build();
    }
}
