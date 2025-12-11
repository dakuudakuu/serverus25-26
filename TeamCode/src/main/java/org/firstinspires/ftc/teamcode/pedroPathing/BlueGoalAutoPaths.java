package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BlueGoalAutoPaths {

    public final Follower follower;

    public final Pose startPose = new Pose(34, 134, Math.toRadians(270));
    public final Pose launchPose = new Pose(54, 107, Math.toRadians(328));
    public final Pose pickup1 = new Pose(24, 85, Math.toRadians(180));
    public final Pose pickup1Control = new Pose(74, 75, Math.toRadians(180));
    public final Pose gatePose = new Pose(16, 74, Math.toRadians(180));
    public final Pose gateControlPoint = new Pose(29, 78, Math.toRadians(180));
    public final Pose gatePickup = new Pose(12, 57, Math.toRadians(150));
    public final Pose gatePickupControl = new Pose(54, 47, Math.toRadians(150));
    public final Pose pickup2 = new Pose(24, 61, Math.toRadians(180));
    public final Pose pickup2Control = new Pose(74, 51, Math.toRadians(180));
    public final Pose controlPoint = new Pose (54, 75, Math.toRadians(75));
    public final Pose pickup3 = new Pose(24, 34, Math.toRadians(180));
    public final Pose pickup3Control = new Pose(74, 24, Math.toRadians(180));
    public final Pose finishPose = new Pose(44, 76, Math.toRadians(328));
    public final Pose basePose = new Pose(106.5, 32, Math.toRadians(90));

    public final double correctHeading = Math.toRadians(180);

    public PathChain start_launch, launch_pickup1, pickup1_launch, pickup1_gatePose, gatePose_launch, launch_gatePickup, gatePickup_launch, launch_pickup2, pickup2_launch, launch_pickup3, pickup3_launch, launch_finishPose;

    public BlueGoalAutoPaths(HardwareMap hardwareMap) {
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