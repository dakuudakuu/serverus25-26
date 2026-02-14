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
    public final Pose launchPose = new Pose(87, 83, Math.toRadians(228));
    public final Pose pickup1 = new Pose(120, 82, Math.toRadians(0));
    public final Pose pickup1Gate = new Pose(125, 75, Math.toRadians(0));
    public final Pose gateControlPoint = new Pose(115, 76, Math.toRadians(0));
    public final Pose gatePickup = new Pose(135, 57, Math.toRadians(33));
    public final Pose gatePickupControl = new Pose(90, 60, Math.toRadians(33));
    public final Pose pickup2 = new Pose(120, 59, Math.toRadians(0));
    public final Pose pickup2Gate = new Pose(125, 63, Math.toRadians(0));
    public final Pose pickup2Control = new Pose(70, 52, Math.toRadians(0));
    public final Pose controlPoint = new Pose (90, 75, Math.toRadians(105));
    public final Pose pickup3 = new Pose(120, 34, Math.toRadians(0));
    public final Pose pickup3Control = new Pose(70, 24, Math.toRadians(0));
    public final Pose finishPose = new Pose(95, 75, Math.toRadians(228));
    public final Pose launchPoseFar = new Pose(84, 22, Math.toRadians(247));

    public final double correctHeading = Math.toDegrees(0);

    public PathChain start_launch, launch_pickup1, launch_pickup1_gate, pickup1Gate_launch, pickup1_launch, launch_gatePickup, gatePickup_launch, launch_pickup2, pickup2_launch, launch_pickup2Gate, pickup2Gate_launch, launch_pickup3, pickup3_launch, launch_finishPose;

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
                .addPath(new BezierLine(launchPose, pickup1))
                .setConstantHeadingInterpolation(pickup1.getHeading())
                .build();
        launch_pickup1_gate = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup1))
                .setConstantHeadingInterpolation(pickup1.getHeading())
                .addPath(new BezierCurve(pickup1, gateControlPoint, pickup1Gate))
                .setConstantHeadingInterpolation(pickup1.getHeading())
                .setGlobalDeceleration(2)
                .build();
        pickup1Gate_launch = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Gate, launchPose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), launchPose.getHeading())
                .build();
        pickup1_launch = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, launchPose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), launchPose.getHeading())
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
        launch_pickup2Gate = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, pickup2Control, pickup2))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .addPath(new BezierLine(pickup2, pickup2Gate))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .setGlobalDeceleration(2)
                .build();
        pickup2Gate_launch = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Gate, controlPoint, launchPose))
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
