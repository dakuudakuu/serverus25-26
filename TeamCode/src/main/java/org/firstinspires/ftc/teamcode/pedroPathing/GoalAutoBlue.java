package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(name = "GoalAutoBlue")
public class GoalAutoBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private DcMotorEx rollers;
    private DcMotorEx wheel1;
    private DcMotorEx wheel2;

    private Servo gate0;
    private Servo gate1;

    private VoltageSensor voltageSensor;

    private final double speed = 1;

    private boolean gateUp = true;

    private final Pose startPose = new Pose(34, 134, Math.toRadians(270));
    private final Pose launchPose = new Pose(54, 107, Math.toRadians(328));
    private final Pose startGrab1 = new Pose(52, 84, Math.toRadians(180));
    private final Pose finishGrab1 = new Pose(29, 84, Math.toRadians(180));
    private final Pose startGrab2 = new Pose(52, 60, Math.toRadians(180));
    private final Pose finishGrab2 = new Pose(29, 60, Math.toRadians(180));
    private final Pose controlPoint = new Pose (54, 75, Math.toRadians(75));
    private final Pose finishPose = new Pose(44, 82, Math.toRadians(90));

    private PathChain launch1, toPickUp1, pickUp1, launch2, toPickUp2, pickUp2, launch3, leave;

    public void buildPaths() {
        launch1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        toPickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startGrab1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startGrab1.getHeading())
                .build();
        pickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(startGrab1, finishGrab1))
                .setConstantHeadingInterpolation(startGrab1.getHeading())
                .setGlobalDeceleration(0.2)
                .build();
        launch2 = follower.pathBuilder()
                .addPath(new BezierLine(finishGrab1, launchPose))
                .setLinearHeadingInterpolation(finishGrab1.getHeading(), launchPose.getHeading())
                .setGlobalDeceleration(0.3)
                .build();
        toPickUp2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startGrab2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startGrab2.getHeading())
                .build();
        pickUp2 = follower.pathBuilder()
                .addPath(new BezierLine(startGrab2, finishGrab2))
                .setConstantHeadingInterpolation(startGrab2.getHeading())
                .setGlobalDeceleration(0.2)
                .build();
        launch3 = follower.pathBuilder()
                .addPath(new BezierCurve(finishGrab2, controlPoint, launchPose))
                .setLinearHeadingInterpolation(finishGrab2.getHeading(), launchPose.getHeading())
                .setGlobalDeceleration(0.3)
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, finishPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), finishPose.getHeading())
                .setGlobalDeceleration(0.5)
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        buildPaths();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        rollers = hardwareMap.get(DcMotorEx.class, "rollers");
        rollers.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rollers.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients rollerPID = new PIDFCoefficients(20.0, 0, 1.0, 12.0);
        rollers.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rollerPID);
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel1.setDirection(DcMotorEx.Direction.REVERSE);
        wheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorEx.Direction.REVERSE);
        wheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gate0 = hardwareMap.get(Servo.class, "gate0");
        gate0.setDirection(Servo.Direction.FORWARD);
        gate1 = hardwareMap.get(Servo.class, "gate1");
        gate1.setDirection(Servo.Direction.REVERSE);
    }

    public void pathUpdate() {
        switch(pathState) {
            case 0:
                startWheels();
                raiseGate();
                follower.followPath(launch1, speed, true);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 2 & !follower.isBusy()) {
                    lowerGate();
                    startRollersLaunch();
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 1.25) {
                    raiseGate();
                    stopWheels();
                    stopRollers();
                    follower.followPath(toPickUp1, speed, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    startRollersPickup();
                    follower.followPath(pickUp1, speed, false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    stopRollers();
                    startWheels();
                    follower.followPath(launch2, speed, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 2.5 & !follower.isBusy()) {
                    lowerGate();
                    startRollersLaunch();
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 1.25) {
                    raiseGate();
                    stopWheels();
                    stopRollers();
                    follower.followPath(toPickUp2, speed, false);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    startRollersPickup();
                    follower.followPath(pickUp2, speed, false);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    stopRollers();
                    startWheels();
                    follower.followPath(launch3, speed, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 2.5 & !follower.isBusy()) {
                    lowerGate();
                    startRollersLaunch();
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 1.25) {
                    raiseGate();
                    stopWheels();
                    stopRollers();
                    follower.followPath(leave, speed, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        updateGate();
        pathUpdate();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void updateGate() {
        if(gateUp) {
            gate0.setPosition(0);
            gate1.setPosition(0);
        } else {
            gate0.setPosition(1);
            gate1.setPosition(1);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void startWheels() {
        wheel1.setVelocity(2500);
        wheel2.setVelocity(-2500);
    }

    public void stopWheels() {
        wheel1.setPower(0);
        wheel2.setPower(0);
    }

    public void raiseGate() {
        gateUp = true;
    }

    public void lowerGate() {
        gateUp = false;
    }

    public void startRollersLaunch() {
        rollers.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rollers.setVelocity(400);
    }

    public void startRollersPickup() {
        rollers.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rollers.setPower(0.8);
    }

    public void stopRollers() {
        rollers.setPower(0);
    }

    @Override
    public void stop() {
        PoseStorage.endingPose = follower.getPose();
        PoseStorage.correctHeading = Math.toRadians(180);
        PoseStorage.launchPose = launchPose;
    }
}