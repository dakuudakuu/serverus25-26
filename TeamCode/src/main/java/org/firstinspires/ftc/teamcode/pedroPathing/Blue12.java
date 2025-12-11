package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue12")
public class Blue12 extends OpMode {

    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private RobotActions actions;
    private BlueGoalAutoPaths paths;
    private DcMotorEx rollers;
    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private Servo gate0;
    private Servo gate1;

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                actions.startWheelsSlow();
                paths.follower.followPath(paths.start_launch, true);
                setPathState(1);
                break;
            case 1:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 300) {
                    actions.startRollersSlowLaunch();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.startRollersPickup();
                    paths.follower.followPath(paths.launch_pickup1, 0.8, false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!paths.follower.isBusy()) {
                    paths.follower.followPath(paths.pickup1_gatePose, 0.6, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 700) {
                    actions.stopRollers();
                    setPathState(6);
                }
                break;
            case 6:
                if (!paths.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    actions.startWheelsSlow();
                    paths.follower.followPath(paths.gatePose_launch, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 300) {
                    actions.startRollersSlowLaunch();
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.startRollersPickup();
                    paths.follower.followPath(paths.launch_pickup2, 0.8, false);
                    setPathState(10);
                }
                break;
            case 10:
                if (!paths.follower.isBusy()) {
                    actions.startWheelsSlow();
                    paths.follower.followPath(paths.pickup2_launch, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 700) {
                    actions.stopRollers();
                    setPathState(12);
                }
                break;
            case 12:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 300) {
                    actions.startRollersSlowLaunch();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.startRollersPickup();
                    paths.follower.followPath(paths.launch_pickup3, 0.8, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!paths.follower.isBusy()) {
                    actions.startWheelsSlow();
                    paths.follower.followPath(paths.pickup3_launch, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 700) {
                    actions.stopRollers();
                    setPathState(17);
                }
                break;
            case 17:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 300) {
                    actions.startRollersSlowLaunch();
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.stopRollers();
                    paths.follower.followPath(paths.launch_finishPose, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (!paths.follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        rollers = hardwareMap.get(DcMotorEx.class, "rollers");
        rollers.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rollers.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients rollerPID = new PIDFCoefficients(20.0, 0, 1.0, 13);
        rollers.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rollerPID);

        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        PIDFCoefficients wheelPID = new PIDFCoefficients(20, 0, 5, 13);
        wheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);
        wheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);

        gate0 = hardwareMap.get(Servo.class, "gate0");
        gate0.setDirection(Servo.Direction.FORWARD);
        gate1 = hardwareMap.get(Servo.class, "gate1");
        gate1.setDirection(Servo.Direction.REVERSE);

        actions = new RobotActions(hardwareMap, rollers, wheel1, wheel2, gate0, gate1);
        paths = new BlueGoalAutoPaths(hardwareMap);
        paths.buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        paths.follower.update();
        actions.setWheelPID();
        actions.setRollerPID();
        actions.updateGate();
        pathUpdate();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        PoseStorage.endingPose = paths.follower.getPose();
        PoseStorage.correctHeading = paths.correctHeading;
        PoseStorage.launchPose = paths.launchPose;
        PoseStorage.basePose = paths.basePose;
    }
}