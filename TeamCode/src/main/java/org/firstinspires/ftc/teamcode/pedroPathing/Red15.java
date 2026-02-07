package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red15")
public class Red15 extends OpMode {

    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private RobotActions actions;
    private RedGoalAutoPaths paths;
    private DcMotorEx rollers;
    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private DcMotorEx transfer;
    private Servo gate0;
    private Servo gate1;
    private RevBlinkinLedDriver blink;

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                actions.startWheelsFast();
                paths.follower.followPath(paths.start_launch, true);
                blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                setPathState(1);
                break;
            case 1:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 200) {
                    actions.startRollersFastLaunch();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.startRollersPickup();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
                    paths.follower.followPath(paths.launch_pickup2, false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!paths.follower.isBusy()) {
                    actions.startWheelsFast();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                    paths.follower.followPath(paths.pickup2_launch, true);
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
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 200) {
                    actions.startRollersFastLaunch();
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.startRollersPickup();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
                    paths.follower.followPath(paths.launch_gatePickup, 0.9, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (!paths.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                    actions.startWheelsFast();
                    actions.stopRollers();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                    paths.follower.followPath(paths.gatePickup_launch, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 200) {
                    actions.startRollersFastLaunch();
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.startRollersPickup();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
                    paths.follower.followPath(paths.launch_pickup1, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!paths.follower.isBusy()) {
                    actions.startWheelsFast();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                    paths.follower.followPath(paths.pickup1_launch, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 700) {
                    actions.stopRollers();
                    setPathState(15);
                }
                break;
            case 15:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 200) {
                    actions.startRollersFastLaunch();
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.startRollersPickup();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
                    paths.follower.followPath(paths.launch_pickup3, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (!paths.follower.isBusy()) {
                    actions.startWheelsFast();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                    paths.follower.followPath(paths.pickup3_launch, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 700) {
                    actions.stopRollers();
                    setPathState(20);
                }
                break;
            case 20:
                if (!paths.follower.isBusy()) {
                    actions.lowerGate();
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTime() > 200) {
                    actions.startRollersFastLaunch();
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    actions.raiseGate();
                    actions.stopWheels();
                    actions.stopRollers();
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
                    paths.follower.followPath(paths.launch_finishPose, true);
                    setPathState(23);
                }
                break;
            case 23:
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

        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gate0 = hardwareMap.get(Servo.class, "gate0");
        gate0.setDirection(Servo.Direction.FORWARD);
        gate1 = hardwareMap.get(Servo.class, "gate1");
        gate1.setDirection(Servo.Direction.REVERSE);

        blink = hardwareMap.get(RevBlinkinLedDriver.class, "blink");
        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        actions = new RobotActions(hardwareMap, rollers, transfer, wheel1, wheel2, gate0, gate1);
        paths = new RedGoalAutoPaths(hardwareMap);
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
        actions.setTransferPID();
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
        PoseStorage.launchPoseFar = paths.launchPoseFar;
    }
}