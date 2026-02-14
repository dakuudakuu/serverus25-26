package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@TeleOp(name = "OneManTele")
public class OneManTele extends OpMode {
    private Follower follower;
    private Timer opModeTimer;
    private boolean isRobotCentric = false;
    private boolean isRobotCentricPrev = false;
    private DcMotorEx rollers;
    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx transfer;
    private Servo gate0;
    private Servo gate1;
    private final Pose launchPose = PoseStorage.launchPose;
    private final Pose gatePose = PoseStorage.gatePose;
    private final Pose launchPoseFar = PoseStorage.launchPoseFar;
    private int wheelVelocityNormal = 1715;
    private int wheelVelocityHigh = 2165;
    public boolean driverMoved;
    RevBlinkinLedDriver blink;
    double flPrev = 0, blPrev = 0, frPrev = 0, brPrev = 0;
    static final double kS = 0.08;
    static final double MAX_ACCEL = 0.32;
    static final double MAX_DECEL = 0.7;
    private boolean updatePIDF;

    List<LynxModule> allHubs;

    enum DriveState {TELEOP, PATHING, HOLD}

    DriveState driveState = DriveState.TELEOP;
    private static final double PATH_MIN_DIST = 1.5;
    private Pose targetPose;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.endingPose);
        follower.update();
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        rollers = hardwareMap.get(DcMotorEx.class, "rollers");
        rollers.setDirection(DcMotorEx.Direction.REVERSE);
        rollers.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setRollerPID();
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setWheelPID();
        gate0 = hardwareMap.get(Servo.class, "gate0");
        gate0.setDirection(Servo.Direction.FORWARD);
        gate1 = hardwareMap.get(Servo.class, "gate1");
        gate1.setDirection(Servo.Direction.REVERSE);
        blink = hardwareMap.get(RevBlinkinLedDriver.class, "blink");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setTransferPID();

        updatePIDF = false;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        follower.startTeleOpDrive();
    }

    @Override
    public void stop() {
        PoseStorage.endingPose = follower.getPose();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        setDriveVariables();
        setDrive();
        moveBalls();
        setColors();
        if (updatePIDF) {
            setTransferPID();
            setRollerPID();
            setWheelPID();
        }
        if(driveState != OneManTele.DriveState.TELEOP) {
            follower.update();
        } else {
            follower.updatePose();
        }
    }

    private void setColors() {
        if (wheel1.getVelocity() > 10) {
            blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        } else if (opModeTimer.getElapsedTimeSeconds() > 95) {
            blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        } else if (opModeTimer.getElapsedTimeSeconds() > 50) {
            blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        } else {
            blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
        }
    }

    private void setDriveVariables() {
        boolean currentButtonState = gamepad1.a;
        if (currentButtonState && !isRobotCentricPrev) {
            isRobotCentric = !isRobotCentric;
        }
        isRobotCentricPrev = currentButtonState;
    }

    void setDrive() {
        double forwardRaw = -gamepad1.left_stick_y;
        double strafeRaw = -gamepad1.left_stick_x;
        double turnRaw = -gamepad1.right_stick_x;
        double dead = 0.05;
        double forward = (Math.abs(forwardRaw) > dead) ? forwardRaw : 0.0;
        double strafe = (Math.abs(strafeRaw) > dead) ? strafeRaw : 0.0;
        double turn = (Math.abs(turnRaw) > dead) ? turnRaw : 0.0;
        driverMoved = Math.abs(forward) > 0 || Math.abs(strafe) > 0 || Math.abs(turn) > 0;
        if ((gamepad1.x || driverMoved) && driveState != DriveState.TELEOP) {
            exitPathing();
        }
        if (gamepad1.y) {
            startPath(launchPose);
        } else if (gamepad1.b) {
            startPath(gatePose);
        } else if (gamepad1.x) {
            startPath(launchPoseFar);
        }
        if (driveState == DriveState.TELEOP) {
            teleOpDrive();
            return;
        }
        if (driveState == DriveState.PATHING) {
            if (!follower.isBusy()) {
                if (targetPose != null) {
                    driveState = DriveState.HOLD;
                } else {
                    exitPathing();
                }
            }
            return;
        }
        if (driveState == DriveState.HOLD && targetPose != null) {
            Pose currentPose = follower.getPose();
            double dist = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
            if (!follower.isBusy() && dist > PATH_MIN_DIST) {
                PathChain chain = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), targetPose)).setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading()).build();
                follower.followPath(chain, true);
            }
        }
    }

    void startPath(Pose p) {
        Pose current = follower.getPose();
        double dist = Math.hypot(p.getX() - current.getX(), p.getY() - current.getY());
        if (dist > PATH_MIN_DIST) {
            targetPose = p;
            PathChain chain = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), p)).setConstantHeadingInterpolation(p.getHeading()).setBrakingStrength(0.6).setGlobalDeceleration(1.15).build();
            follower.followPath(chain, true);
            driveState = DriveState.PATHING;
        } else {
            targetPose = p;
            driveState = DriveState.HOLD;
        }
    }

    private void moveBalls() {
        if (gamepad1.right_bumper) {
            wheel1.setVelocity(wheelVelocityNormal);
            wheel2.setVelocity(-wheelVelocityNormal);
            gate0.setPosition(1);
            gate1.setPosition(1);
            if (gamepad1.left_bumper) {
                rollers.setVelocity(1400);
                transfer.setVelocity(1500);
                updatePIDF = true;
            } else {
                rollers.setVelocity(0);
                transfer.setVelocity(0);
                updatePIDF = false;
            }
        } else if (gamepad1.right_trigger > 0.5) {
            wheel1.setVelocity(wheelVelocityHigh);
            wheel2.setVelocity(-wheelVelocityHigh);
            gate0.setPosition(1);
            gate1.setPosition(1);
            if (gamepad1.left_bumper) {
                rollers.setVelocity(850);
                transfer.setVelocity(950);
                updatePIDF = true;
            } else {
                rollers.setPower(0);
                transfer.setPower(0);
                updatePIDF = false;
            }
        } else {
            updatePIDF = false;
            wheel1.setVelocity(0);
            wheel2.setVelocity(0);
            gate0.setPosition(0.82);
            gate1.setPosition(0.82);
            transfer.setVelocity(0);
            if (gamepad1.left_bumper) {
                rollers.setPower(0.9);
            } else if (gamepad1.left_trigger > 0.5) {
                rollers.setPower(-800);
            } else {
                rollers.setPower(0);
            }
        }
    }

    void exitPathing() {
        follower.breakFollowing();
        driveState = DriveState.TELEOP;
        targetPose = null;
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void teleOpDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double driveX;
        double driveY;
        if (isRobotCentric) {
            driveX = x;
            driveY = y;
        } else {
            double h = -(follower.getHeading() - PoseStorage.correctHeading);
            double cos = Math.cos(h);
            double sin = Math.sin(h);
            driveX = x * cos - y * sin;
            driveY = x * sin + y * cos;
        }
        double denominator = Math.max(Math.abs(driveY) + Math.abs(driveX) + Math.abs(rx), 1);
        double flTarget = (driveY + driveX + rx) / denominator;
        double blTarget = (driveY - driveX + rx) / denominator;
        double frTarget = (driveY - driveX - rx) / denominator;
        double brTarget = (driveY + driveX - rx) / denominator;
        flPrev = slew(flTarget, flPrev);
        blPrev = slew(blTarget, blPrev);
        frPrev = slew(frTarget, frPrev);
        brPrev = slew(brTarget, brPrev);
        flPrev = applyBraking(flTarget, flPrev);
        blPrev = applyBraking(blTarget, blPrev);
        frPrev = applyBraking(frTarget, frPrev);
        brPrev = applyBraking(brTarget, brPrev);
        frontLeft.setPower(applyKS(flPrev));
        backLeft.setPower(applyKS(blPrev));
        frontRight.setPower(applyKS(frPrev));
        backRight.setPower(applyKS(brPrev));
    }

    double slew(double target, double current) {
        double delta = target - current;
        double limit = (Math.signum(delta) == Math.signum(current)) ? MAX_ACCEL : MAX_DECEL;
        delta = Math.max(-limit, Math.min(limit, delta));
        return current + delta;
    }

    double applyBraking(double target, double current) {
        if (Math.abs(target) < 0.05 && Math.abs(current) > 0.1) {
            return -Math.signum(current) * 0.15;
        }
        return current;
    }

    double applyKS(double power) {
        if (Math.abs(power) < 0.05) return 0;
        return power + Math.signum(power) * kS;
    }

    public void setWheelPID() {
        PIDFCoefficients wheelPID = new PIDFCoefficients(20, 0, 5, 13 * (12.0 / getBatteryVoltage()));
        wheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);
        wheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);
    }

    public void setRollerPID() {
        PIDFCoefficients roller = new PIDFCoefficients(7, 0, 0, 16.4 * (12.0 / getBatteryVoltage()));
        rollers.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, roller);
    }

    public void setTransferPID() {
        PIDFCoefficients transferPID = new PIDFCoefficients(7, 0, 3, 12.4 * (12.0 / getBatteryVoltage()));
        transfer.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, transferPID);
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}