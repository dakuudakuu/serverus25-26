package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {

    private Follower follower;
    private boolean isRobotCentric = false;
    private boolean isRobotCentricPrev = false;
    private DcMotorEx rollers;
    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private DcMotor slides;
    private Servo gate0;
    private Servo gate1;
    private final Pose launchPose = PoseStorage.launchPose;
    private final Pose basePose = PoseStorage.basePose;
    private int wheelVelocity = 1725;
    public double speed;
    public boolean driverMoved;

    private final PIDF xHold = new PIDF(new PIDFCoefficients(0.1, 0, 0.01, 0));
    private final PIDF yHold = new PIDF(new PIDFCoefficients(0.1, 0, 0.01, 0));
    private final PIDF hHold = new PIDF(new PIDFCoefficients(1, 0, 0.1, 0));

    enum DriveState { TELEOP, PATHING, PID_HOLD }
    DriveState driveState = DriveState.TELEOP;

    private static final double PATH_MIN_DIST = 1.5;

    private Pose targetPose;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.endingPose);
        follower.update();
        speed = 0.8;

        rollers = hardwareMap.get(DcMotorEx.class, "rollers");
        rollers.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients rollerPID = new PIDFCoefficients(20, 0, 1, 12);
        rollers.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rollerPID);

        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        setWheelPID();

        gate0 = hardwareMap.get(Servo.class, "gate0");
        gate0.setDirection(Servo.Direction.FORWARD);
        gate1 = hardwareMap.get(Servo.class, "gate1");
        gate1.setDirection(Servo.Direction.REVERSE);

        slides = hardwareMap.get(DcMotor.class, "slides");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void stop() {
        PoseStorage.endingPose = follower.getPose();
    }

    @Override
    public void loop() {
        follower.update();
        setDriveVariables();
        setDrive();
        setWheelPID();
        moveBalls();
        moveSlides();
        updateTelemetry();
    }

    public void updateTelemetry() {
        double v = wheel1.getVelocity();
        telemetry.addData("Target", wheelVelocity);
        telemetry.addData("Velocity", v);
        telemetry.addData("Error", wheelVelocity - v);
        telemetry.update();
    }

    private void setDriveVariables() {
        boolean currentButtonState = gamepad1.a;
        if (currentButtonState && !isRobotCentricPrev) {
            isRobotCentric = !isRobotCentric;
        }
        isRobotCentricPrev = currentButtonState;

        if(gamepad1.left_bumper) {
            speed = 0.2;
        } else {
            speed = 0.8;
        }
    }

    private void enterTeleopDrive() {
        xHold.reset();
        yHold.reset();
        hHold.reset();
        follower.startTeleopDrive();
    }

    void setDrive() {
        Pose current = follower.getPose();

        double forwardRaw = -gamepad1.left_stick_y;
        double strafeRaw = -gamepad1.left_stick_x;
        double turnRaw = -gamepad1.right_stick_x;
        double dead = 0.05;
        double forward = (Math.abs(forwardRaw) > dead) ? forwardRaw * speed : 0.0;
        double strafe = (Math.abs(strafeRaw) > dead) ? strafeRaw * speed : 0.0;
        double turn = (Math.abs(turnRaw) > dead) ? turnRaw * speed : 0.0;
        driverMoved = Math.abs(forward) > 0 || Math.abs(strafe) > 0 || Math.abs(turn) > 0;

        if ((gamepad1.x || driverMoved) && driveState != DriveState.TELEOP) {
            follower.breakFollowing();
            targetPose = null;
            enterTeleopDrive();
            driveState = DriveState.TELEOP;
        }

        if (gamepad1.y) {
            startPath(launchPose);
        } else if (gamepad1.b) {
            startPath(basePose);
        }

        if (driveState == DriveState.TELEOP) {
            follower.setTeleOpDrive(forward, strafe, turn, isRobotCentric, PoseStorage.correctHeading);
            return;
        }

        if (driveState == DriveState.PATHING) {
            if (!follower.isBusy()) {
                driveState = (targetPose != null) ? DriveState.PID_HOLD : DriveState.TELEOP;
                follower.startTeleOpDrive();
            }
            return;
        }

        if (driveState == DriveState.PID_HOLD && targetPose != null) {
            double dx = targetPose.getX() - current.getX();
            double dy = targetPose.getY() - current.getY();
            double headErr = normDelta(targetPose.getHeading() - current.getHeading());
            double xPow = xHold.calculate(dx);
            double yPow = yHold.calculate(dy);
            double tPow = hHold.calculate(headErr);
            follower.setTeleOpDrive(xPow, yPow, tPow, false, PoseStorage.correctHeading);
        }
    }

    void startPath(Pose p) {
        Pose current = follower.getPose();
        double dist = Math.hypot(p.getX() - current.getX(), p.getY() - current.getY());
        if (dist > PATH_MIN_DIST) {
            targetPose = p;
            PathChain chain = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), p))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), p.getHeading())
                    .setGlobalDeceleration(0.2)
                    .build();
            follower.followPath(chain, false);
            driveState = DriveState.PATHING;
        } else {
            targetPose = p;
            driveState = DriveState.PID_HOLD;
        }
    }

    private static double normDelta(double angle) {
        while (angle >  Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private void moveBalls() {
        if(gamepad2.a) {
            rollers.setPower(0.9);
            gate0.setPosition(0.82);
            gate1.setPosition(0.82);
        } else if(gamepad2.y) {
            rollers.setVelocity(600);
            gate0.setPosition(1);
            gate1.setPosition(1);
        } else {
            rollers.setPower(0);
            gate0.setPosition(0.82);
            gate1.setPosition(0.82);
        } if (gamepad2.dpad_up) {
            wheel1.setVelocity(wheelVelocity);
            wheel2.setVelocity(-wheelVelocity);
        } else {
            wheel1.setPower(0);
            wheel2.setPower(0);
        }
    }

    private void moveSlides() {
        if(gamepad2.left_bumper) {
            slides.setPower(-0.7);
        } else if(gamepad2.right_bumper) {
            slides.setPower(0.7);
        } else {
            slides.setPower(0);
        }
    }

    public void setWheelPID() {
        PIDFCoefficients wheelPID = new PIDFCoefficients(20, 0, 5, 13 * (12 / getBatteryVoltage()));
        wheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);
        wheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);
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

    private static class PIDF {
        private final PIDFCoefficients coeffs;
        private double lastError = 0;
        private double i = 0;

        PIDF(PIDFCoefficients c) { coeffs = c; }

        double calculate(double error) {
            double p = error * coeffs.p;
            i += error * coeffs.i * 0.02;
            double d = (error - lastError) * coeffs.d * 50.0;
            double f = coeffs.f * Math.signum(error);
            lastError = error;
            return p + i + d + f;
        }
        void reset() { lastError = 0; i = 0; }
    }
}