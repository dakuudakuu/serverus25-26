package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private int wheelVelocityNormal = 1680;
    private int wheelVelocityMotif = 1855;
    private int wheelVelocityHigh = 2420;
    public double speed;
    public boolean driverMoved;
    public boolean raised = false;

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
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
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
        setRollerPID();
        moveBalls();
        moveSlides();
        telemetry.addData("Current", String.valueOf(rollers.getVelocity()));
        telemetry.addData("Target", "1500");
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
        follower.startTeleopDrive();
    }

    void setDrive() {
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
            Pose currentPose = follower.getPose();
            double dist = Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
            if(!follower.isBusy() && dist > PATH_MIN_DIST) {
                PathChain chain = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), targetPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                        .build();
                follower.followPath(chain, true);
            }
        }
    }

    void startPath(Pose p) {
        Pose current = follower.getPose();
        double dist = Math.hypot(p.getX() - current.getX(), p.getY() - current.getY());
        if (dist > PATH_MIN_DIST) {
            targetPose = p;
            PathChain chain = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), p))
                    .setConstantHeadingInterpolation(p.getHeading())
                    .setGlobalDeceleration(0.5)
                    .build();
            follower.followPath(chain, false);
            driveState = DriveState.PATHING;
        } else {
            targetPose = p;
            driveState = DriveState.PID_HOLD;
        }
    }

    private void moveBalls() {
        if (gamepad2.dpad_right) {
            wheel1.setVelocity(wheelVelocityNormal);
            wheel2.setVelocity(-wheelVelocityNormal);
            gate0.setPosition(1);
            gate1.setPosition(1);
            if(gamepad2.a) {
                rollers.setVelocity(1500);
            } else {
                rollers.setPower(0);
            }
        } else if(gamepad2.dpad_up) {
            wheel1.setVelocity(wheelVelocityMotif);
            wheel2.setVelocity(-wheelVelocityMotif);
            gate0.setPosition(1);
            gate1.setPosition(1);
            if(gamepad2.a) {
                rollers.setVelocity(600);
            } else {
                rollers.setPower(0);
            }
        } else if(gamepad2.dpad_left) {
            wheel1.setVelocity(wheelVelocityHigh);
            wheel2.setVelocity(-wheelVelocityHigh);
            gate0.setPosition(1);
            gate1.setPosition(1);
            if(gamepad2.a) {
                rollers.setVelocity(600);
            } else {
                rollers.setPower(0);
            }
        } else {
            wheel1.setPower(0);
            wheel2.setPower(0);
            gate0.setPosition(0.82);
            gate1.setPosition(0.82);
            if(gamepad2.a) {
                rollers.setPower(0.9);
            } else if (gamepad2.x) {
                rollers.setPower(-0.5);
            } else {
                rollers.setPower(0);
            }
        }
    }

    private void moveSlides() {
        if(gamepad2.left_bumper) {
            raised = true;
        } else if(gamepad2.right_bumper) {
            raised = false;
            slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slides.setPower(-0.4);
        }

        if(Math.abs(slides.getCurrentPosition() - 8000) > 50 && raised) {
            slides.setTargetPosition(8000);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(1);
        } else if(raised) {
            slides.setTargetPosition(8000);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(0.4);
        } else if (!gamepad2.right_bumper) {
            slides.setPower(0);
        }
    }

    public void setWheelPID() {
        PIDFCoefficients wheelPID = new PIDFCoefficients(20, 0, 5, 13 * (12 / getBatteryVoltage()));
        wheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);
        wheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, wheelPID);
    }

    public void setRollerPID() {
        PIDFCoefficients roller = new PIDFCoefficients(10, 0, 5, 15 * (12 / getBatteryVoltage()));
        rollers.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, roller);
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