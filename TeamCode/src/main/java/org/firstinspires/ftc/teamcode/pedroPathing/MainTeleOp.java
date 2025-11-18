package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {

    private Follower follower;
    private boolean isRobotCentric = false;
    private boolean isRobotCentricPrev = false;
    private DcMotor rollers;
    private DcMotorEx wheel1;
    private DcMotorEx wheel2;
    private DcMotor slides;
    private final Pose launchPose = PoseStorage.launchPose;
    private boolean returningFromPath = false;
    private VoltageSensor voltageSensor;
    public double speed;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.endingPose);
        follower.update();
        speed = 0.8;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        rollers = hardwareMap.get(DcMotor.class, "rollers");
        rollers.setDirection(DcMotor.Direction.REVERSE);
        wheel1 = hardwareMap.get(DcMotorEx.class, "wheel1");
        wheel1.setDirection(DcMotorEx.Direction.REVERSE);
        wheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheel2 = hardwareMap.get(DcMotorEx.class, "wheel2");
        wheel2.setDirection(DcMotorEx.Direction.REVERSE);
        wheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides = hardwareMap.get(DcMotor.class, "slides");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void stop() {
        PoseStorage.endingPose = new Pose(0, 0, 0);
    }

    @Override
    public void loop() {
        follower.update();
        setDriveVariables();
        setDrive();
        moveBalls();
        moveSlides();
        telemetry.addData("Final X", follower.getPose().getX());
        telemetry.addData("Final Y", follower.getPose().getY());
        telemetry.addData("Final Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void setDriveVariables() {
        boolean currentButtonState = gamepad1.a;
        if (currentButtonState && !isRobotCentricPrev) {
            isRobotCentric = !isRobotCentric;
        }
        isRobotCentricPrev = currentButtonState;

        if(gamepad1.left_bumper) {
            speed = 0.5;
        } else {
            speed = 0.8;
        }
    }

    private void setDrive() {
        if (!follower.isBusy() && gamepad1.y) {
            PathChain launchPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), launchPose))
                    .setLinearHeadingInterpolation(follower.getHeading(), launchPose.getHeading())
                    .setGlobalDeceleration(0.3)
                    .build();
            follower.followPath(launchPath, false);
            returningFromPath = true;
        }
        else if (returningFromPath && !follower.isBusy()) {
            follower.startTeleopDrive();
            returningFromPath = false;
        }
        else if (!follower.isBusy()) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed,
                    -gamepad1.right_stick_x * speed,
                    isRobotCentric,
                    PoseStorage.correctHeading
            );
        }
    }

    private void moveBalls() {
        if(gamepad2.a) {
            rollers.setPower(0.9);
            wheel1.setPower(-0.9);
            wheel2.setPower(0.9);
        } else if(gamepad2.dpad_up) {
            wheel1.setVelocity(3000);
            wheel2.setVelocity(-3000);
        } else if (gamepad2.x) {
            rollers.setPower(-0.5);
            wheel1.setPower(-0.5);
            wheel2.setPower(0.5);
        } else {
            rollers.setPower(0);
            wheel1.setPower(0);
            wheel2.setPower(0);
        }
        if(gamepad2.y) {
            rollers.setPower(0.7);
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
}
