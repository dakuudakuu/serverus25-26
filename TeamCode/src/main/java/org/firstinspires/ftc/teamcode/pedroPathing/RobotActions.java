package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotActions {

    private final DcMotorEx rollers, wheel1, wheel2;
    private final Servo gate0, gate1;
    private boolean gateUp = true;
    private final HardwareMap hardwareMap;

    public RobotActions(HardwareMap hardwareMap, DcMotorEx rollers, DcMotorEx wheel1, DcMotorEx wheel2, Servo gate0, Servo gate1) {
        this.rollers = rollers;
        this.wheel1 = wheel1;
        this.wheel2 = wheel2;
        this.gate0 = gate0;
        this.gate1 = gate1;
        this.hardwareMap = hardwareMap;
    }

    public void updateGate() {
        if (gateUp) {
            gate0.setPosition(0.82);
            gate1.setPosition(0.82);
        } else {
            gate0.setPosition(1);
            gate1.setPosition(1);
        }
    }

    public void raiseGate() {gateUp = true;}
    public void lowerGate() {gateUp = false;}

    public void startRollersFastLaunch() {rollers.setVelocity(1400);}
    public void startRollersSlowLaunch() {rollers.setVelocity(1100);}
    public void startRollersPickup() {rollers.setPower(1);}
    public void stopRollers() {rollers.setPower(0);}

    public void startWheelsFast() {
        wheel1.setVelocity(1680);
        wheel2.setVelocity(-1680);
    }
    public void startWheelsSlow() {
        wheel1.setVelocity(1720);
        wheel2.setVelocity(-1720);
    }
    public void stopWheels() {
        wheel1.setPower(0);
        wheel2.setPower(0);
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