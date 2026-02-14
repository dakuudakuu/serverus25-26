package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotActions {

    private final DcMotorEx rollers, wheel1, wheel2, transfer;
    private final Servo gate0, gate1;
    private boolean gateUp = true;
    private final HardwareMap hardwareMap;

    public RobotActions(HardwareMap hardwareMap, DcMotorEx rollers, DcMotorEx transfer, DcMotorEx wheel1, DcMotorEx wheel2, Servo gate0, Servo gate1) {
        this.rollers = rollers;
        this.wheel1 = wheel1;
        this.wheel2 = wheel2;
        this.gate0 = gate0;
        this.gate1 = gate1;
        this.transfer = transfer;
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

    public void startRollersFastLaunch() {
        rollers.setVelocity(1400);
        transfer.setVelocity(1500);
    }

    public void startRollersPickup() {
        transfer.setVelocity(0);
        rollers.setPower(0.9);
    }
    public void stopRollers() {
        transfer.setVelocity(0);
        rollers.setPower(0);
    }

    public void startWheelsFast() {
        wheel1.setVelocity(1715);
        wheel2.setVelocity(-1715);
    }

    public void stopWheels() {
        wheel1.setPower(0);
        wheel2.setPower(0);
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
        PIDFCoefficients transferPID = new PIDFCoefficients(7, 0, 0, 12.4 * (12.0 / getBatteryVoltage()));
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