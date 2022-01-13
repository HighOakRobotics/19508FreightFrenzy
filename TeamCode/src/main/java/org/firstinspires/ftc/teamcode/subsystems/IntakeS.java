package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeS extends Subsystem {
    DcMotorEx intake;
    double power; //?

    public double getSetpoint() {
        return power;
    }

    public void setSetpoint(double power) {
        this.power = power;
    }

    public void in() {
        intake.setPower(1.0);
    }

    public void inSlow() {
        intake.setPower(.3);
    }

    public void out() {
        intake.setPower(-0.7);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setVelocity(0, AngleUnit.DEGREES);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = 0;
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        intake.setPower(power);
    }

    @Override
    public void stop() {
        intake.setPower(0);
    }
}
