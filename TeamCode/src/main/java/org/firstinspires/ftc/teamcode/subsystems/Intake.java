package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    protected HardwareMap hwMap;
    DcMotorEx intake;
    double power = 0.0;

    public Intake (HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init() {
        intake = hwMap.get(DcMotorEx.class, "intake");
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

    public void stop() {
        intake.setPower(0);
    }

}

