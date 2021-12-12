package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    protected HardwareMap hwMap;
    DcMotorEx carousel;
    double power = 0.0;

    public Carousel (HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init() {
        carousel = hwMap.get(DcMotorEx.class, "carousel");
    }

    public void blue() { carousel.setPower(-0.5); }

    public void inSlow() {
        carousel.setPower(-.2);
    }

    public void red() { carousel.setPower(0.5); }//speed

    public void stop() {
        carousel.setPower(0);
    }

}

