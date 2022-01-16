package org.firstinspires.ftc.teamcode.opmodes.competition;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanumBasic;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SwingArm;

@Autonomous(name = "Red Left", group = "Quackology")
@Disabled

public class AutoRL extends LinearOpMode {
    private EyeOpenCV eye;
    private int pos;
    private DriveTrainMecanumBasic drive;
    private SwingArm arm;
    private Carousel carousel;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        drive = new DriveTrainMecanumBasic(hardwareMap, telemetry);
        drive.resetAngle();

        carousel = new Carousel(hardwareMap);
        carousel.init();
        arm = new SwingArm(hardwareMap);
        arm.init();

        eye = new org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV(0, 140, 280, 320, 240);
        eye.init(hardwareMap, telemetry);
        pos = -9999;

        while (!isStarted()) {
            if (eye.getAnalysis() == org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV.DuckPosition.RIGHT) {
                pos = 1;
                telemetry.addData("right ", "%d", pos);
            } else if (eye.getAnalysis() == org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV.DuckPosition.LEFT) {
                pos = -1;
                telemetry.addData("left ", "%d", pos);
            } else if (eye.getAnalysis() == org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV.DuckPosition.CENTER) {
                pos = 0;
                telemetry.addData("center ", "%d", pos);
            } else {
                pos = -9999;
                telemetry.addData("none ", "%d", pos);
            }
            telemetry.update();
        }

        runtime.reset();
        drive.forwardByInch(19, .5); //drive foward

        arm.mWristLift(); //lift in the middle
        sleep(800);
        arm.left();
        sleep(800);

        if (pos == 1) {
            drive.strafeByInch(24, .3); //left
            arm.deliver3(); //right
            sleep(750);
            drive.strafeByInch(5, .3); //left
            sleep(750);
            arm.release(); //release arm
            sleep(750);
            drive.strafeByInch(-29, .4); //right
        }
        else if (pos == -1) {
            drive.strafeByInch(16, .3); //left
            arm.deliver1(); //left
            sleep(750);
            drive.strafeByInch(3, .3); //left
            sleep(750);
            arm.release(); //release arm
            sleep(750);
            drive.strafeByInch(-20, .4); //right
        }
        else {
            drive.strafeByInch(16, .3); //left
            arm.deliver2(); //center
            sleep(750);
            drive.strafeByInch(5, .3); //left
            sleep(750);
            arm.release(); //release arm
            sleep(750);
            drive.strafeByInch(-23, .4); //right
        }
        sleep(750);

        //drive.rotate(-30, 0.3, 500); //align to the wall

        arm.mWristLift();
        sleep(250);
        arm.mWristHome();


        drive.forwardByInch(-39, .3); //backward
        sleep(500);

        drive.strafeByInch(7,.3);
        sleep(250);

        drive.rotate(90, 0.2, 2000);
        sleep(500);

        drive.strafeByInch(4,.3);
        sleep(250);

        drive.forwardByInch(-6,.2);

        carousel.red();
        sleep(2000);

        drive.forwardByInch(7,.3);
        sleep(200);

        drive.rotate2(40, 0.6, 1000);
        sleep(500);

        drive.forwardByInch(15,.3);
        sleep(20000);

//        //parking to warehouse if there is time
//        if (runtime.seconds() < 22 ) {
//            drive.rotate(45, 0.3, 1000);
//            //drive.forwardByInch(10, .5); //drive foward
//            //drive.strafeByInch(-4, .4); //right
//           //drive.forwardByInch(60, .5); //drive foward
//        }
//        else { //park to team station
//            drive.strafeByInch(12, .4); //left
//            drive.forwardByInch(-12, .5); //drive back
//        }

    }
}
