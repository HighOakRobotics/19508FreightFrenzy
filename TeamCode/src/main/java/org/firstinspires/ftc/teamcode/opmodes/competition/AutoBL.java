package org.firstinspires.ftc.teamcode.opmodes.competition;
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

import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanumBasic;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SwingArm;
import org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV;

@Autonomous(name = "BlueLeft", group = "Quackology")
//@Disabled

public class AutoBL extends LinearOpMode {
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
        drive.forwardByInch(24, .5); //drive forward

        arm.mWristLift(); //lift in the middle
        sleep(500);
        arm.left();
        sleep(200);

        if (pos == 1) {
            drive.strafeByInch(24, .3); //left
            arm.deliver3(); //right
            sleep(800);
            drive.strafeByInch(5, .3); //left
            sleep(800);
            arm.release(); //release arm
            sleep(800);
            drive.strafeByInch(-29, .4); //right
        }
        else if (pos == -1) {
            drive.strafeByInch(16, .3); //left
            arm.deliver1(); //left
            sleep(800);
            drive.strafeByInch(3, .3); //left
            sleep(800);
            arm.release(); //release arm
            sleep(800);
            drive.strafeByInch(-20, .4); //right
        }
        else {
            drive.strafeByInch(16, .3); //left
            arm.deliver2(); //center
            sleep(800);
            drive.strafeByInch(5, .3); //left
            sleep(800);
            arm.release(); //release arm
            sleep(800);
            drive.strafeByInch(-23, .4); //right
        }
        sleep(800);

//        drive.rotate(-30, 0.3, 500); //tune right a little to align to the wall
        arm.mWristLift();
        sleep(800);
        arm.mWristHome();
        sleep(800);

        drive.forwardByInch(-60, .6); //back to park
    }
}