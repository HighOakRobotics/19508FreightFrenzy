package org.firstinspires.ftc.teamcode.opmodes.competition;

//@Disabled

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanumBasic;
import org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV;
import org.firstinspires.ftc.teamcode.subsystems.SwingArm;

@Autonomous(name = "Red Right", group = "Quackology")
//@Disabled
public class AutoRR extends LinearOpMode {
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
        }

        runtime.reset();
        drive.forwardByInch(-24, .5); //drive back

        arm.lift(-1); //lift in the middle
        sleep(200);
        arm.left();
        sleep(500);

        if (pos == 1) {
            drive.strafeByInch(24, .4); //left
            arm.deliver3(); //right
            sleep(1000);
            drive.strafeByInch(4, .3); //left
        }
        else if (pos == -1) {
            drive.strafeByInch(14, .4); //left
            arm.deliver1(); //left
            sleep(1000);
            drive.strafeByInch(3, .3); //left
        }
        else {
            drive.strafeByInch(15, .4); //left
            arm.deliver2(); //center
            sleep(1000);
            drive.strafeByInch(4, .3); //left
        }
        sleep(1000);
        arm.release(); //release arm

        sleep(1000);

        drive.strafeByInch(-28, .4); //right
        sleep(1000);
        arm.lift(-1);
        sleep(1000);
        arm.home();
        sleep(1000);

        drive.forwardByInch(50, .6);

    }
}
