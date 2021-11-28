package org.firstinspires.ftc.teamcode.subsystems.operation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanumTest;
import org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV;
import org.firstinspires.ftc.teamcode.subsystems.Waver;

@Autonomous(name = "Auto Drive To Duck Test", group = "Quackology")
//@Disabled
public class AutoDriveToDuck extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanumTest drive;
    private org.firstinspires.ftc.teamcode.subsystems.Waver waver;
    private org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV eye;
    private int pos;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        drive = new org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanumTest(hardwareMap, telemetry);
        drive.autoInit();

        waver = new org.firstinspires.ftc.teamcode.subsystems.Waver(hardwareMap);
        eye = new org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV(0, 140, 280, 320, 240);
        eye.init(hardwareMap, telemetry);
        pos = -9999;

        while (!isStarted()) {
            if (eye.getAnalysis() == org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV.DuckPosition.RIGHT) {
                waver.right();
                pos = 1;
                telemetry.addData("right ", "%d", pos);
            } else if (eye.getAnalysis() == org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV.DuckPosition.LEFT) {
                waver.left();
                pos = -1;
                telemetry.addData("left ", "%d", pos);
            } else if (eye.getAnalysis() == org.firstinspires.ftc.teamcode.subsystems.EyeOpenCV.DuckPosition.CENTER) {
                waver.center();
                pos = 0;
                telemetry.addData("center ", "%d", pos);
            } else {
                pos = -9999;
                telemetry.addData("none ", "%d", pos);
            }

            eye.init_loop();
            telemetry.update();
        }
        // waitForStart();
        if (isStopRequested()) return;

        drive.forwardByInch(12, .3);
        drive.sleep(3);
        if (pos == 1) {
            drive.strafeByInch(-10, .3); //right
        }
        else if (pos == -1) {
            drive.strafeByInch(10, .3); //left
        }
        else {

        }

        drive.stop();
        telemetry.addData("final pos ", "%d", pos);
        telemetry.update();
        sleep(5000);
        //stop();
    }

}