package org.firstinspires.ftc.teamcode.opmodes.testing;
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
@Autonomous(name = "TurningTest", group = "Quackology")
//@Disabled

public class TurningTest extends LinearOpMode {
    private DriveTrainMecanumBasic drive;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        drive = new DriveTrainMecanumBasic(hardwareMap, telemetry);
        drive.resetAngle();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("angle ", "%f5.2", drive.getRawExternalHeading());
            telemetry.addData("global angle ", "%f5.2", drive.getAngle());
            telemetry.update();
        }

        /*while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("angle ", "%f5.2", drive.getRawExternalHeading());
            telemetry.addData("global angle ", "%f5.2", drive.getAngle());
            telemetry.update();
        }
        */

        drive.rotate(90, 0.3, 1500); //worked, over turned during to imu delay
        sleep(1000);
        drive.rotate(-90,0.3,3000);


    }
}