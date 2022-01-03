package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanumBasic;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SwingArm;
import org.firstinspires.ftc.teamcode.subsystems.TeamShipping;

@TeleOp(name = "swing arm", group = "Quackology")
//@Disabled
public class SwingArmOpMode extends OpMode {
    //private final Mecanum drivetrain = new Mecanum();
    private DriveTrainMecanumBasic drive;
    private Intake intake;
    private SwingArm arm;
    private TeamShipping teamShipping;
    private Carousel carousel;

    @Override
    public void init() {
        drive = new DriveTrainMecanumBasic(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        intake.init();
        carousel = new Carousel(hardwareMap);
        carousel.init();
        arm = new SwingArm(hardwareMap);
        arm.init();
        teamShipping = new TeamShipping(hardwareMap);
        teamShipping.init();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("hand ", "%f2.1", arm.getHandPos());
        telemetry.addData("shoulder ", "%f2.1", arm.getShoulderPos());
        telemetry.addData("m wrist ", "%f2.1", arm.getMWristPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive.humanControl(gamepad1);
        if (gamepad1.dpad_right) intake.in(); //intake.in();
        else if (gamepad1.dpad_down) intake.stop();
        else if (gamepad1.dpad_left) intake.out();

        if(gamepad1.b) carousel.red();
        else if(gamepad1.a) carousel.stop();
        else if(gamepad1.x) carousel.blue();

        if (gamepad2.dpad_right) arm.right();
        else if (gamepad2.dpad_left) arm.left();
        else if (gamepad2.dpad_up) arm.mWristLift();
        else if (gamepad2.dpad_down) arm.mWristHome();
        else if (gamepad2.a) arm.intake();
        else if (gamepad2.x ) arm.deliver1();
        else if (gamepad2.b) arm.deliver2();
        else if (gamepad2.y) arm.deliver3();
        else if (gamepad2.right_bumper) arm.release();
        else if (gamepad2.left_bumper) arm.retrieve();

        telemetry.addData("hand ", "%f2.1", arm.getHandPos());
        telemetry.addData("shoulder ", "%f2.1", arm.getShoulderPos());
        telemetry.addData("m wrist ", "%f2.1", arm.getMWristPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        arm.stop();
    }
}
