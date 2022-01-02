package org.firstinspires.ftc.teamcode.opmodes.testing;

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
    boolean home;
    //    private final Arm arm = new Arm();
    //private final SwingArm arm = new SwingArm();
    private Carousel carousel;
//    private final Gripper gripper = new Gripper();

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
        arm.slowHome();
        telemetry.addData("hand ", "%f2.1", arm.getHandPos());
        telemetry.addData("write ", "%f2.1", arm.getWristPos());
        telemetry.addData("shoulder ", "%f2.1", arm.getShoulderPos());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        arm.home();
        home = true;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive.humanControl(gamepad1);
        if (gamepad1.dpad_right) intake.inSlow(); //intake.in();
        else if (gamepad1.dpad_down) intake.stop();
        else if (gamepad1.dpad_left) intake.out();
        else if (gamepad1.dpad_up) arm.start();

        if(gamepad1.b) carousel.red();
        else if(gamepad1.a) carousel.stop();
        else if(gamepad1.x) carousel.blue();
        else if (gamepad1.y) arm.stop();

        if (gamepad2.dpad_right) arm.right();
        else if (gamepad2.dpad_left) arm.left();
        else if (gamepad2.dpad_up) arm.center();//arm.save(); //arm.slowSetPostion(0.6, -0.01, 50); // arm.lift(-1);
        else if (gamepad2.dpad_down )arm.slowHome(); //arm.home();
        else if(gamepad2.a) arm.slowHand(-0.005); //arm.intake();
        else if(gamepad2.x ) arm.deliver1();
        else if (gamepad2.b) arm.deliver2();
        else if (gamepad2.y) arm.slowHand(0.005); //.deliver3();
        if(gamepad2.right_bumper) arm.release();
        if(gamepad2.left_bumper) arm.retrieve();

        if (Math.abs(gamepad2.left_stick_y) > 0.02) {
            arm.adjust(gamepad2.left_stick_y);
            //arm.setCwrist(gamepad2.left_stick_y );
        }
        if (gamepad2.right_stick_y > 0.1) arm.slowLift(0.005);
        else if (gamepad2.right_stick_y < -0.1) arm.slowLift(-0.005);

        telemetry.addData("hand ", "%f2.1", arm.getHandPos());
        telemetry.addData("write ", "%f2.1", arm.getWristPos());
        telemetry.addData("shoulder ", "%f2.1", arm.getShoulderPos());
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
