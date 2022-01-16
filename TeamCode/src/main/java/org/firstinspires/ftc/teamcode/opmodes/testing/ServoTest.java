package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Concept")
@Disabled
public class ServoTest extends LinearOpMode {

    SlowServo servo;
    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = new SlowServo(hardwareMap);
        servo.init();

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        boolean rampup = true;
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
            if (rampup) {
                servo.slowSetPostion(0.8, 0.01, 50);
                if (!servo.busy) rampup = false;
            }

            else {
                servo.slowSetPostion(0.2, -0.01, 50);
                if (!servo.busy ) rampup = true;
            }
            /*
            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            */
            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            //servo.setPosition(position);
            //sleep(CYCLE_MS);
            //idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

