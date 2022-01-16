package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.triggers.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;

@TeleOp
@Disabled
public class DuckDetectorTestOpMode extends SequoiaOpMode {

    DuckDetector duckDetector = new DuckDetector(0, 105, 185);

    @Override
    public void initTriggers() {
        new Trigger(() -> true).whileOn(new InstantTask(() -> {telemetry.addData("Detection", duckDetector.getAnalysis());}));
    }

    @Override
    public void runTriggers() {
        duckDetector.stop();
    }
}
