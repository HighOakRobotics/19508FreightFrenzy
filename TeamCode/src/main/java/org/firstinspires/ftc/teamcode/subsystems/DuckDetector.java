package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.ftc11392.sequoia.util.Clock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class DuckDetector extends Subsystem {

    protected OpenCvCamera camera;
    protected Clock clock;

    private DuckPipeline duckPipeline;

    private boolean open = false;

    private int leftDuckX;
    private int centerDuckX;
    private int rightDuckX;

    public DuckDetector(int leftDuckX, int centerDuckX, int rightDuckX) {
        this.leftDuckX = leftDuckX;
        this.centerDuckX = centerDuckX;
        this.rightDuckX = rightDuckX;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        clock = new Clock();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        duckPipeline = new DuckPipeline(leftDuckX, centerDuckX, rightDuckX);
        camera.setPipeline(duckPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.log().add("DuckDetector camera acquisition error: " + errorCode);
            }
        });
        open = true;
    }

    @Override
    public void initPeriodic() {
        telemetry.addData("avg", "%d %d %d", duckPipeline.avg1, duckPipeline.avg2, duckPipeline.avg3);
        telemetry.addData("maxVal", Math.max(Math.max(duckPipeline.avg1, duckPipeline.avg2), duckPipeline.avg3));
    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {

    }

    @Override
    public void stop() {
        if (open) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
        open = false;
    }

    public DuckPipeline.DuckPosition getAnalysis(){
        return duckPipeline.getAnalysis();
    }

    public static class DuckPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum DuckPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar RED = new Scalar(255,0,0);
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        Point REGION1_TOPLEFT_ANCHOR_POINT;
        Point REGION2_TOPLEFT_ANCHOR_POINT;
        Point REGION3_TOPLEFT_ANCHOR_POINT;
        int REGION_WIDTH;
        int REGION_HEIGHT;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA;
        Point region1_pointB;
        Point region2_pointA;
        Point region2_pointB;
        Point region3_pointA;
        Point region3_pointB;
        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile DuckPosition position = DuckPosition.LEFT;

        public DuckPipeline(int leftDuckX, int centerDuckX, int rightDuckX) {
            /*
             * The core values which define the location and size of the sample regions
             */
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(leftDuckX,100);
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(centerDuckX,100);
            REGION3_TOPLEFT_ANCHOR_POINT = new Point(rightDuckX,100);
            REGION_WIDTH = 40;
            REGION_HEIGHT = 40;

            /*
             * Points which actually define the sample region rectangles, derived from above values
             *
             * Example of how points A and B work to define a rectangle
             *
             *   ------------------------------------
             *   | (0,0) Point A                    |
             *   |                                  |
             *   |                                  |
             *   |                                  |
             *   |                                  |
             *   |                                  |
             *   |                                  |
             *   |                  Point B (70,50) |
             *   ------------------------------------
             *
             */
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int min = Math.min(Math.min(avg1, avg2), avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(min == avg1) // Was it from region 1?
            {
                position = DuckPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        min < 100 ? GREEN : RED, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(min == avg2) // Was it from region 2?
            {
                position = DuckPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        min < 100 ? GREEN : RED, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(min == avg3) // Was it from region 3?
            {
                position = DuckPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        min < 100 ? GREEN : RED, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            Imgproc.putText(input, Integer.toString(avg1), region1_pointB, Imgproc.FONT_HERSHEY_DUPLEX, 0.75, avg1 < 100 ? GREEN : RED);
            Imgproc.putText(input, Integer.toString(avg2), region2_pointB, Imgproc.FONT_HERSHEY_DUPLEX, 0.75, avg2 < 100 ? GREEN : RED);
            Imgproc.putText(input, Integer.toString(avg3), region3_pointB, Imgproc.FONT_HERSHEY_DUPLEX, 0.75, avg3 < 100 ? GREEN : RED);
            Imgproc.putText(input, "Detection: "+position, new Point(0,25), Imgproc.FONT_HERSHEY_DUPLEX, 0.75, min < 100 ? GREEN : RED);
            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public DuckPosition getAnalysis()
        {
            return position;
        }
    }
}