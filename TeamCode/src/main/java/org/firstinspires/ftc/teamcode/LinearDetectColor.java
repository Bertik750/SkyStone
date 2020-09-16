package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;

import org.firstinspires.ftc.robotcontroller.internal.for_camera_opmodes.LinearOpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "LinearDetectColor", group = "ZZOpModeCameraPackage")
public class LinearDetectColor extends LinearOpModeCamera {

    int ds2 = 4;  // additional downsampling of the image
    String colorString = "NONE";

    @Override
    public void runOpMode() {

        if (isCameraAvailable()) {

            setCameraDownsampling(8);
            // parameter determines how downsampled you want your images
            // 8, 4, 2, or 1.
            // higher number is more downsampled, so less resolution but faster
            // 1 is original resolution, which is detailed but slow
            // must be called before super.init sets up the camera

            telemetry.addLine("Wait for camera to finish initializing!");
            telemetry.update();
            startCamera();  // can take a while.
            // best started before waitForStart
            telemetry.addLine("Camera ready!");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                analyze();
                sleep(500);

                sleep(10);
            }
            stopCamera();

        }
    }

    public void analyze() {
        int[] result = measure();

        int smallestPos = 0;
        int secondPos = 1;
        int x = 0;
        int y = 0;
        for (int i=0; i < result.length; i++) {
            int ri = result[i];
            if(i==0) {
                x = ri;
                y = result[i+1];
            } else {
                if(ri < x) {
                    y = x;
                    secondPos = smallestPos;
                    x = ri;
                    smallestPos = i;
                } else if(ri > x && ri < y) {
                    y = ri;
                    secondPos = i;
                }
            }
        }
        telemetry.addData("smallest:", smallestPos);
        telemetry.addData("Place:", Arrays.toString(result));
        telemetry.update();
    }

    public int[] measure() {
        int yellowLeft = 0;
        int yellowMiddle = 0;
        int yellowRight = 0;

        if (imageReady()) { // only do this if an image has been returned from the camera

            // get image, rotated so (0,0) is in the bottom left of the preview window
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

            int width = rgbImage.getWidth();
            int height = rgbImage.getHeight();

            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    int pixel = rgbImage.getPixel(x, y);
                            /*telemetry.addData("red:", red(pixel));
                            telemetry.addData("blue:", blue(pixel));
                            telemetry.addData("green:", green(pixel));
                            telemetry.update();
                            redValue += red(pixel);
                            blueValue += blue(pixel);
                            greenValue += green(pixel);*/
                    if(y < height-30 && y > 40) {
                        if (x <= width/3) {
                            yellowLeft += red(pixel) + green(pixel);
                        } else if(x > width/3 && x <= (width - width/3) ) {
                            yellowMiddle += red(pixel) + green(pixel);
                        } else {
                            yellowRight += red(pixel) + green(pixel);
                        }
                    }

                }
            }

            /*telemetry.addData("left", yellowLeft);
            telemetry.addData("middle", yellowMiddle);
            telemetry.addData("right", yellowRight);
            int color = highestColor(yellowLeft, yellowMiddle, yellowRight);

            switch (color) {
                case 0:
                    colorString = "LEFT";
                    break;
                case 1:
                    colorString = "MIDDLE";
                    break;
                case 2:
                    colorString = "RIGHT";
            }*/
        }
        int[] measurement = {yellowLeft, yellowMiddle, yellowRight};
        return measurement;

    }
}
