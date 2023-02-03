package org.firstinspires.ftc.teamcode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalPipeline extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    //An enum to define the sleeve colors
    public enum TeamShippingElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    //Some colors for the display
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    //20 x 20 pixel box. We'll need to adjust this
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    //replace with space that is most useful
    private static volatile TeamShippingElementPosition position = TeamShippingElementPosition.LEFT;

    @Override
    public Mat processFrame(Mat input){

        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(areaMat);

        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        Imgproc.rectangle(
                input,
                new Point(155,85),
                new Point(165,95),
                YELLOW,
                -1);

        //if statements to determine what color the sleeve is
        if(sumColors.val[0] == minColor) {
            position = TeamShippingElementPosition.LEFT;

            Imgproc.rectangle(
                    input,
                    new Point(155,85),
                    new Point(165,95),
                    YELLOW,
                    -1);

        }

        if(sumColors.val[1] == minColor) {
            position = TeamShippingElementPosition.CENTER;

            Imgproc.rectangle(
                    input,
                    new Point(155,85),
                    new Point(165,95),
                    CYAN,
                    -1);

        }

        if(sumColors.val[2] == minColor) {
            position = TeamShippingElementPosition.RIGHT;

            Imgproc.rectangle(
                    input,
                    new Point(155,85),
                    new Point(165,95),
                    MAGENTA,
                    -1);

        }
        double uno = sumColors.val[0];
        double dos = sumColors.val[1];
        double tres = sumColors.val[2];
        areaMat.release();
        return input;
    }
    public static TeamShippingElementPosition getAnalysis() {return position;}

}
