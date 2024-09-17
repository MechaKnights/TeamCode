package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalPipelineTests extends OpenCvPipeline {
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
    public enum Colors {
        GREEN,
        BLACK,
        YELLOW,
        WHITE
    }

    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(240,125);
    private static Point SLEEVE_TOPLEFT2_ANCHOR_POINT = new Point(230,162);

    public static int REGION_WIDTH = 10;
    public static int REGION_HEIGHT = 10;

    //Some colors for the display
    private final Scalar GREEN = new Scalar(3, 137, 45);
    private final Scalar BLACK = new Scalar(0,0,0);
    private final Scalar YELLOW = new Scalar(255,255,50);
    private final Scalar WHITE = new Scalar(255,255,255);

    //20 x 20 pixel box. We'll need to adjust this
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point sleeve_point2A = new Point(
            SLEEVE_TOPLEFT2_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT2_ANCHOR_POINT.y);
    Point sleeve_point2B = new Point(
            SLEEVE_TOPLEFT2_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT2_ANCHOR_POINT.y + REGION_HEIGHT);
    //replace with space that is most useful
    private static volatile TeamShippingElementPosition position = TeamShippingElementPosition.LEFT;
    private static volatile Colors color = Colors.GREEN;

    @Override
    public Mat processFrame(Mat input){

        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Mat areaMat2 = input.submat(new Rect(sleeve_point2A, sleeve_point2B));
        Scalar sumColors = Core.sumElems(areaMat);
        Scalar sumColors2 = Core.sumElems(areaMat2);

        double minColor = Math.min(sumColors.val[0], sumColors.val[1]);
        double minColor2 = Math.min(sumColors2.val[0], sumColors2.val[1]);

        Imgproc.rectangle(
                input,
                new Point(240,125),
                new Point(210,135),
                GREEN,
                -1);

        Imgproc.rectangle(
                input,
                new Point(230,162),
                new Point(200,172),
                BLACK,
                -1);

        //if statements to determine what color the sleeve is
        if(sumColors.val[1] == minColor) {
            position = TeamShippingElementPosition.LEFT;
            color = Colors.GREEN;

            Imgproc.rectangle(
                    input,
                    new Point(240,125),
                    new Point(210,135),
                    GREEN,
                    -1);

            Imgproc.rectangle(
                    input,
                    new Point(230,162),
                    new Point(200,172),
                    BLACK,
                    -1);
        }

        if(sumColors2.val[0] == minColor) {
            position = TeamShippingElementPosition.CENTER;
            color = Colors.BLACK;

            Imgproc.rectangle(
                    input,
                    new Point(240,125),
                    new Point(210,135),
                    GREEN,
                    -1);

            Imgproc.rectangle(
                    input,
                    new Point(230,162),
                    new Point(200,172),
                    BLACK,
                    -1);

        }

        if(sumColors.val[0] != minColor & sumColors2.val[0] != minColor2) {
            position = TeamShippingElementPosition.RIGHT;
            color = Colors.YELLOW;

            Imgproc.rectangle(
                    input,
                    new Point(240,125),
                    new Point(210,135),
                    GREEN,
                    -1);

            Imgproc.rectangle(
                    input,
                    new Point(230,162),
                    new Point(200,172),
                    BLACK,
                    -1);

        }
        areaMat.release();
        areaMat2.release();
        return input;
    }
    public static TeamShippingElementPosition getPosition() {
        return position;
    }
    public static Colors getColor() {
        return color;
    }
}