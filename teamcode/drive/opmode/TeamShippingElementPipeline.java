package org.firstinspires.ftc.teamcode.drive.opmode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamShippingElementPipeline extends OpenCvPipeline {
    //An enum to define the Team Shipping Element position
    public enum TeamShippingElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }
    //Some colors for the display
    static final Scalar BLUE = new Scalar(0,0,255);
    static final Scalar GREEN = new Scalar(0,255,0);

    //20 x 20 pixel boxes. We'll need to adjust these

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 200),
            new Point(20, 220));

    static final Rect CENTER_ROI = new Rect(
            new Point(170, 200),
            new Point(190, 220));

    static final Rect RIGHT_ROI = new Rect(
            new Point(300, 200),
            new Point(320, 220));

    Mat mat = new Mat();

    private volatile TeamShippingElementPosition position = TeamShippingElementPosition.RIGHT;

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //This is what detects the color green
        Scalar lowHSV = new Scalar(36,50,70);
        Scalar highHSV = new Scalar(86,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat center = mat.submat(CENTER_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        double maxOneTwo = Math.max(leftValue, centerValue);
        double max = Math.max(maxOneTwo, rightValue);

        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(
                input,
                new Point(0,200),
                new Point(20,220),
                BLUE,
                -1);

        Imgproc.rectangle(
                input,
                new Point(170,200),
                new Point(190,220),
                BLUE,
                -1);

        Imgproc.rectangle(
                input,
                new Point(300,200),
                new Point(320,220),
                BLUE,
                -1);

        //if statements to determine where the green object is
        if(max == leftValue) {
            position = TeamShippingElementPosition.LEFT;

            Imgproc.rectangle(
                    input,
                    new Point(0,200),
                    new Point(20,220),
                    GREEN,
                    -1);

        }

        if(max == centerValue) {
            position = TeamShippingElementPosition.CENTER;

            Imgproc.rectangle(
                    input,
                    new Point(170,200),
                    new Point(190,220),
                    GREEN,
                    -1);

        }

        if(max == rightValue) {
            position = TeamShippingElementPosition.RIGHT;

            Imgproc.rectangle(
                    input,
                    new Point(300,200),
                    new Point(320,220),
                    GREEN,
                    -1);

        }

        return input;
    }
    public TeamShippingElementPosition getAnalysis() {return position;}
}

