package org.firstinspires.ftc.teamcode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalPipeline extends OpenCvPipeline {

    //An enum to define the sleeve colors
    public enum TeamShippingElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }
    //Declare variables for values
    private static double greenValue;
    private static double purpleValue;
    private static double redValue;
    //Some colors for the display
    static final Scalar BLUE = new Scalar(0,0,255);
    static final Scalar GREEN = new Scalar(0,255,0);
    static final Scalar Purple = new Scalar(255,0,255);
    static final Scalar ORANGE = new Scalar(255,130,0);

    //20 x 20 pixel box. We'll need to adjust this

    static final Rect CENTER_ROI = new Rect(
            new Point(170, 200),
            new Point(190, 220));

    Mat mat = new Mat();
    Mat greenmat = new Mat();
    Mat redmat = new Mat();
    Mat purplemat = new Mat();
    //replace with space that is most useful
    private volatile TeamShippingElementPosition position = TeamShippingElementPosition.LEFT;

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //This is what detects the different colors
        Scalar lowRed = new Scalar(0,50,70);
        Scalar highRed = new Scalar(35,255,255);
        Scalar lowGreen = new Scalar(36,50,70);
        Scalar highGreen = new Scalar(86,255,255);
        Scalar lowPurple = new Scalar(120,50,70);
        Scalar highPurple = new Scalar(300,255,255);

        Core.inRange(mat,lowGreen,highGreen,greenmat);
        Core.inRange(mat,lowPurple,highPurple,purplemat);
        Core.inRange(mat,lowRed,highRed,redmat);

        //Mat left = greenmat.submat(CENTER_ROI);
        //Mat center = Purplemat.submat(CENTER_ROI);
        //Mat right = orangemat.submat(CENTER_ROI);

        //finding values
        greenValue = Core.sumElems(greenmat).val[0] / CENTER_ROI.area() / 255;
        purpleValue = Core.sumElems(purplemat).val[0] / CENTER_ROI.area() / 255;
        redValue = Core.sumElems(redmat).val[0] / CENTER_ROI.area() / 255;

        greenmat.release();
        purplemat.release();
        redmat.release();

        double max = Math.max(redValue, Math.max(greenValue, purpleValue));

        Imgproc.rectangle(
                input,
                new Point(170,200),
                new Point(190,220),
                BLUE,
                -1);

        //if statements to determine what color the sleeve is
        if(max == greenValue) {
            position = TeamShippingElementPosition.LEFT;

            Imgproc.rectangle(
                    input,
                    new Point(170,200),
                    new Point(190,220),
                    GREEN,
                    -1);

        }

        if(max == purpleValue) {
            position = TeamShippingElementPosition.CENTER;

            Imgproc.rectangle(
                    input,
                    new Point(170,200),
                    new Point(190,220),
                    Purple,
                    -1);

        }

        if(max == redValue) {
            position = TeamShippingElementPosition.RIGHT;

            Imgproc.rectangle(
                    input,
                    new Point(170,200),
                    new Point(190,220),
                    ORANGE,
                    -1);

        }

        return input;
    }
    public TeamShippingElementPosition getAnalysis() {return position;}

    public static double getGreenValue() {return greenValue; }
    public static double getPurpleValue() {return purpleValue; }
    public static double getRedValue() {return redValue; }
}
