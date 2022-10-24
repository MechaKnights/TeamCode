package org.firstinspires.ftc.teamcode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalPipeline1 extends OpenCvPipeline {
    //An enum to define the sleeve colors
    public enum TeamShippingElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }
    //Some colors for the display
    static final Scalar BLUE = new Scalar(0,0,255);
    static final Scalar GREEN = new Scalar(0,255,0);
    static final Scalar PINK = new Scalar(255,0,255);
    static final Scalar ORANGE = new Scalar(255,130,0);

    //20 x 20 pixel box. We'll need to adjust this

    static final Rect CENTER_ROI = new Rect(
            new Point(170, 200),
            new Point(190, 220));

    Mat greenmat = new Mat();
    Mat pinkmat = new Mat();
    Mat orangemat = new Mat();
    //replace with space that is most useful
    private volatile TeamShippingElementPosition position = TeamShippingElementPosition.CENTER;

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, greenmat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, pinkmat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, orangemat, Imgproc.COLOR_RGB2HSV);
        //This is what detects the different colors
        Scalar lowGreen = new Scalar(120,45,70);
        Scalar highGreen = new Scalar(120,100,100);
        Scalar lowPink = new Scalar(290,35,75);
        Scalar highPink = new Scalar(290,100,100);
        Scalar lowOrange = new Scalar(30,55,80);
        Scalar highOrange = new Scalar(28,100,100);

        Core.inRange(greenmat,lowGreen,highGreen,greenmat);
        Core.inRange(pinkmat,lowPink,highPink,pinkmat);
        Core.inRange(orangemat,lowOrange,highOrange,orangemat);

        //Mat left = greenmat.submat(CENTER_ROI);
        //Mat center = pinkmat.submat(CENTER_ROI);
        //Mat right = orangemat.submat(CENTER_ROI);

        //finding values
        double greenValue = Core.sumElems(greenmat).val[0] / CENTER_ROI.area() / 255;
        double pinkValue = Core.sumElems(pinkmat).val[0] / CENTER_ROI.area() / 255;
        double orangeValue = Core.sumElems(orangemat).val[0] / CENTER_ROI.area() / 255;

        Imgproc.cvtColor(greenmat,greenmat,Imgproc.COLOR_GRAY2RGB);

        greenmat.release();
        pinkmat.release();
        orangemat.release();

        double maxOneTwo = Math.max(greenValue, pinkValue);
        double max = Math.max(maxOneTwo, orangeValue);


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

        if(max == pinkValue) {
            position = TeamShippingElementPosition.CENTER;

            Imgproc.rectangle(
                    input,
                    new Point(170,200),
                    new Point(190,220),
                    PINK,
                    -1);

        }

        if(max == orangeValue) {
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
}
