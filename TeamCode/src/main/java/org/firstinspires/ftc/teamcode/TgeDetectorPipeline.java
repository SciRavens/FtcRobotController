package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class TgeDetectorPipeline extends OpenCvPipeline {

    Rect z1Rect = new Rect(0, 300, 75, 150);
    Rect z2Rect = new Rect(220, 270, 120, 120);
    Rect z3Rect = new Rect(475  , 293, 150, 150);
    Scalar red = new Scalar(255, 0, 0);
    Scalar blue = new Scalar(0, 0, 255);
    Scalar grey = new Scalar(93, 93, 93);

    String tgeColor = "blue";
    int tgeZone = 1;

    public TgeDetectorPipeline() {
        super();
    }

    @Override
    public Mat processFrame(Mat frame) {
        double blueDist[] = new double[3];
        double redDist[] = new double[3];
        Scalar z1AvgColor, z2AvgColor, z3AvgColor;
        Mat z1, z2, z3;
        double colorDist;

        //Creating duplicate of original frame with no edits
       // original = frame.clone();

        z1 = frame.submat(z1Rect);
        z2 = frame.submat(z2Rect);
        z3 = frame.submat(z3Rect);

        z1AvgColor = Core.mean(z1);
        z2AvgColor = Core.mean(z2);
        z3AvgColor = Core.mean(z3);


        blueDist[0] = colorDist(z1AvgColor, blue);
        blueDist[1] = colorDist(z2AvgColor, blue);
        blueDist[2] = colorDist(z3AvgColor, blue);

        redDist[0] = colorDist(z1AvgColor, red);
        redDist[1] = colorDist(z2AvgColor, red);
        redDist[2] = colorDist(z3AvgColor, red);

        int bluemin = 0;
        int redmin = 0;
        for (int i = 1; i < 3; i++) {
            if (blueDist[i] < blueDist[bluemin]) {
                bluemin = i;
            }
            if (redDist[i] < redDist[redmin]) {
                redmin = i;
            }
        }

        tgeColor = "blue";
        tgeZone = 3;
        colorDist = blueDist[bluemin];
        if (blueDist[bluemin] < 190) {
            tgeZone = bluemin + 1;
        }
        if (redDist[redmin] < blueDist[bluemin]) {
            tgeZone = redmin + 1;
            tgeColor = "red";
        }
        if (redDist[redmin] < 190) {
            tgeZone = redmin + 1;
        }

        return frame;
    }

    public double colorDist(Scalar c1, Scalar c2){
        double rDiff = c1.val[0] - c2.val[0];
        double gDiff = c1.val[1] - c2.val[1];
        double bDiff = c1.val[2] - c2.val[2];
        return Math.sqrt(rDiff * rDiff + gDiff * gDiff + bDiff * bDiff);
    }

    public int getTgeZone() {
        return tgeZone;
    }
    public String getTgeColor() {
        return tgeColor;
    }
}
