package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TgeDetection {
    private OpenCvCamera camera;
    Scalar red = new Scalar(255, 0, 0);
    Scalar blue = new Scalar(0, 0, 255);
    Robot robot = null;

    private TgeDetectorPipeline tgeDetectorPipeline;

    public TgeDetection(Robot robot) {
        this.robot = robot;
        tgeDetectorPipeline = new TgeDetectorPipeline();
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.webcam);
        camera.setPipeline(tgeDetectorPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }

    public int getZone()
    {
        return tgeDetectorPipeline.getTgeZone();
    }
    public String getTgeColor()
    {
        return tgeDetectorPipeline.getTgeColor();
    }
}
