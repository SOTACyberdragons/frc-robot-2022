package frc.robot.grip;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.vision.VisionRunner.Listener;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionRunner;

import frc.robot.RobotMap;

public class CustomPipeline extends GripPipeline {

    private final UsbCamera camera = CameraServer.startAutomaticCapture();

    private VisionThread visionThread;

    private final Object imgLock = new Object();

    private double centerX = 0.0;

    public CustomPipeline()
    {
        camera.setResolution(RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);

        VisionRunner<GripPipeline> runner = new VisionRunner<GripPipeline>(camera, new GripPipeline(), new CustomListenerHeir());

        visionThread = new VisionThread(runner);

        visionThread.start();
    }
    
}
