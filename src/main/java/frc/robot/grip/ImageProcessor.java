package frc.robot.grip;

import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.LineSegmentDetector;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class ImageProcessor extends RedBallPipeline {

    private UsbCamera camera;
    private boolean running;

    public double ballCenter(Rect rect) {
        System.out.println(rect.x);
        System.out.println(rect.y);
        return (rect.x + rect.x + rect.width) / 2;
    }

    private Thread m_visionThread = new Thread(() -> {
        CvSink sink = CameraServer.getVideo(); // captures mats from camera
        CvSource outputStream = CameraServer.putVideo("Camera", RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
            if (sink.grabFrame(mat) == 0) {
                outputStream.notifyError(sink.getError());
                continue;
            }
            process(mat);
            // MatOfKeyPoint blobOutput = this.findBlobsOutput();
            // Imgproc.rectangle(
            //        mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
            outputStream.putFrame(mat);
        }
    });

    public void stopRunning() {
        this.running = false;
    }

    public ImageProcessor() {
        this.camera = CameraServer.startAutomaticCapture();
        if (!camera.isConnected()) {
            System.out.println("Error: Camera is not connected.\n");
        }

        camera.setResolution(RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);
        m_visionThread.setDaemon(true);
    }

    public void execute() {
        if (m_visionThread.isAlive()) {
            System.out.println("Error: vision thread is already running!\n");
            return;
        }
        this.running = true;
        m_visionThread.start();
    }
}
