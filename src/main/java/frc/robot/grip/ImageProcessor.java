package frc.robot.grip;

import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class ImageProcessor extends GripPipeline {
    
    private UsbCamera camera;

    private boolean running;

    public double angleFromRobot(Rect rect)
    {
        return 0;
    }

    public double distance(Rect rect)
    {
        return 0;
    }

    private Thread m_visionThread = new Thread(() ->
    {
        CvSink sink = CameraServer.getVideo(); //captures mats from camera

        CvSource outputStream = CameraServer.putVideo("Camera", RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT); 

        Mat mat = new Mat(); 

        while (this.running) {
            if (sink.grabFrame(mat) == 0) {
                outputStream.notifyError(sink.getError());
                
                continue;
            }

            MatOfKeyPoint blobOutput = this.findBlobsOutput();

            Rect rect = Imgproc.boundingRect(blobOutput);

            RobotContainer.ballDistance.setDouble(distance(rect));
            RobotContainer.ballAngle.setDouble(angleFromRobot(rect));

            outputStream.putFrame(mat);
        }
    });

    public void stopRunning()
    {
        this.running = false;
    }

    public ImageProcessor()
    {
        this.camera = CameraServer.startAutomaticCapture();
        if (!camera.isConnected()) {
            System.out.println("Error: Camera is not connected.\n");
        }
        camera.setResolution(RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);

        m_visionThread.setDaemon(true);
    }
    
    public void execute()
    {
        if (m_visionThread.isAlive()) {
            System.out.println("Error: vision thread is already running!\n");
    
            return;
        }

        

        this.running = true;

        m_visionThread.start();
    }
}
