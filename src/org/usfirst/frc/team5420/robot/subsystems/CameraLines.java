package org.usfirst.frc.team5420.robot.subsystems;


import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team5420.robot.Robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @link https://github.com/FRCTeam3255/BenchBot2017/blob/master/src/org/usfirst/frc/team3255/testbot2017/subsystems/Vision.java
 */
public class CameraLines extends Subsystem implements Runnable {

	private UsbCamera camera = null;
	
	private CameraServer cameraServer = null;
	
	private CvSink cameraSink = null;
	private CvSource outputStream = null;
	private Thread visionThread = null;
	
	public static final int CAMERA_WIDTH = Robot.width;
	public static final int CAMERA_HEIGHT = Robot.height;
	public static final int CAMERA_FPS = 30;
	
	public CameraLines() {
		visionThread = new Thread(this);
		visionThread.setDaemon(true);
		visionThread.start();
	}
	
	@Override
	public void run() {
		
		// get an instance of the CameraServer class
		cameraServer = CameraServer.getInstance();

		// create the front camera
		camera = new UsbCamera("Camera (DNU)", 0);

        // set the resolution of the rear camera
		camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);

        // set front camera by frames per second (FPS), and set the rear camera FPS to 0
		camera.setFPS(CAMERA_FPS);
		
        // create a CvSink for sourcing a camera
        cameraSink = new CvSink("CameraCvSink");

        // set the source for the CvSink to the selected camera
        cameraSink.setSource(camera);
        
        // create an outPutStream to write video the dashboard
        // This stream can be viewed on SmartDashboard by adding a "CameraServer Stream Viewer" widget
        // and setting its "Camera Choice" property to "Selected Camera"
        outputStream = cameraServer.putVideo("Selected Camera", CAMERA_WIDTH, CAMERA_HEIGHT);
        
        // Creates a mat to hold a video image frame
        Mat image = new Mat();
        
        ////======= Thread Loop ======////
		// This cannot be 'true'. The program will never exit if it is. This
		// lets the robot stop this thread when restarting robot code or
		// deploying.
		while (!Thread.interrupted()) {
			// grab a frame from the CvSink
			if(cameraSink.grabFrame(image) == 0) {
				// Send the output the error.
				outputStream.notifyError("grabFrame failed: " + cameraSink.getError());
				// skip the rest of the current iteration
				continue;
			}

        	// Draw Left Line
            Imgproc.line(image, new Point(50, CAMERA_HEIGHT- 10 ), new Point(120, 10),
				new Scalar(0, 0, 255), 2);
            
            // Draw Right Line
            Imgproc.line(image, new Point(CAMERA_WIDTH - 80, CAMERA_HEIGHT- 10 ), new Point(CAMERA_WIDTH - 120, 10),
    				new Scalar(0, 0, 255), 2);
        	
            //Imgproc.putText(image, "Main Camera", new Point(10, CAMERA_HEIGHT / 1.2), 0, 1.0, new Scalar(255, 255, 255));
            
			outputStream.putFrame(image);
		}
	}
	
	@Override
	protected void initDefaultCommand() {
	}
}