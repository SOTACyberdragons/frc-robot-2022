package frc.robot.subsystems;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReadJsonOutput extends SubsystemBase {

     private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
     private static NetworkTable outputTable = inst.getTable("ml");
     private static NetworkTableEntry detectionsEntry = outputTable.getEntry("detections");

    public static double xMin, yMin, xMax, yMax;

    public static ArrayList<Ball> balls = new ArrayList<>();

    public static boolean noTargets = false;

    public static void printBallData(Ball ball)
    {
        System.out.println(ball.color);
        System.out.println(ball.confidence);
    }

    public static double getBallYaw(Ball color)
    {
        
        return 0;
    }

    public static void readBallData() throws Exception
    {
        balls.clear(); //refresh balls 

        //Path testFilePath = Paths.get("C:/Users/SOTAC/robotics_projects/frc-robot-2022/src/main/example1.Json");
        String output = "[\n    {\n        \"label\": \"red\",\n        \"box\": {\n            \"ymin\": 216,\n            \"xmin\": 369,\n            \"ymax\": 359,\n            \"xmax\": 513\n        },\n        \"confidence\": 0.96875\n    },\n    {\n        \"label\": \"blue\",\n        \"box\": {\n            \"ymin\": 208,\n            \"xmin\": 90,\n            \"ymax\": 330,\n            \"xmax\": 225\n        },\n        \"confidence\": 0.88671875\n    }\n]";

        System.out.println(output);

        ObjectMapper mapper = new ObjectMapper();
        
        Ball[] ballArr = mapper.readValue(output, Ball[].class);
        
        for (Ball b : ballArr) {
            balls.add(b);
        }
    }

    public static Ball getClosestBall(boolean red)
    {
        Ball defaultBall = new Ball(); //return this if no balls are in front of robot
        String color;

        if (red) {
            color = new String("red");
        } else {
            color = new String("blue");
        }

        //get the list of all balls in our field of view
        try {
            readBallData();
        } catch (Exception e) {
            e.printStackTrace();

            return defaultBall;
        }

        for (Ball b : balls) {
            printBallData(b);
        }

        if (balls.size() == 0) { //if no balls are in front of robot
            return defaultBall;
        }

        //delete balls from list that aren't of our color
        for (int i = 0; i < balls.size(); i++) {
            if (!(balls.get(i).color.equals(color))) {
                System.out.println(balls.get(i).color + " is not equal to " + color);
                balls.remove(i);
                i--; //we have deleted an element, so our index must shrink by 1
            }
        }

        for (int i = 0; i < 25; i++) {
            System.out.println(balls.size());
        }

        int closestBallIndex = 0;

        //get index of ball with largest box
        for (int i = 0; i < balls.size(); i++) { 
            Box currentBox = balls.get(i).box;

            Box largestBox = balls.get(closestBallIndex).box;

            if (currentBox.xMax - currentBox.xMin > largestBox.xMax - largestBox.xMin) {
                closestBallIndex = i;
            }
        }

        return balls.get(closestBallIndex);
    }
}