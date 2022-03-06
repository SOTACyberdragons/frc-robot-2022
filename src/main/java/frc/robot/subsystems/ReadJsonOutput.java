package frc.robot.subsystems;

import org.json.*;

import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ContainerFactory;
import org.json.simple.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReadJsonOutput extends SubsystemBase {

     private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
     private static NetworkTable outputTable = inst.getTable("ml");
     private static NetworkTableEntry detectionsEntry = outputTable.getEntry("detections");

    public static double xMin, yMin, xMax, yMax;

    public static List<Ball> balls;

    private static void readBallData()
    {
        balls.clear(); //refresh balls 

        String output = detectionsEntry.getString("");

        ObjectMapper mapper = new ObjectMapper();


        JsonFactory factory = new JsonFactory();

        try {
            JsonParser parser = factory.createParser(output);

            balls = mapper.readTree(parser);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static Ball getClosestBall(boolean red)
    {
        String color;

        if (red) {
            color = new String("red");
        } else {
            color = new String("blue");
        }

        readBallData(); //get the list of all balls in our field of view

        int closestBallIndex = 0;

        //delete balls from list that aren't of our color
        for (int i = 0; i < balls.size(); i++) {
            if (balls.get(i).color != color) {
                balls.remove(i);
                i--; //we have deleted an element, so our index must shrink by 1
            }
        }

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
