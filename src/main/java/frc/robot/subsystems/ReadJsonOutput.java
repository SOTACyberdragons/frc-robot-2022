package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;

import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.Vector;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReadJsonOutput extends SubsystemBase {

    private static Map<String, Integer> map;

    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable outputTable = inst.getTable("ml");
    private static NetworkTableEntry detectionsEntry = outputTable.getEntry("detections");

    public static double xMin, yMin, xMax, yMax;

    public static List<Ball> balls;

    public static void readBalls()
    {
        
    }

    public static void readRedBallData()
    {
        String output = detectionsEntry.getString("");

        ObjectMapper mapper = new ObjectMapper();

        Ball ball = new Ball();

        List<Ball> balls;

        try {
            
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
