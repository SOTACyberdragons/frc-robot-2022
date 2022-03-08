// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TensorVision.Box;
import frc.robot.utils.TensorVision.Target;

public class TensorVision extends SubsystemBase {
    /** Creates a new TensorVision. */
    private static final double kImageHeight = 480;
    private static final double kImageWidth = 640;
    private static final double kFOVWidth = 60;
    private static final double kPixelsPerDegreeWidth = kImageWidth / kFOVWidth;

    public static Target[] m_targets;
    public static String teamColor = "";

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable outputTable = inst.getTable("ML");
        NetworkTableEntry detectionsEntry = outputTable.getEntry("detections");
        String m_detectionsEntry = detectionsEntry.getString("[]");

        try {
            m_targets = parseTargets(m_detectionsEntry);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static Target[] parseTargets(String payload) throws JsonParseException, JsonMappingException, IOException {
        // targets.clear(); // refresh targets
        String detections = payload;
        final ObjectMapper objectMapper = new ObjectMapper();
        return objectMapper.readValue(detections, Target[].class);
    }

    // Do we have a target? If not, move on ...
    public static boolean hasTargets(Target[] targetList, String teamColor) {
        boolean foundTarget = false;
        for (int i = 0; i < targetList.length; i++) {
            if (targetList[i].color.equals(teamColor)) {
                foundTarget = true;
            }
        }
        return foundTarget;
    }

    // Get the array index of the best target, the target with the close y value
    public static int getBestTarget(Target[] targetList, String teamColor) {
        int bestTarget = 0;
        Box largestBox = new Box();

        for (int i = 0; i < targetList.length; i++) {
            Box currentBox = targetList[i].box;

            // System.out.println("Checking ball ..." + targetList[i].color + " " +
            // currentBox.yMax + " " + largestBox.yMax);

            if (targetList[i].color.equals(teamColor)) {
                if (currentBox.yMax > largestBox.yMax) {
                    largestBox = targetList[i].box;
                    bestTarget = i;
                }
            }
        }
        return bestTarget;
    }

    // Used for finding the depth of the closet target
    public static double getTargetDistance(Target[] targetList, String teamColor) {
        double targetDistance = 0;
        targetDistance = targetList[getBestTarget(targetList, teamColor)].box.yMax;
        return targetDistance;
    }

    // Return rumbles strength for the controller, the closer the stronger the
    // effect
    public static double getRumbleStrength(Target[] targetList, String teamColor) {
        double rumbleStrength = 0;
        rumbleStrength = getTargetDistance(targetList, teamColor);
        rumbleStrength = rumbleStrength / kImageHeight;
        return rumbleStrength;
    }

    // Get the Yaw of the best target in order to turn the robot
    public static double getTargetYaw(Target[] targetList, String teamColor) {
        int myTarget = 0;
        double myTargetCentroid = 0;
        double myTargetPosition = 0;
        double myTargetYaw = 0;

        myTarget = getBestTarget(targetList, teamColor);
        myTargetCentroid = ((targetList[myTarget].box.xMax - targetList[myTarget].box.xMin) / 2)
                + targetList[myTarget].box.xMin;

        // Ensure that yaw is negative if the target is left of the center
        myTargetPosition = ((kImageWidth / 2) - myTargetCentroid) * -1;
        myTargetYaw = myTargetPosition / kPixelsPerDegreeWidth;

        return myTargetYaw;
    }

}
