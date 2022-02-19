package frc.robot.grip;

import edu.wpi.first.vision.VisionRunner.Listener;

abstract class CustomListener implements Listener<GripPipeline> {
    
    public abstract void copyPipelineOutputs(GripPipeline pipeline);
}
