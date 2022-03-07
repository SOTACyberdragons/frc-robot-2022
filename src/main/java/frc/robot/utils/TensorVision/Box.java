package frc.robot.utils.TensorVision;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Box {
    @JsonProperty("xmin")
    public int xMin;

    @JsonProperty("ymin")
    public int yMin;

    @JsonProperty("xmax")
    public int xMax;

    @JsonProperty("ymax")
    public int yMax;
}