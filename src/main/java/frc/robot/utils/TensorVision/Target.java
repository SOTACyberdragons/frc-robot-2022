package frc.robot.utils.TensorVision;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Target {
    @JsonProperty("label")
    public String color;

    @JsonProperty("box")
    public Box box;

    @JsonProperty("confidence")
    public double confidence;
}
