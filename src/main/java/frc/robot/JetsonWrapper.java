package frc.robot;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import lombok.Getter;
import lombok.ToString;

@JsonIgnoreProperties(ignoreUnknown = true)
@Getter
@ToString
public class JetsonWrapper implements Comparable<JetsonWrapper> {

    private String color;
    private double x;
    private double y;
    private double height;
    private double width;
    private double area;
    private double distance;

    @Override
    public int compareTo(JetsonWrapper o) {
        return Double.compare(y, o.y);
    }
    
}
