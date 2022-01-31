package com.stormbots.devices.pixy2;

import static com.stormbots.Lerp.lerp;

/**
 * Represent a Line from the getMainFeautures() call When handling raw
 * resolutions, note the camera resolution is 79 x 52 in mainFeautures mode.
 */
public class Line{
    public double x0=0,y0=0,x1=0,y1=0;
    boolean normalized = false;
    //TODO : Possibly determine out a good way to verify the resolution without soaking time in critical loop
    //Possibly static class variables?

    public Line(){}
    public Line(byte x0,byte x1,byte y0,byte y1){
        this.x0 = x0;
        this.y0 = y0;
        this.x1 = x1;
        this.y1 = y1;
    }

    public String toString(){
        return String.format("Line (%.2f,%.2f)->(%.2f,%.2f)",x0,y0,x1,y1);
    }

    /** Convert coordinates to [-1..1], with the bottom center of the screen as (0,0) */
    public Line normalizeBottom(){
        if(normalized)return this;
        normalized = true; 
        x0 = lerp(x0, 0, 79, -1, 1);
        y0 = lerp(y0, 0, 52, 0, 1);
        x1 = lerp(x1, 0, 79, -1, 1);
        y1 = lerp(y1, 0, 52, 0, 1);
        return this;
    }
    
    /** Convert coordinates to [-1..1], with the center of the screen as (0,0)  */
    public Line normalizeCenter(){
        if(normalized)return this;
        normalized = true; 
        x0 = lerp(x0, 0, 79, -1, 1);
        y0 = lerp(y0, 0, 52, -1, 1);
        x1 = lerp(x1, 0, 79, -1, 1);
        y1 = lerp(y1, 0, 52, -1, 1);
        // x0 = 2*(x0/79.0-0.5);
        // y0 = 2*(0.5-y0/52.0);
        // x1 = 2*(x1/79.0-0.5);
        // y1 = 2*(0.5-y1/52.0);
        return this;
    }

}    