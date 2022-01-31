package com.stormbots.devices.pixy2;

public class Block{
    public int signature = 0;
    public int xCenter = 0;
    public int yCenter = 0;
    public int width = 0;
    public int height = 0;
    /** HSV Hue angle, which represents a specific color */
    public int colorAngle = 0;
    public int trackingIndex = 0;
    public int age = 0;

    public Block(){}

    public String toString(){
        return "Blocks " + signature +" "+ 
        xCenter +" "+ 
        yCenter +" "+ 
        width +" x "+ 
        height + " " + 
        colorAngle + " " + 
        trackingIndex + " " +
        age;
    }
}