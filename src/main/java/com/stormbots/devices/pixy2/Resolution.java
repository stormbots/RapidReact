package com.stormbots.devices.pixy2;

public class Resolution{
    //Just in case, pick a sane default for Block and Line processing
    public int width = 79;
    public int height = 52;

    public Resolution(){}

    public String toString(){
        return "Resolution " + width +" x "+ height;
    }
}