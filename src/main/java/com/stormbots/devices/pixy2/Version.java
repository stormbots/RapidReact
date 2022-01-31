package com.stormbots.devices.pixy2;

public class Version{
    public short hardwareVersion = 0;
    public byte major = 0;
    public byte minor = 0;
    public short firmware = 0;
    public char firmwareType = ' ';

    public Version(){}

    public String toString(){
        return "Version " + major +"."+ minor +" "+ hardwareVersion +" "+ firmware +""+ firmwareType;
    }
}