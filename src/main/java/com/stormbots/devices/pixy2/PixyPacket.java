package com.stormbots.devices.pixy2;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * PixyPacket contains the return information by a pixy in minimally 
 * processed form.
 * 
 */
class PixyPacket{
    public boolean valid = false;
    /** Length of the data byte buffer stream */
    public byte length=0;
    /** Pixy packet type */
    public byte type = 0; 
    /** Pixy data checksum for use with validation */
    public short checksum = 0; 

    /** Buffer containing packet data */
    ByteBuffer data = ByteBuffer.allocate(40).order(ByteOrder.LITTLE_ENDIAN);

    public PixyPacket(){
    }

    public boolean validateChecksum(){
        // TODO: Check the checksum
        return this.valid;
    }
    
    public String toString(){
        if(valid) return String.format("PixyPacket(%d)[%d]#%x", type,length,checksum);
        else      return String.format("PixyPacket(INVALID)", type,length,checksum);
    }

}

