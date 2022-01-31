
package com.stormbots;


public class PiecewiseLerp{

    public double[] inputs;
    public double[] outputs;

    public PiecewiseLerp(double[] inputs, double[] outputs){
        this.inputs = inputs;
        this.outputs = outputs;
    }

    public double getOutputAt(double input) {
        if(outputs.length != inputs.length){
    		System.err.println("Number of elements in the list does not match!!!");
    		return  inputs[0];
        }
        //Scan through our inputs to find which values to extrapolate with
        for (int i=0; i<outputs.length-1;i++){
            if(inputs[i+1]>input){
                return Lerp.lerp(input, inputs[i], inputs[i+1],outputs[i], outputs[i+1]);
            }
        }
        //If our input is larger than our input range,extrapolate using last known data
        return Lerp.lerp(input, 
            inputs[outputs.length-2], inputs[outputs.length-1],
            outputs[outputs.length-2], outputs[outputs.length-1]
        );
    }
    
}












