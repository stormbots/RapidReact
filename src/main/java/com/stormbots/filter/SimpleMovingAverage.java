package com.stormbots.filter;

/**
 * Return the average of the last N samples measured.
 * <br/>
 * Does a good job of cleaning up data in general. Can significantly reduce noise, and will
 * nearly eliminate high-frequency oscillations with a frequency less than 1/2 the sample size.
 * <br/>
 * However, the output value is time-shifted, and larger sample sizes can apply
 * significant delay to your output.
 */
public class SimpleMovingAverage extends Filter{
  double[] samples;
  int index = 0;
  int size = 0;
  boolean full = false;
  double mean = 0;

  public SimpleMovingAverage(int samplesize){
    samples = new double[samplesize];
    size = samplesize;
  }

  /** Clears the tracked samples */
  public void clear(){
    index = 0; 
    full = false;
  }

  /** Add data to our samples, removing the oldest */
  public void put(double value){
    //Manage our buffer
    samples[index]=value;
    if (++index == size) index = 0;;
    if(index==0)full = true;

    //Do this on write to make reads faster
    mean = 0;
    if(full){
      //Count up the whole buffer
      for(int i=0; i<size;i++){mean += samples[i];}
      mean /= size;
    }
    else{
      //count up to what we've written
      for(int i=0; i<index;i++){mean += samples[i];}
      mean /= index;
    }
  }

  /** Get the current average */
  public double get(){
    return mean;
  }

}

