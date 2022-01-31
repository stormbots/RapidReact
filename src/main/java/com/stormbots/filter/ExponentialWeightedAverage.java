package com.stormbots.filter;

/**
 * Return an exponentially weighted average.
 * <br/>
 * This decreases the importance of each data point by how long ago it was added,
 * although every data point always plays a small part in the entire sum.
 * <br/>
 * Effective for reducing the effect of small input noise without inducing significant
 * lag to your dataset. However, as a result, it does not effectively block oscillations.
 */
public class ExponentialWeightedAverage extends Filter{
  double alpha;
  double mean = 0;
  boolean first = true;

  /**
   * @param alpha between (0..1], with higher values showing preference for the newest values
   */
  public ExponentialWeightedAverage(double alpha){
    if(alpha<=0 || alpha > 1) throw new IllegalArgumentException("Filter alpha must be between 0 and 1");
    this.alpha = alpha;
  }

  /** Clears the tracked samples */
  public void clear(){
    mean = 0;
    first = true;
  }

  /** Add a value to our weighted sum  */
  public void put(double value){
    if(first){
      mean = value;
      first = false;
    }else{
      mean = value*alpha + mean*(1-alpha);
    }
  }

  /** Get the current average */
  public double get(){
    return mean;
  }

}

