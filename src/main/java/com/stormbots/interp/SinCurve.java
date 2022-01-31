/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.stormbots.interp;

/**
 * Useful sin approximations that allow quick non-linear value mapping along 
 * specific sections of a sin curve. 
 * <br>
 * These curves can be useful as basic motion planning for position and velocity
 * without incurring a large calculation cost, or for any other calculation where
 * a smooth adjustment is needed between distinct points.
 */
public class SinCurve {
  /* IMPLEMENTATION NOTES: 
  * These operations are done using a polynomial approximation of a sin curve. 
  * It's generated using carefully picked (but more or less arbitrary) constants
  * which form the basic operations for the two curves.
  * Because these polynomials are chosen for correctness only within the range of 
  * [0..1], the values are clamped to that range to ensure correct operation.
  */


  /** Generates a sin curve approximation of sin(-pi/2..pi/2), but scaled
   *  and translated to align to provided coordinates. An approximate graph
   *  the output is as follows
   * <pre>
   *   |        (x2,y2)
   *   |      .--------
   *   |     /   
   *   |    /      
   *   |---' 
   *       (x1,y1)
   * </pre>
   * Values in the range of (x1..x2) will be scaled by the sin function. <br/>
   * Values below x1 return y1. Values above x2 return y2 <br/>
   * 
   * @param x input value
   * @param x1
   * @param x2
   * @param y1
   * @param y2
   * @return output between [y1..y2] for any x
   */
  public static double scurve(double x,double x1,double x2,double y1,double y2){
    x = (x-x1)/(x2-x1);
    if(x<0)x=0;
    if(x>1)x=1;
    double xx=x*x;
    double xxx=xx*x;
    return ((10*xxx)-(15*xx*xx)+(6*xx*xxx))*(y2-y1)+y1;
  }
  

  /** Generates a sin curve approximation of cos(-pi/2..pi/2), but scaled
   *  and translated to align to provided coordinates. An approximate graph
   *  the output is as follows
   * <pre>
   *   |       ((x1+x2)/2, yPeak)
   *   |      .-.
   *   |     /   \
   *   |    /     \ 
   *   |---'       '---
   *     (x1,y1)  (x2,y1)
   * </pre>
   * I
   * Values in the range of (x1..x2) will be scaled by the sin function.<br/>
   * Values outside that range will return y1.<br/>
   * 
   * @param x input value
   * @param x1
   * @param x2
   * @param y1
   * @param yPeak
   * @return output between [y1..yPeak] for any x
   */  
  public static double ncurve(double x,double x1,double x2,double y1,double yPeak){
    x = (x-x1)/(x2-x1);
    if(x<0)x=0;
    if(x>1)x=1;
    double xx=x*x;
    double xxx=xx*x;
    return ((16*xx)-(32*xxx)+(16*xx*xx))*(yPeak-y1)+y1;
  }
}
