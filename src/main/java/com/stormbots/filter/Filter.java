package com.stormbots.filter;

/** Class used to help clean up and filter streaming data */
public abstract class Filter{
  public abstract void clear();
  public abstract void put(double value);
  public abstract double get();
}