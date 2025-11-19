package com.team581.autos;

public interface BaseAuto {
  Point getStartingPoint();

  /** Returns the name of this auto. */
  default String name() {
    var className = this.getClass().getSimpleName();
    return className.substring(className.lastIndexOf('.') + 1);
  }
}
