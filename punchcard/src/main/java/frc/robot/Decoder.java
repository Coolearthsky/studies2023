package frc.robot;

/**
 * Decode the sensor readings into a number.
 * 
 * The output is inversely proportional to measures incident light: 0v is a lot,
 * 5v is dark. so ambient light can drive it low and if the surface is too
 * close, then the illuminator is shaded from the sensor, so a "light" surface
 * can seem "dark" so you need the surface to be a few mm away.
 */
public class Decoder {

}
