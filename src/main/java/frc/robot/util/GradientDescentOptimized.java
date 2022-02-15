package frc.robot.util;

import java.util.Arrays;
import java.util.Random;

public abstract class GradientDescentOptimized {
    private final Random random = new Random();
	private double[][] lastParameters = null;
	private double[] lastErrors = null;
	private int lastParameterIndex = 0;

	private final double[] parameters;
    private final int bufferSize;
	/**
	 * Reset all parameters and buffers.
	 */
	public void reset() {
		Arrays.fill(parameters, 0);
		lastParameterIndex=0;
	}
    public abstract double error(Object... errorparams);
    /**
	 * degree - Largest power in the polynomial.
	 * bufferSize - Number of parameters in the path to store to combat randomness errors (20 is about good).
	 * @param degree
	 * @param bufferSize
	 */
	public GradientDescentOptimized(int numparams, int bufferSize) {
		parameters = new double[numparams];
		this.bufferSize = bufferSize;
	}
	/**
	 * Alpha - learning rate
	 * x - x values
	 * y - y values
	 * stepSize - make as low as possible
	 * noise - Amount of randomness/entropy to use for escaping local minima. Proportional to alpha. 200 is about good for a 0-1 normalized x-value.
	 * @param alpha
	 * @param x
	 * @param y
	 * @param stepSize
	 * @param noise
	 */
	public void gradStep(double alpha, double stepSize, double noise, Object... errorparams) {
		if (lastParameters == null) lastParameters = new double[bufferSize][parameters.length];
		if (lastErrors == null) lastErrors = new double[bufferSize];

		double[] deltas = new double[parameters.length];
		double currError = error(errorparams);

		System.arraycopy(parameters, 0, lastParameters[lastParameterIndex % bufferSize], 0, parameters.length);

		lastErrors[lastParameterIndex % bufferSize] = currError;
		lastParameterIndex++;

		for (int i = 0; i < parameters.length; i++) {
			parameters[i] += stepSize;
			double newError = error(errorparams);
			deltas[i] = (newError - currError) / stepSize;
			parameters[i] -= stepSize;
		}

		for (int i = 0; i < parameters.length; i++) {
			double r = random.nextDouble();
			r *= noise * alpha * Math.signum(deltas[i]);
			parameters[i] -= (alpha * deltas[i] + r);
		}
	}
    /**
	 * Set the current parameters to the lowest in the buffer of bufferSize previous parameters and their errors.
	 * This is useful to eliminate some of the error that noise adds, and get the actual lowest values.
	 * Noise is incredibly useful, but sometimes it will cause errors, and buffering helps getting rid of them.
	 */
	public void setLowestInBuffer() {
		int num = Math.min(lastParameterIndex, bufferSize);
		double lowestError = -1;

		for (int i = 0; i < num; i++) {
			double error = lastErrors[i];

			if (error < lowestError || lowestError < 0) {
				lowestError = error;
				System.arraycopy(lastParameters[i], 0, parameters, 0, parameters.length);
			}
		}
	}
	/**
	 * Get the parameters
	 * @return parameters
	 */
	public double[] getParameters() {
		return parameters;
	}
}
