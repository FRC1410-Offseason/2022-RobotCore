package frc.robot.util;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class PolynomialRegression {
	private final Random random = new Random();
	private double[][] lastParameters = null;
	private double[] lastErrors = null;
	private int lastParameterIndex = 0;

	private final double[] parameters;
    private final int bufferSize;

	// TODO: docs
	public double f(double x) {
		double ret = 0;
		for (int i = 0; i < parameters.length; i++) {
			ret += parameters[i] * Math.pow(x, i+1);
		}
		return ret;
	}

	public PolynomialRegression(int degree, int bufferSize) {
		parameters = new double[degree];
		this.bufferSize = bufferSize;
	}

	public double error(List<Double> x, List<Double> y) {
		if (x.size() != y.size()) throw new IllegalArgumentException("Unequal length");

		double error = 0;
		double tError = 0;
		double yBar = 0;

		for (int i = 0; i < x.size(); i++) {
			yBar += y.get(i);
			error += Math.pow(f(x.get(i))- y.get(i), 2);
		}

		yBar /= y.size();

		for (int i = 0; i < x.size(); i++) tError += Math.pow(y.get(i) - yBar, 2);

		return error / tError;
	}

	public void reset() {
		Arrays.fill(parameters, 0);
		lastParameterIndex=0;
	}

	public void gradStep(double alpha, List<Double> x, List<Double> y, double stepSize, double noise) {
		if (lastParameters == null) lastParameters = new double[bufferSize][parameters.length];
		if (lastErrors == null) lastErrors = new double[bufferSize];

		double[] deltas = new double[parameters.length];
		double currError = error(x, y);

		System.arraycopy(parameters, 0, lastParameters[lastParameterIndex % bufferSize], 0, parameters.length);

		lastErrors[lastParameterIndex % bufferSize] = currError;
		lastParameterIndex++;

		for (int i = 0; i < parameters.length; i++) {
			parameters[i] += stepSize;
			double newError = error(x, y);
			deltas[i] = (newError - currError) / stepSize;
			parameters[i] -= stepSize;
		}

		for (int i = 0; i < parameters.length; i++) {
			double r = random.nextDouble();
			r *= noise * alpha * Math.signum(deltas[i]);
			parameters[i] -= (alpha * deltas[i] + r);
		}
	}
    
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

	public double[] getParameters() {
		return parameters;
	}
}
