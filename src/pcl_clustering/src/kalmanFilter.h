//
// Created by areza on 11/27/23.
//

#ifndef SRC_KALMANFILTER_H
#define SRC_KALMANFILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::VectorXf;

class KalmanFilter {
public:
    KalmanFilter(int stateDim, int measurementDim);

    KalmanFilter(const KalmanFilter &filter);

    void prediction();

    void update();

    void setProcessModel(const MatrixXf &processModel);

    void setMeasurementModel(const MatrixXf &measurementModel);

    void setCovariance(const MatrixXf &covariance);

    void setState(const VectorXf &state);

    void setProcessNoise(const MatrixXf &processNoise);

    void setMeasurementNoise(const MatrixXf &measurementNoise);

    void setMeasurement(const VectorXf &measurement);

    const MatrixXf &getCovariance() const;

    const MatrixXf &getProcessModel() const;

    const MatrixXf &getMeasurementModel() const;

    const MatrixXf &getProcessNoise() const;

    const MatrixXf &getMeasurementNoise() const;

    const VectorXf &getMeasurement() const;

    const VectorXf &getState() const;

private:
    MatrixXf _processModel;
    MatrixXf _measurementModel;
    MatrixXf _processNoise;
    MatrixXf _measurementNoise;
    MatrixXf _covariance;
    VectorXf _state;
    VectorXf _measurement;
};


#endif //SRC_KALMANFILTER_H
