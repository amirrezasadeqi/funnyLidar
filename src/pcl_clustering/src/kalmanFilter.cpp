//
// Created by areza on 11/27/23.
//

#include "kalmanFilter.h"

#include <utility>

KalmanFilter::KalmanFilter(int stateDim, int measurementDim) {
    MatrixXf processModel(stateDim, stateDim);
    setProcessModel(processModel);

    MatrixXf measurementModel(measurementDim, stateDim);
    setMeasurementModel(measurementModel);

    MatrixXf processNoise(stateDim, stateDim);
    setProcessNoise(processNoise);

    MatrixXf measurementNoise(measurementDim, measurementDim);
    setMeasurementNoise(measurementNoise);

    MatrixXf initialCovariance(stateDim, stateDim);
    initialCovariance.setZero();
    setCovariance(initialCovariance);

    VectorXf initialState(stateDim);
    initialState.setZero();
    setState(initialState);

    VectorXf measurement(measurementDim);
    setMeasurement(measurement);
}

KalmanFilter::KalmanFilter(const KalmanFilter &filter) {
    setProcessModel(filter.getProcessModel());
    setMeasurementModel(filter.getMeasurementModel());
    setProcessNoise(filter.getProcessNoise());
    setMeasurementNoise(filter.getMeasurementNoise());
    setCovariance(filter.getCovariance());
    setState(filter.getState());
    setMeasurement(filter.getMeasurement());
}

void KalmanFilter::prediction() {
    // Actually determining the priori knowledge

    // X = F * X
    _state = _processModel * _state;
    // P = F * P * F.transpose() + Q;
    _covariance = _processModel * _covariance * _processModel.transpose() + _processNoise;
}

void KalmanFilter::update() {
    // VectorXf predictionError = Y - H * X;
    VectorXf predictionError = _measurement - _measurementModel * _state;
    // MatrixXf S = H * P * H.transpose() + R;
    MatrixXf S = _measurementModel * _covariance * _measurementModel.transpose() + _measurementNoise;
    MatrixXf SInverse = S.inverse();
    // MatrixXf K = P * H.transpose() * SInverse;
    MatrixXf K = _covariance * _measurementModel.transpose() * SInverse;
    // X = X + K * predictionError;
    _state = _state + K * predictionError;
    // P = P - K * H * P;
    _covariance = _covariance - K * _measurementModel * _covariance;
}

void KalmanFilter::setProcessModel(const MatrixXf &processModel) {
    _processModel = processModel;
}

void KalmanFilter::setMeasurementModel(const MatrixXf &measurementModel) {
    _measurementModel = measurementModel;
}

void KalmanFilter::setCovariance(const MatrixXf &covariance) {
    _covariance = covariance;
}

void KalmanFilter::setState(const VectorXf &state) {
    _state = state;
}

void KalmanFilter::setProcessNoise(const MatrixXf &processNoise) {
    _processNoise = processNoise;
}

void KalmanFilter::setMeasurementNoise(const MatrixXf &measurementNoise) {
    _measurementNoise = measurementNoise;
}

void KalmanFilter::setMeasurement(const VectorXf &measurement) {
    _measurement = measurement;
}

const MatrixXf &KalmanFilter::getCovariance() const {
    return _covariance;
}

const VectorXf &KalmanFilter::getState() const {
    return _state;
}

const MatrixXf &KalmanFilter::getProcessModel() const {
    return _processModel;
}

const MatrixXf &KalmanFilter::getMeasurementModel() const {
    return _measurementModel;
}

const MatrixXf &KalmanFilter::getProcessNoise() const {
    return _processNoise;
}

const MatrixXf &KalmanFilter::getMeasurementNoise() const {
    return _measurementNoise;
}

const VectorXf &KalmanFilter::getMeasurement() const {
    return _measurement;
}


