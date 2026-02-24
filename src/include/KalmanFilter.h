#pragma once

#pragma region Includes
#include "pico/stdlib.h"
#pragma endregion

/// @brief A 2-state Kalman filter for estimating altitude and vertical velocity
///        by fusing accelerometer and barometric altitude measurements.
///
/// State vector:  x = [altitude, velocity]^T
/// Control input: u = vertical acceleration (m/s^2, gravity-compensated)
/// Measurement:   z = barometric altitude (m)
class KalmanFilter
{
    public:
        explicit KalmanFilter();

        /// @brief Initialize the filter with starting altitude and tuning parameters.
        /// @param initial_altitude Starting altitude in meters.
        /// @param accel_variance Process noise variance for accelerometer (m/s^2)^2.
        /// @param altitude_variance Measurement noise variance for barometric altitude (m)^2.
        void init(float initial_altitude, float accel_variance, float altitude_variance);

        /// @brief Run one predict + update cycle of the Kalman filter.
        /// @param accel_z Vertical acceleration in m/s^2 (gravity-compensated, positive = up).
        /// @param baro_altitude Barometric altitude measurement in meters.
        /// @param dt Time step in seconds.
        void update(float accel_z, float baro_altitude, float dt);

        /// @brief Run a predict-only cycle (no barometer correction).
        /// @param accel_z Vertical acceleration in m/s^2 (gravity-compensated, positive = up).
        /// @param dt Time step in seconds.
        void update(float accel_z, float dt);

        /// @brief Get the estimated altitude.
        float get_altitude() const { return m_x_alt; }

        /// @brief Get the estimated vertical velocity.
        float get_velocity() const { return m_x_vel; }

    private:
        // State estimates
        float m_x_alt; // Estimated altitude (m)
        float m_x_vel; // Estimated vertical velocity (m/s)

        // Error covariance matrix P (2x2, stored as 4 floats)
        float m_p00, m_p01;
        float m_p10, m_p11;

        // Tuning parameters
        float m_accel_variance;   // Process noise: accelerometer variance (m/s^2)^2
        float m_altitude_variance; // Measurement noise: barometric altitude variance (m)^2

        /// @brief Internal prediction step shared by both update overloads.
        void predict(float accel_z, float dt);

        /// @brief Internal measurement correction step using barometric altitude.
        void correct(float baro_altitude);
};
