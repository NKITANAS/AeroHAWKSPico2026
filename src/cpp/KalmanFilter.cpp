#include "KalmanFilter.h"

#pragma region Constructor
/// @brief Default constructor. Call init() before using the filter.
KalmanFilter::KalmanFilter()
    : m_x_alt(0.0f), m_x_vel(0.0f),
      m_p00(1.0f), m_p01(0.0f),
      m_p10(0.0f), m_p11(1.0f),
      m_accel_variance(1.0f),
      m_altitude_variance(1.0f)
{
}
#pragma endregion

#pragma region Init
/// @brief Initialize the Kalman filter state and noise parameters.
/// @param initial_altitude Starting altitude in meters.
/// @param accel_variance Process noise variance for accelerometer (m/s^2)^2.
///        Higher value = less trust in accelerometer. Typical: 0.5 - 4.0
/// @param altitude_variance Measurement noise variance for barometric altitude (m)^2.
///        Higher value = less trust in barometer. Typical: 0.5 - 5.0
void KalmanFilter::init(float initial_altitude, float accel_variance, float altitude_variance)
{
    m_x_alt = initial_altitude;
    m_x_vel = 0.0f;

    // Initial covariance - moderately uncertain
    m_p00 = 1.0f;  m_p01 = 0.0f;
    m_p10 = 0.0f;  m_p11 = 1.0f;

    m_accel_variance   = accel_variance;
    m_altitude_variance = altitude_variance;
}
#pragma endregion

#pragma region Update
/// @brief Run one predict + update cycle of the Kalman filter (with barometer).
///
/// @param accel_z Vertical acceleration in m/s^2 (gravity-compensated, positive = up).
/// @param baro_altitude Barometric altitude measurement in meters.
/// @param dt Time step in seconds.
void KalmanFilter::update(float accel_z, float baro_altitude, float dt)
{
    predict(accel_z, dt);
    correct(baro_altitude);
}
#pragma endregion

#pragma region Update (no barometer)
/// @brief Run a predict-only cycle without barometer correction.
///        Velocity and altitude are estimated purely from accelerometer integration.
///        Covariance will grow over time without measurement corrections.
///
/// @param accel_z Vertical acceleration in m/s^2 (gravity-compensated, positive = up).
/// @param dt Time step in seconds.
void KalmanFilter::update(float accel_z, float dt)
{
    predict(accel_z, dt);
}
#pragma endregion

#pragma region Predict
/// @brief Internal prediction step.
///
/// State transition with acceleration as control input:
///   x_alt = x_alt + x_vel * dt + 0.5 * accel_z * dt^2
///   x_vel = x_vel + accel_z * dt
///   P = F * P * F^T + Q
///
/// @param accel_z Vertical acceleration in m/s^2 (gravity-compensated, positive = up).
/// @param dt Time step in seconds.
void KalmanFilter::predict(float accel_z, float dt)
{
    // State prediction:
    //   F = | 1  dt |    B = | 0.5*dt^2 |
    //       | 0   1 |        |    dt    |
    m_x_alt += m_x_vel * dt + 0.5f * accel_z * dt * dt;
    m_x_vel += accel_z * dt;

    // Covariance prediction: P = F * P * F^T + Q
    // F * P:
    //   | 1  dt | * | p00  p01 | = | p00 + dt*p10    p01 + dt*p11 |
    //   | 0   1 |   | p10  p11 |   |       p10              p11   |
    float fp00 = m_p00 + dt * m_p10;
    float fp01 = m_p01 + dt * m_p11;
    float fp10 = m_p10;
    float fp11 = m_p11;

    // (F * P) * F^T:
    //   | fp00 fp01 | * | 1   0 | = | fp00 + fp01*dt    fp01 |
    //   | fp10 fp11 |   | dt  1 |   | fp10 + fp11*dt    fp11 |
    m_p00 = fp00 + fp01 * dt;
    m_p01 = fp01;
    m_p10 = fp10 + fp11 * dt;
    m_p11 = fp11;

    // Add process noise Q:
    //   Q = | 0.25*dt^4  0.5*dt^3 | * accel_variance
    //       | 0.5*dt^3     dt^2   |
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    m_p00 += 0.25f * dt4 * m_accel_variance;
    m_p01 += 0.5f  * dt3 * m_accel_variance;
    m_p10 += 0.5f  * dt3 * m_accel_variance;
    m_p11 +=         dt2 * m_accel_variance;
}
#pragma endregion

#pragma region Correct
/// @brief Internal measurement correction step.
///
/// Correct with barometric altitude measurement:
///   y = baro_altitude - x_alt        (innovation)
///   S = H * P * H^T + R              (innovation covariance)
///   K = P * H^T * S^-1               (Kalman gain)
///   x = x + K * y                    (state update)
///   P = (I - K * H) * P              (covariance update)
///
/// @param baro_altitude Barometric altitude measurement in meters.
void KalmanFilter::correct(float baro_altitude)
{
    // Measurement model: H = | 1  0 |  (we measure altitude)
    // Innovation: y = z - H * x = baro_altitude - x_alt
    float y = baro_altitude - m_x_alt;

    // Innovation covariance: S = H * P * H^T + R = P[0][0] + R
    float s = m_p00 + m_altitude_variance;

    // Kalman gain: K = P * H^T / S = | P[0][0] / S |
    //                                  | P[1][0] / S |
    float k0 = m_p00 / s;
    float k1 = m_p10 / s;

    // State update: x = x + K * y
    m_x_alt += k0 * y;
    m_x_vel += k1 * y;

    // Covariance update: P = (I - K * H) * P
    // (I - K*H) = | 1-k0  0 |
    //             | -k1   1 |
    float new_p00 = m_p00 - k0 * m_p00;
    float new_p01 = m_p01 - k0 * m_p01;
    float new_p10 = m_p10 - k1 * m_p00;
    float new_p11 = m_p11 - k1 * m_p01;

    m_p00 = new_p00;
    m_p01 = new_p01;
    m_p10 = new_p10;
    m_p11 = new_p11;
}
#pragma endregion
