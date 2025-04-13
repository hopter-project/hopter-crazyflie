#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use crate::{
    math::{mat_mult, mat_trans, ArmMatrixF32},
    module::controller::types::{
        AccelerationT, AttitudeRateT, AttitudeT, PositionT, StateT, VelocityT,
    },
};

pub const KC_STATE_X: usize = 0;
pub const KC_STATE_Y: usize = 1;
pub const KC_STATE_Z: usize = 2;
pub const KC_STATE_PX: usize = 3;
pub const KC_STATE_PY: usize = 4;
pub const KC_STATE_PZ: usize = 5;
pub const KC_STATE_D0: usize = 6;
pub const KC_STATE_D1: usize = 7;
pub const KC_STATE_D2: usize = 8;
pub const KC_STATE_DIM: usize = 9;

pub const MAX_COVARIANCE: f32 = 100.0;
pub const MIN_COVARIANCE: f32 = 1e-6;

pub const PROCESS_NOISE_ACCEL_XY: f32 = 0.5;
pub const PROCESS_NOISE_ACCEL_Z: f32 = 1.0;
pub const PROCESS_NOISE_POSITION: f32 = 0.0;
pub const PROCESS_NOISE_VELOCITY: f32 = 0.0;
pub const PROCESS_NOISE_ATTITUDE_RATE_RP: f32 = 0.1;
pub const PROCESS_NOISE_ATTITUDE_RATE_Y: f32 = 0.1;
pub const PROCESS_NOISE_ATTITUDE: f32 = 0.0;

pub const GRAVITY: f32 = 9.80665;
pub const EPS: f32 = 1e-6;
#[derive(Debug)]
pub struct KalmanCoreData {
    /**
     * Quadrocopter State
     *
     * The internally-estimated state is:
     * - X, Y, Z: the quad's position in the global frame
     * - PX, PY, PZ: the quad's velocity in its body frame
     * - D0, D1, D2: attitude error
     *
     * For more information, refer to the paper
     */
    pub S: [f32; KC_STATE_DIM],

    // The quad's attitude as a quaternion (w,x,y,z)
    // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
    // while also being robust against singularities (in comparison to euler angles)
    pub q: [f32; 4],

    // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
    // body: x -> global: X
    pub R: [[f32; 3]; 3],

    // The covariance matrix, estimate uncertainty
    // P: [[f32; KC_STATE_DIM]; KC_STATE_DIM],
    pub Pm: ArmMatrixF32,

    // intermediate results

    // Indicates that the internal state is corrupt and should be reset
    pub reset: bool,
}

impl KalmanCoreData {
    pub fn new() -> KalmanCoreData {
        let mut kcd = KalmanCoreData {
            S: [0.0; KC_STATE_DIM],
            q: [1.0, 0.0, 0.0, 0.0],
            R: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            Pm: ArmMatrixF32::new(KC_STATE_DIM, KC_STATE_DIM),
            reset: false,
        };
        // Initial variances, uncertain of position, but know we're stationary and roughly flat
        kcd.Pm[KC_STATE_X][KC_STATE_X] = 100.0 * 100.0;
        kcd.Pm[KC_STATE_Y][KC_STATE_Y] = 100.0 * 100.0;
        kcd.Pm[KC_STATE_Z][KC_STATE_Z] = 1.0 * 1.0;
        kcd.Pm[KC_STATE_PX][KC_STATE_PX] = 0.01 * 0.01;
        kcd.Pm[KC_STATE_PY][KC_STATE_PY] = 0.01 * 0.01;
        kcd.Pm[KC_STATE_PZ][KC_STATE_PZ] = 0.01 * 0.01;
        kcd.Pm[KC_STATE_D0][KC_STATE_D0] = 0.01 * 0.01;
        kcd.Pm[KC_STATE_D1][KC_STATE_D1] = 0.01 * 0.01;
        kcd.Pm[KC_STATE_D2][KC_STATE_D2] = 0.01 * 0.01;
        kcd
    }

    pub fn update_scalar(&mut self, Hm: &ArmMatrixF32, error: f32, std_meas_noise: f32) {
        if Hm.row_count() != 1 || Hm.column_count() != KC_STATE_DIM {
            panic!("Hm must be 1x9");
        }

        // H^T
        let HTm = mat_trans(Hm);
        // P_n,n-1 * H^T
        let PHTm = mat_mult(&self.Pm, &HTm);
        // R_n
        let R = std_meas_noise * std_meas_noise;
        let mut HPHR = R;
        // Add the element of HPH' to HPHR
        let Hm_data = Hm.as_slice();
        let PHTm_data = PHTm.as_slice();
        for i in 0..KC_STATE_DIM {
            HPHR += Hm_data[i] * PHTm_data[i];
        }

        // ====== MEASUREMENT UPDATE: K_n = P_n,n-1 * H^T * (H * P_n,n-1 * H^T + R_n)^-1 ======
        // Calculate the Kalman gain and perform the state update
        let mut Km = ArmMatrixF32::new(KC_STATE_DIM, 1);
        {
            let Km_data = Km.as_mut_slice();
            for i in 0..KC_STATE_DIM {
                Km_data[i] = PHTm_data[i] / HPHR;
                self.S[i] += Km_data[i] * error;
            }
        }

        // ====== COVARIANCE UPDATE: P_n,n = (K_n * H - I) * P_n,n-1 * (K_n * H - I)^T + K_n * R_n * K_n^T ======
        // K_n * H
        let mut T1m = mat_mult(&Km, Hm);
        // K_n * H - I
        let T1m_data = T1m.as_mut_slice();
        for i in 0..KC_STATE_DIM {
            T1m_data[i * KC_STATE_DIM + i] -= 1.0;
        }
        // (K_n * H - I)^T
        let T1Tm = mat_trans(&T1m);
        // (K_n * H - I) * P_n,n-1
        let T2m = mat_mult(&T1m, &self.Pm);
        // (K_n * H - I) * P_n,n-1 * (K_n * H - I)^T
        self.Pm = mat_mult(&T2m, &T1Tm);

        // add the measurement variance and ensure boundedness and symmetry
        // TODO: Why would it hit these bounds? Needs to be investigated.
        // K_n * R_n * K_n^T
        let Km_data = Km.as_mut_slice();
        let Pm_data = self.Pm.as_mut_slice();
        for i in 0..KC_STATE_DIM {
            for j in 0..KC_STATE_DIM {
                let v = Km_data[i] * R * Km_data[j];
                let p = 0.5 * (Pm_data[i * KC_STATE_DIM + j] + Pm_data[j * KC_STATE_DIM + i]) + v;
                if p > MAX_COVARIANCE {
                    Pm_data[i * KC_STATE_DIM + j] = MAX_COVARIANCE;
                    Pm_data[j * KC_STATE_DIM + i] = MAX_COVARIANCE;
                } else if i == j && p < MIN_COVARIANCE {
                    Pm_data[i * KC_STATE_DIM + j] = MIN_COVARIANCE;
                } else {
                    Pm_data[i * KC_STATE_DIM + j] = p;
                    Pm_data[j * KC_STATE_DIM + i] = p;
                }
            }
        }
    }

    pub fn predict(&mut self, accel: AccelerationT, gyro: AttitudeRateT, dt: f32, is_flying: bool) {
        let mut Fm = ArmMatrixF32::new(KC_STATE_DIM, KC_STATE_DIM);
        let Fm_data = Fm.as_mut_slice();
        // Initialize as the identity
        for i in 0..KC_STATE_DIM {
            Fm_data[i * KC_STATE_DIM + i] = 1.0;
        }
        // position from body-frame velocity
        Fm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_PX] = self.R[0][0] * dt;
        Fm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_PX] = self.R[1][0] * dt;
        Fm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_PX] = self.R[2][0] * dt;

        Fm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_PY] = self.R[0][1] * dt;
        Fm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_PY] = self.R[1][1] * dt;
        Fm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_PY] = self.R[2][1] * dt;

        Fm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_PZ] = self.R[0][2] * dt;
        Fm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_PZ] = self.R[1][2] * dt;
        Fm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_PZ] = self.R[2][2] * dt;

        // position from attitude error
        Fm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_D0] =
            (self.S[KC_STATE_PY] * self.R[0][2] - self.S[KC_STATE_PZ] * self.R[0][1]) * dt;
        Fm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_D0] =
            (self.S[KC_STATE_PY] * self.R[1][2] - self.S[KC_STATE_PZ] * self.R[1][1]) * dt;
        Fm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_D0] =
            (self.S[KC_STATE_PY] * self.R[2][2] - self.S[KC_STATE_PZ] * self.R[2][1]) * dt;

        Fm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_D1] =
            (-self.S[KC_STATE_PX] * self.R[0][2] + self.S[KC_STATE_PZ] * self.R[0][0]) * dt;
        Fm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_D1] =
            (-self.S[KC_STATE_PX] * self.R[1][2] + self.S[KC_STATE_PZ] * self.R[1][0]) * dt;
        Fm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_D1] =
            (-self.S[KC_STATE_PX] * self.R[2][2] + self.S[KC_STATE_PZ] * self.R[2][0]) * dt;

        Fm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_D2] =
            (self.S[KC_STATE_PX] * self.R[0][1] - self.S[KC_STATE_PY] * self.R[0][0]) * dt;
        Fm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_D2] =
            (self.S[KC_STATE_PX] * self.R[1][1] - self.S[KC_STATE_PY] * self.R[1][0]) * dt;
        Fm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_D2] =
            (self.S[KC_STATE_PX] * self.R[2][1] - self.S[KC_STATE_PY] * self.R[2][0]) * dt;

        // body-frame velocity change from attitude change (rotation)
        Fm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_PX] = 1.0;
        Fm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_PX] = -gyro.yaw * dt;
        Fm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_PX] = gyro.pitch * dt;

        Fm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_PY] = gyro.yaw * dt;
        Fm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_PY] = 1.0;
        Fm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_PY] = -gyro.roll * dt;

        Fm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_PZ] = -gyro.pitch * dt;
        Fm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_PZ] = gyro.roll * dt;
        Fm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_PZ] = 1.0;

        // body-frame velocity from attitude error
        Fm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_D0] = 0.0;
        Fm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_D0] = -GRAVITY * self.R[2][2] * dt;
        Fm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_D0] = GRAVITY * self.R[2][1] * dt;

        Fm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_D1] = GRAVITY * self.R[2][2] * dt;
        Fm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_D1] = 0.0;
        Fm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_D1] = -GRAVITY * self.R[2][0] * dt;

        Fm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_D2] = -GRAVITY * self.R[2][1] * dt;
        Fm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_D2] = GRAVITY * self.R[2][0] * dt;
        Fm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_D2] = 0.0;

        // attitude error from attitude error
        let d0 = gyro.roll * dt / 2.0;
        let d1 = gyro.pitch * dt / 2.0;
        let d2 = gyro.yaw * dt / 2.0;

        Fm_data[KC_STATE_D0 * KC_STATE_DIM + KC_STATE_D0] = 1.0 - d1 * d1 / 2.0 - d2 * d2 / 2.0;
        Fm_data[KC_STATE_D0 * KC_STATE_DIM + KC_STATE_D1] = d2 + d0 * d1 / 2.0;
        Fm_data[KC_STATE_D0 * KC_STATE_DIM + KC_STATE_D2] = -d1 + d0 * d2 / 2.0;

        Fm_data[KC_STATE_D1 * KC_STATE_DIM + KC_STATE_D0] = -d2 + d0 * d1 / 2.0;
        Fm_data[KC_STATE_D1 * KC_STATE_DIM + KC_STATE_D1] = 1.0 - d0 * d0 / 2.0 - d2 * d2 / 2.0;
        Fm_data[KC_STATE_D1 * KC_STATE_DIM + KC_STATE_D2] = d0 + d1 * d2 / 2.0;

        Fm_data[KC_STATE_D2 * KC_STATE_DIM + KC_STATE_D0] = d1 + d0 * d2 / 2.0;
        Fm_data[KC_STATE_D2 * KC_STATE_DIM + KC_STATE_D1] = -d0 + d1 * d2 / 2.0;
        Fm_data[KC_STATE_D2 * KC_STATE_DIM + KC_STATE_D2] = 1.0 - d0 * d0 / 2.0 - d1 * d1 / 2.0;

        // ====== COVARIANCE UPDATE: P_n+1,n = F * P_n,n * F^T + Q ======
        // F * P_n,n
        let FPm = mat_mult(&Fm, &self.Pm);
        let FTm = mat_trans(&Fm);
        // F * P_n,n * F^T
        self.Pm = mat_mult(&FPm, &FTm);

        // ====== PREDICTION STEP: S_n+1,n = F * S_n,n + G * u_n + w_n ======
        // The control input u_n depends on whether we're on the ground, or in flight.
        // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)
        let dt2_2 = dt * dt / 2.0;
        if is_flying {
            // only acceleration in z direction
            // Use accelerometer and not commanded thrust, as this has proper physical units

            // position updates in the body frame (will be rotated to inertial frame
            let dx = self.S[KC_STATE_PX] * dt;
            let dy = self.S[KC_STATE_PY] * dt;
            let dz = self.S[KC_STATE_PZ] * dt + accel.z * dt2_2;

            // position update
            self.S[KC_STATE_X] += self.R[0][0] * dx + self.R[0][1] * dy + self.R[0][2] * dz;
            self.S[KC_STATE_Y] += self.R[1][0] * dx + self.R[1][1] * dy + self.R[1][2] * dz;
            self.S[KC_STATE_Z] +=
                self.R[2][0] * dx + self.R[2][1] * dy + self.R[2][2] * dz - GRAVITY * dt2_2;

            // keep previous time step's state for the update
            let tmp_spx = self.S[KC_STATE_PX];
            let tmp_spy = self.S[KC_STATE_PY];
            let tmp_spz = self.S[KC_STATE_PZ];

            // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
            self.S[KC_STATE_PX] +=
                dt * (gyro.yaw * tmp_spy - gyro.pitch * tmp_spz - GRAVITY * self.R[2][0]);
            self.S[KC_STATE_PY] +=
                dt * (-gyro.yaw * tmp_spx + gyro.roll * tmp_spz - GRAVITY * self.R[2][1]);
            self.S[KC_STATE_PZ] += dt
                * (accel.z + gyro.pitch * tmp_spx - gyro.roll * tmp_spy - GRAVITY * self.R[2][2]);
        } else {
            // Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
            // position updates in the body frame (will be rotated to inertial frame)
            let dx = self.S[KC_STATE_PX] * dt + accel.x * dt2_2;
            let dy = self.S[KC_STATE_PY] * dt + accel.y * dt2_2;
            let dz = self.S[KC_STATE_PZ] * dt + accel.z * dt2_2; // thrust can only be produced in the body's Z direction

            // position update
            self.S[KC_STATE_X] += self.R[0][0] * dx + self.R[0][1] * dy + self.R[0][2] * dz;
            self.S[KC_STATE_Y] += self.R[1][0] * dx + self.R[1][1] * dy + self.R[1][2] * dz;
            self.S[KC_STATE_Z] +=
                self.R[2][0] * dx + self.R[2][1] * dy + self.R[2][2] * dz - GRAVITY * dt2_2;

            // keep previous time step's state for the update
            let tmp_spx = self.S[KC_STATE_PX];
            let tmp_spy = self.S[KC_STATE_PY];
            let tmp_spz = self.S[KC_STATE_PZ];

            // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
            self.S[KC_STATE_PX] +=
                dt * (accel.x + gyro.yaw * tmp_spy - gyro.pitch * tmp_spz - GRAVITY * self.R[2][0]);
            self.S[KC_STATE_PY] +=
                dt * (accel.y - gyro.yaw * tmp_spx + gyro.roll * tmp_spz - GRAVITY * self.R[2][1]);
            self.S[KC_STATE_PZ] += dt
                * (accel.z + gyro.pitch * tmp_spx - gyro.roll * tmp_spy - GRAVITY * self.R[2][2]);
        }

        // attitude update (rotate by gyroscope), we do this in quaternions
        // this is the gyroscope angular velocity integrated over the sample period
        let dt_wx = dt * gyro.roll;
        let dt_wy = dt * gyro.pitch;
        let dt_wz = dt * gyro.yaw;

        // compute the quaternion values in [w,x,y,z] order
        let angle = libm::sqrtf(dt_wx * dt_wx + dt_wy * dt_wy + dt_wz * dt_wz) + EPS;
        let c = libm::cosf(angle / 2.0);
        let s = libm::sinf(angle / 2.0) / angle;
        let dq = [c, s * dt_wx, s * dt_wy, s * dt_wz];

        // rotate the quad's attitude by the delta quaternion vector computed above
        let mut tmp_q0 =
            dq[0] * self.q[0] - dq[1] * self.q[1] - dq[2] * self.q[2] - dq[3] * self.q[3];
        let mut tmp_q1 =
            dq[1] * self.q[0] + dq[0] * self.q[1] + dq[3] * self.q[2] - dq[2] * self.q[3];
        let mut tmp_q2 =
            dq[2] * self.q[0] - dq[3] * self.q[1] + dq[0] * self.q[2] + dq[1] * self.q[3];
        let mut tmp_q3 =
            dq[3] * self.q[0] + dq[2] * self.q[1] - dq[1] * self.q[2] + dq[0] * self.q[3];

        if !is_flying {
            let keep = 1.0 - 0.001;
            tmp_q0 = keep * tmp_q0 + 0.001 * 1.0;
            tmp_q1 = keep * tmp_q1 + 0.001 * 0.0;
            tmp_q2 = keep * tmp_q2 + 0.001 * 0.0;
            tmp_q3 = keep * tmp_q3 + 0.001 * 0.0;
        }

        // normalize and store the result
        let norm =
            libm::sqrtf(tmp_q0 * tmp_q0 + tmp_q1 * tmp_q1 + tmp_q2 * tmp_q2 + tmp_q3 * tmp_q3)
                + EPS;
        self.q[0] = tmp_q0 / norm;
        self.q[1] = tmp_q1 / norm;
        self.q[2] = tmp_q2 / norm;
        self.q[3] = tmp_q3 / norm;
    }

    pub fn add_process_noise(&mut self, dt: f32) {
        let Pm_data = self.Pm.as_mut_slice();
        if dt > 0.0 {
            let dt2 = dt * dt;

            let noise_xy =
                PROCESS_NOISE_ACCEL_XY * dt2 + PROCESS_NOISE_VELOCITY * dt + PROCESS_NOISE_POSITION;
            let noise_z =
                PROCESS_NOISE_ACCEL_Z * dt2 + PROCESS_NOISE_VELOCITY * dt + PROCESS_NOISE_POSITION;
            Pm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_X] += noise_xy * noise_xy;
            Pm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_Y] += noise_xy * noise_xy;
            Pm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_Z] += noise_z * noise_z;

            let noise_pxy = PROCESS_NOISE_ACCEL_XY * dt + PROCESS_NOISE_VELOCITY;
            let noise_pz = PROCESS_NOISE_ACCEL_Z * dt + PROCESS_NOISE_VELOCITY;
            Pm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_PX] += noise_pxy * noise_pxy;
            Pm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_PY] += noise_pxy * noise_pxy;
            Pm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_PZ] += noise_pz * noise_pz;

            let noise_rp = PROCESS_NOISE_ATTITUDE_RATE_RP * dt + PROCESS_NOISE_ATTITUDE;
            let noise_y = PROCESS_NOISE_ATTITUDE_RATE_Y * dt + PROCESS_NOISE_ATTITUDE;
            Pm_data[KC_STATE_D0 * KC_STATE_DIM + KC_STATE_D0] += noise_rp * noise_rp;
            Pm_data[KC_STATE_D1 * KC_STATE_DIM + KC_STATE_D1] += noise_rp * noise_rp;
            Pm_data[KC_STATE_D2 * KC_STATE_DIM + KC_STATE_D2] += noise_y * noise_y;
        }

        for i in 0..KC_STATE_DIM {
            for j in 0..KC_STATE_DIM {
                let p = 0.5 * (Pm_data[i * KC_STATE_DIM + j] + Pm_data[j * KC_STATE_DIM + i]);
                if p > MAX_COVARIANCE {
                    Pm_data[i * KC_STATE_DIM + j] = MAX_COVARIANCE;
                    Pm_data[j * KC_STATE_DIM + i] = MAX_COVARIANCE;
                } else if i == j && p < MIN_COVARIANCE {
                    Pm_data[i * KC_STATE_DIM + j] = MIN_COVARIANCE;
                } else {
                    Pm_data[i * KC_STATE_DIM + j] = p;
                    Pm_data[j * KC_STATE_DIM + i] = p;
                }
            }
        }
    }

    pub fn finalize(&mut self) {
        #[cfg(feature = "estimator_panic")]
        {
            use crate::flight_tasks::ESTIMATOR_PANIC_REQUIRED;
            use core::sync::atomic::Ordering;
            if ESTIMATOR_PANIC_REQUIRED.load(Ordering::Relaxed) {
                ESTIMATOR_PANIC_REQUIRED.store(false, Ordering::Relaxed);
                panic!()
            }
        }

        // Incorporate the attitude error (Kalman filter state) with the attitude
        let v0 = self.S[KC_STATE_D0];
        let v1 = self.S[KC_STATE_D1];
        let v2 = self.S[KC_STATE_D2];

        // Move attitude error into attitude if any of the angle errors are large enough
        if (libm::fabsf(v0) > 1e-4 || libm::fabsf(v1) > 1e-4 || libm::fabsf(v2) > 1e-4)
            && libm::fabsf(v0) < 10.0
            && libm::fabsf(v1) < 10.0
            && libm::fabsf(v2) < 10.0
        {
            let angle = libm::sqrtf(v0 * v0 + v1 * v1 + v2 * v2) + EPS;
            let c = libm::cosf(angle / 2.0);
            let s = libm::sinf(angle / 2.0) / angle;
            let dq = [c, v0 * s, v1 * s, v2 * s];

            let q0 = dq[0] * self.q[0] - dq[1] * self.q[1] - dq[2] * self.q[2] - dq[3] * self.q[3];
            let q1 = dq[1] * self.q[0] + dq[0] * self.q[1] + dq[3] * self.q[2] - dq[2] * self.q[3];
            let q2 = dq[2] * self.q[0] - dq[3] * self.q[1] + dq[0] * self.q[2] + dq[1] * self.q[3];
            let q3 = dq[3] * self.q[0] + dq[2] * self.q[1] - dq[1] * self.q[2] + dq[0] * self.q[3];

            let norm = libm::sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) + EPS;
            self.q[0] = q0 / norm;
            self.q[1] = q1 / norm;
            self.q[2] = q2 / norm;
            self.q[3] = q3 / norm;

            /* Rotate the covariance, since we've rotated the body
             *
             * This comes from a second order approximation to:
             * Sigma_post = exps(-d) Sigma_pre exps(-d)'
             *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
             * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
             *
             * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
             * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
             */
            let d0 = v0 / 2.0; // the attitude error vector (v0,v1,v2) is small,
            let d1 = v1 / 2.0; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
            let d2 = v2 / 2.0; // and similarly for d1 and d2
            let mut Fm = ArmMatrixF32::new(KC_STATE_DIM, KC_STATE_DIM);
            let Fm_data = Fm.as_mut_slice();
            Fm_data[KC_STATE_X * KC_STATE_DIM + KC_STATE_X] = 1.0;
            Fm_data[KC_STATE_Y * KC_STATE_DIM + KC_STATE_Y] = 1.0;
            Fm_data[KC_STATE_Z * KC_STATE_DIM + KC_STATE_Z] = 1.0;
            Fm_data[KC_STATE_PX * KC_STATE_DIM + KC_STATE_PX] = 1.0;
            Fm_data[KC_STATE_PY * KC_STATE_DIM + KC_STATE_PY] = 1.0;
            Fm_data[KC_STATE_PZ * KC_STATE_DIM + KC_STATE_PZ] = 1.0;
            Fm_data[KC_STATE_D0 * KC_STATE_DIM + KC_STATE_D0] = 1.0 - d1 * d1 / 2.0 - d2 * d2 / 2.0;
            Fm_data[KC_STATE_D0 * KC_STATE_DIM + KC_STATE_D1] = d2 + d0 * d1 / 2.0;
            Fm_data[KC_STATE_D0 * KC_STATE_DIM + KC_STATE_D2] = -d1 + d0 * d2 / 2.0;
            Fm_data[KC_STATE_D1 * KC_STATE_DIM + KC_STATE_D0] = -d2 + d0 * d1 / 2.0;
            Fm_data[KC_STATE_D1 * KC_STATE_DIM + KC_STATE_D1] = 1.0 - d0 * d0 / 2.0 - d2 * d2 / 2.0;
            Fm_data[KC_STATE_D1 * KC_STATE_DIM + KC_STATE_D2] = d0 + d1 * d2 / 2.0;
            Fm_data[KC_STATE_D2 * KC_STATE_DIM + KC_STATE_D0] = d1 + d0 * d2 / 2.0;
            Fm_data[KC_STATE_D2 * KC_STATE_DIM + KC_STATE_D1] = -d0 + d1 * d2 / 2.0;
            Fm_data[KC_STATE_D2 * KC_STATE_DIM + KC_STATE_D2] = 1.0 - d0 * d0 / 2.0 - d1 * d1 / 2.0;
            let FTm = mat_trans(&Fm);
            let FPm = mat_mult(&Fm, &self.Pm);
            self.Pm = mat_mult(&FPm, &FTm);
        }

        // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
        self.R[0][0] = self.q[0] * self.q[0] + self.q[1] * self.q[1]
            - self.q[2] * self.q[2]
            - self.q[3] * self.q[3];
        self.R[0][1] = 2.0 * self.q[1] * self.q[2] - 2.0 * self.q[0] * self.q[3];
        self.R[0][2] = 2.0 * self.q[1] * self.q[3] + 2.0 * self.q[0] * self.q[2];

        self.R[1][0] = 2.0 * self.q[1] * self.q[2] + 2.0 * self.q[0] * self.q[3];
        self.R[1][1] = self.q[0] * self.q[0] - self.q[1] * self.q[1] + self.q[2] * self.q[2]
            - self.q[3] * self.q[3];
        self.R[1][2] = 2.0 * self.q[2] * self.q[3] - 2.0 * self.q[0] * self.q[1];

        self.R[2][0] = 2.0 * self.q[1] * self.q[3] - 2.0 * self.q[0] * self.q[2];
        self.R[2][1] = 2.0 * self.q[2] * self.q[3] + 2.0 * self.q[0] * self.q[1];
        self.R[2][2] = self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2]
            + self.q[3] * self.q[3];

        // reset the attitude error
        self.S[KC_STATE_D0] = 0.0;
        self.S[KC_STATE_D1] = 0.0;
        self.S[KC_STATE_D2] = 0.0;

        // enforce symmetry of the covariance matrix, and ensure the values stay bounded
        let Pm_data = &mut self.Pm.as_mut_slice();
        for i in 0..KC_STATE_DIM {
            for j in 0..KC_STATE_DIM {
                let p = 0.5 * (Pm_data[i * KC_STATE_DIM + j] + Pm_data[j * KC_STATE_DIM + i]);
                if p > MAX_COVARIANCE {
                    Pm_data[i * KC_STATE_DIM + j] = MAX_COVARIANCE;
                    Pm_data[j * KC_STATE_DIM + i] = MAX_COVARIANCE;
                } else if i == j && p < MIN_COVARIANCE {
                    Pm_data[i * KC_STATE_DIM + j] = MIN_COVARIANCE;
                } else {
                    Pm_data[i * KC_STATE_DIM + j] = p;
                    Pm_data[j * KC_STATE_DIM + i] = p;
                }
            }
        }
    }

    pub fn externalize_state(&self, latest_accel: AccelerationT) -> StateT {
        let mut state = StateT::default();
        state.position = PositionT {
            x: self.S[KC_STATE_X],
            y: self.S[KC_STATE_Y],
            z: self.S[KC_STATE_Z],
        };
        // velocity is in body frame and needs to be rotated to world frame
        state.velocity = VelocityT {
            x: self.R[0][0] * self.S[KC_STATE_PX]
                + self.R[0][1] * self.S[KC_STATE_PY]
                + self.R[0][2] * self.S[KC_STATE_PZ],
            y: self.R[1][0] * self.S[KC_STATE_PX]
                + self.R[1][1] * self.S[KC_STATE_PY]
                + self.R[1][2] * self.S[KC_STATE_PZ],
            z: self.R[2][0] * self.S[KC_STATE_PX]
                + self.R[2][1] * self.S[KC_STATE_PY]
                + self.R[2][2] * self.S[KC_STATE_PZ],
        };

        // Accelerometer measurements are in the body frame and need to be rotated to world frame.
        // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
        // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
        state.accel = AccelerationT {
            x: self.R[0][0] * latest_accel.x
                + self.R[0][1] * latest_accel.y
                + self.R[0][2] * latest_accel.z,
            y: self.R[1][0] * latest_accel.x
                + self.R[1][1] * latest_accel.y
                + self.R[1][2] * latest_accel.z,
            z: self.R[2][0] * latest_accel.x
                + self.R[2][1] * latest_accel.y
                + self.R[2][2] * latest_accel.z
                - 1.0,
        };

        // convert the new attitude into Euler YPR
        let yaw = libm::atan2f(
            2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1]
                - self.q[2] * self.q[2]
                - self.q[3] * self.q[3],
        );
        let pitch = -libm::asinf(-2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2]));
        let roll = libm::atan2f(
            2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2]
                + self.q[3] * self.q[3],
        );
        state.attitude = AttitudeT {
            yaw: yaw.to_degrees(),
            pitch: pitch.to_degrees(),
            roll: roll.to_degrees(),
        };

        state
    }

    pub fn check_bound(&self) -> bool {
        let MAX_POSITION = 100.0;
        let MAX_VELOCITY = 10.0;
        for i in 0..3 {
            if self.S[KC_STATE_X + i] > MAX_POSITION || self.S[KC_STATE_X + i] < -MAX_POSITION {
                return false;
            }

            if self.S[KC_STATE_PX + i] > MAX_VELOCITY || self.S[KC_STATE_PX + i] < -MAX_VELOCITY {
                return false;
            }
        }
        return true;
    }
}
