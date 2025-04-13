use crate::{
    math::ArmMatrixF32,
    module::{controller::types::AttitudeT, flow::FlowData, tof::ToFData},
};

use super::kalman::{KalmanCoreData, KC_STATE_DIM, KC_STATE_PX, KC_STATE_PY, KC_STATE_Z};

impl KalmanCoreData {
    pub fn update_with_tof(&mut self, tof_data: &ToFData) {
        // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
        if self.R[2][2] > 0.1 {
            let mut hm = ArmMatrixF32::new(1, KC_STATE_DIM);
            let hm_data = hm.as_mut_slice();
            // let mut angle = libm::fabsf(libm::cosf(self.R[2][2])) - 7.5_f32.to_radians();
            // angle = angle.max(0.0);

            let error = tof_data.distance - self.S[KC_STATE_Z] / self.R[2][2];
            hm_data[KC_STATE_Z] = 1.0 / self.R[2][2];
            self.update_scalar(&hm, error, tof_data.std_dev);
        }
    }

    pub fn update_with_flow(&mut self, flow_data: &FlowData, gyro: &AttitudeT) {
        // Inclusion of flow measurements in the EKF done by two scalar updates
        // ~~~ Camera constants ~~~
        // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
        let pixel_number: f32 = 30.0; // [pixels] (same in x and y)
        let thetapix: f32 = 4.2_f32.to_radians();
        //~~~ Body rates ~~~
        // TODO check if this is feasible or if some filtering has to be done
        let omega_factor: f32 = 1.25;
        let omegax_b = gyro.roll.to_radians();
        let omegay_b = gyro.pitch.to_radians();

        // ~~~ Moves the body velocity into the global coordinate system ~~~
        // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
        //
        // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
        // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
        //
        // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
        // _G and _B refer to the global/body coordinate systems.

        // Modification 1
        //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * S[KC_STATE_PZ];
        //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * S[KC_STATE_PZ];
        let dx_g = self.S[KC_STATE_PX];
        let dy_g = self.S[KC_STATE_PY];
        let z_g = if self.S[KC_STATE_Z] < 0.1 {
            0.1
        } else {
            self.S[KC_STATE_Z]
        };
        // ~~~ X velocity prediction and update ~~~
        // predics the number of accumulated pixels in the x-direction
        let mut hxm = ArmMatrixF32::new(1, KC_STATE_DIM);
        let hxm_data = hxm.as_mut_slice();
        hxm_data[KC_STATE_Z] =
            pixel_number * flow_data.dt / thetapix * (dx_g * self.R[2][2] / (-z_g * z_g));
        hxm_data[KC_STATE_PX] = pixel_number * flow_data.dt / thetapix * (self.R[2][2] / z_g);
        let error_x = flow_data.delta_x
            - flow_data.dt * pixel_number / thetapix
                * ((dx_g * self.R[2][2] / z_g) - omega_factor * omegay_b);

        self.update_scalar(&hxm, error_x, flow_data.std_dev_x);

        // ~~~ Y velocity prediction and update ~~~
        // predics the number of accumulated pixels in the y-direction
        let mut hym = ArmMatrixF32::new(1, KC_STATE_DIM);
        let hym_data = hym.as_mut_slice();
        hym_data[KC_STATE_Z] =
            pixel_number * flow_data.dt / thetapix * (dy_g * self.R[2][2] / (-z_g * z_g));
        hym_data[KC_STATE_PY] = pixel_number * flow_data.dt / thetapix * (self.R[2][2] / z_g);
        let error_y = flow_data.delta_y
            - flow_data.dt * pixel_number / thetapix
                * ((dy_g * self.R[2][2] / z_g) + omega_factor * omegax_b);

        self.update_scalar(&hym, error_y, flow_data.std_dev_y);
    }
}
