// Damped spring motion

//  Copyright (c) 2008-2012 Ryan Juckett http://www.ryanjuckett.com/
//
//  This software is provided 'as-is', without any express or implied
//  warranty. In no event will the authors be held liable for any
//  damages arising from the use of this software.
//
//  Permission is granted to anyone to use this software for any purpose,
//  including commercial applications, and to alter it and redistribute
//  it freely, subject to the following restrictions:
//
//  1. The origin of this software must not be misrepresented; you
//  must not
//     claim that you wrote the original software. If you use this
//     software in a product, an acknowledgment in the product
//     documentation would be appreciated but is not required.
//
//  2. Altered source versions must be plainly marked as such, and must
//  not be
//     misrepresented as being the original software.
//
//  3. This notice may not be removed or altered from any source
//     distribution.

// Sk: Consider the following code an 'altered source version' in
// its entirety

extern crate num;
use num::Float;

/// Caches coefficients so that multiple springs with the same
/// parameters may be used without re-running trigonometry calls.
pub struct DampedSpringController<F> {
    pos_pos_coef: F,
    pos_vel_coef: F,
    vel_pos_coef: F,
    vel_vel_coef: F,
}

impl<F> DampedSpringController<F>
where
    F: num::Float,
    f64: Into<F>
{
    /// Performs (somewhat nontrivial) calculations to prepare the
    /// motion of a damped spring. Once initialized, you may update
    /// any spring with identical properties with `update` using the
    /// same object.
    pub fn with_coefficients(delta_time: F, desired_angular_frequency: F, desired_damping_ratio: F) -> DampedSpringController<F> {
        // clamping
        let damping_ratio     = if desired_damping_ratio     < 0.0.into() { 0.0.into() } else { desired_damping_ratio };
        let angular_frequency = if desired_angular_frequency < 0.0.into() { 0.0.into() } else { desired_angular_frequency };

        // special case: no angular frequency
        if angular_frequency < F::epsilon() {
            DampedSpringController{
                pos_pos_coef: 1.0.into(),
                pos_vel_coef: 0.0.into(),
                vel_pos_coef: 0.0.into(),
                vel_vel_coef: 1.0.into()}
        } else if damping_ratio > 1.0.into() + F::epsilon() { // over-damped
            let za = -angular_frequency * damping_ratio;
            let zb = angular_frequency * Float::sqrt(damping_ratio * damping_ratio - 1.0.into());
            let z1 = za - zb;
            let z2 = za + zb;
            let e1 = Float::exp(z1 * delta_time);
            let e2 = Float::exp(z2 * delta_time);
            let inv_two_zb = 1.0.into() / (2.0.into() * zb);
            let e1_over_twozb = e1 * inv_two_zb;
            let e2_over_twozb = e2 * inv_two_zb;
            let z1e1_over_twozb = z1 * e1_over_twozb;
            let z2e2_over_twozb = z2 * e2_over_twozb;
            DampedSpringController {
                pos_pos_coef: e1_over_twozb * z2 - z2e2_over_twozb + e2,
                pos_vel_coef: -e1_over_twozb + e2_over_twozb,
                vel_pos_coef: (z1e1_over_twozb - z2e2_over_twozb + e2) * z2,
                vel_vel_coef: -z1e1_over_twozb + z2e2_over_twozb
            }
        } else if damping_ratio < 1.0.into() - F::epsilon() { // under-damped
            let omega_zeta = angular_frequency * damping_ratio;
            let alpha = angular_frequency * Float::sqrt(1.0.into() - damping_ratio * damping_ratio);
            let exp_term = Float::exp(-omega_zeta * delta_time);
            let cos_term = Float::cos(alpha * delta_time);
            let sin_term = Float::sin(alpha * delta_time);
            let inv_alpha = 1.0.into() / alpha;
            let exp_sin = exp_term * sin_term;
            let exp_cos = exp_term * cos_term;
            let exp_omega_zeta_sin_over_alpha = exp_term * omega_zeta * sin_term * inv_alpha;
            DampedSpringController {
                pos_pos_coef: exp_cos + exp_omega_zeta_sin_over_alpha,
                pos_vel_coef: exp_sin * inv_alpha,
                vel_pos_coef: -exp_sin * alpha - omega_zeta * exp_omega_zeta_sin_over_alpha,
                vel_vel_coef: exp_cos - exp_omega_zeta_sin_over_alpha
            }
        } else { // critically damped
            let exp_term = Float::exp(-angular_frequency * delta_time);
            let time_exp = delta_time * exp_term;
            let time_exp_freq = time_exp * angular_frequency;
            DampedSpringController {
                pos_pos_coef: time_exp_freq + exp_term,
                pos_vel_coef: time_exp,
                vel_pos_coef: -angular_frequency * time_exp_freq,
                vel_vel_coef: -time_exp_freq + exp_term
            }
        }
    }

    /// Updates the position and velocity of a spring using this
    /// controller's physical properties.
    pub fn update(&self, position: &mut F, velocity: &mut F, target: F) {
        let old_pos = *position - target;
        let old_vel = *velocity;
        *position = old_pos * self.pos_pos_coef + old_vel * self.pos_vel_coef + target;
        *velocity = old_pos * self.vel_pos_coef + old_vel * self.vel_vel_coef;
    }
}

