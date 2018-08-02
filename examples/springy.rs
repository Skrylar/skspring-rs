
extern crate skspringrs;
use skspringrs::*;

fn main() {
    let spring = DampedSpringController::with_coefficients(1.0, 10.0, 10.0);
    let mut v: f64 = 0.0;
    let mut p: f64 = 0.0;

    for _i in 0..10 {
        spring.update(&mut p, &mut v, 100.0);
        print!("{}\n", p);
    }
}

