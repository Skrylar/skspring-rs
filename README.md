# Physically Based Springs in Rust

A port of Ryan Juckett's damped spring models to Rust.

A `DampedSpringController` is created `with_coefficients` to specify the tension and angular velocity of a spring. The controller may then be used to update the position and velocity of any spring with similar characteristics.

Creating `DampedSpringController`s requires solving differential equations for that particular model, so it is best to cache them for use by multiple springs of similar parameters.

# Motivation
Damped spring models can be used in conjunction with interpolation systems to create fluid character animation out of few keyframes and a small amount of hand tuning:

https://www.gdcvault.com/play/1020583/Animation-Bootcamp-An-Indie-Approach

Physics-based animations are also useful for touch-based systems, as they allow the screen to appear more lifelike. Screens can be flung and rely on friction to slow them down, or spring models used to show toaster boxes or move scroll panels in to position.

# Reference
http://www.ryanjuckett.com/programming/damped-springs/
