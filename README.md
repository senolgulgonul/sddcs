SDDCS Tuner, supplementary material
Paper: Specification-Driven Differentiable Controller Synthesis

Files
  index.html  The tuner, v0.27 (2026-11-25). Open in any modern browser;
    no installation, no network access required. Also live at
    https://senolgulgonul.github.io/sddcs/
  migo_solver_20260718_1530.js  Exact MIGO solver (Node.js). Run
    node -e "const M=require('./migo_solver_20260718_1530.js');
    console.log(M.selfTest()); console.log(M.bookValidation());"
    to reproduce the pure-delay anchor and the worked designs of Astrom
    and Hagglund used in the validation table of the paper.

Conventions (v0.27)
  Settling time Ts is the time of the last sample outside the band.
    Versions up to v0.26 reported the first sample inside the band, one
    sampling interval later.
  Zero tolerance: TV0 <= 1e-9 and overshoot <= 1e-9, double precision.
  Defaults: dt = 0.025, 2 percent band, horizon 40.

Replaying the certified controllers of the paper
  Readout, Evaluate external gains, Interpretation "discrete factored".
  Enter  kI ; zeros ; poles besides z = 1.  The token P among the zeros
  places zeros exactly on all discrete plant poles. Set the plant first.
    P1 = e^-s/(2s+1), denominator "2 1"
      order 2:  0.598804 ; P 0.932900 ; 0.694214                     Ts 3.125
      order 3:  0.601934 ; P 0.933100 0.171674 ; 0.713303 -0.140123  Ts 3.100
    P2 = e^-s/((s+1)(0.5s+1)), denominator "0.5 1.5 1"
      order 2:  0.490462 ; 0.974653 0.965793 ; 0.083176              Ts 3.875
      order 3:  0.601562 ; P 0.932405 ; 0.658269 -0.717149           Ts 3.125
    P3 = e^-s/(s^2+s+1), denominator "1 1 1"
      order 2:  0.365940 ; P ; -0.493628                             Ts 6.450
      order 3:  0.601562 ; P 0.932405 ; 0.658269 -0.717149           Ts 3.125
  PI rows, Interpretation "continuous PIDF", enter Kp Ki:
      P1  0.8872569 0.4200906   Ts 4.875
      P2  0.419768 0.321956     Ts 6.250
      P3  -0.004822 0.197479    Ts 12.575
  Rounding the tap coefficients w, d of these controllers to six digits
  breaks the zero-tolerance test; use the factored form.

Other uses
  Train: gradient training from random restarts under specifications
    A (monotone), B (overshoot cap), C (MIGO), D (phase margin).
  Certify: zero-tolerance shrinking-grid search at orders 1 and 2.
    Order 3 certificates of the paper were computed offline.
  Evaluate external gains: paste pidtune or textbook gains to grade any
    controller in the same instrument.
