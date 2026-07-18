SDDCS Tuner, supplementary material
Paper: Specification-Driven Differentiable Controller Synthesis

Files
  sddcs_tuner_v22_20260718_1130.html  The tuner. Open in any modern
    browser; no installation, no network access required. Also live at
    https://senolgulgonul.github.io/sddcs/
  migo_solver_20260718_1530.js  Exact MIGO solver (Node.js). Run
    node -e "const M=require('./migo_solver_20260718_1530.js');
    console.log(M.selfTest()); console.log(M.bookValidation());"
    to reproduce the pure-delay anchor and Tables 6.11/6.12 of Astrom
    and Hagglund (paper Table V).

Reproducing the paper with the tuner
  Case A default plant: Train reproduces the order-2 wall Ts = 3.225
    (paper Tables I, III).
  Case C, order 1: reproduces the exact MIGO PI (kp = 0.642,
    ki = 0.355 against the analytic 0.643/0.354).
  Case D, PM = 60: reproduces the direct phase-margin optimum of the
    scorecard (paper Tables III, VI).
  Evaluate external gains: paste pidtune or textbook gains to grade any
    controller in the same instrument (ZOH, chosen dt, 2 percent band).
