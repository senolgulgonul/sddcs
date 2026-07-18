// MIGO solver for FOTD plants P(s) = e^{-sL}/(1+sT), K=1 (normalize gain first).
// Implements Astrom & Hagglund, Advanced PID Control, Sec. 6.8:
//   Eq. (6.43): lower-vertex locus  k(w) = -(c/rho) cos(phi)
//                                   ki(w) = -(w/rho)(r + c sin(phi)) + w^2 kd
//   Eq. (6.44): tangency frequency  (r + c sin phi)(rho'/rho - 1/w) - c phi' cos phi + 2 rho kd = 0
//   Algorithm 6.2: fix kd, solve (6.44), gains from (6.43), verify full-frequency
//   M-circle, increase kd to the largest value passing the check.
// Combined sensitivity circle: center -c, radius r,
//   c = (2M^2-2M+1)/(2M(M-1)),  r = (2M-1)/(2M(M-1)),  c^2 - r^2 = 1.
// Smooth-envelope case only; the full-frequency check rejects corner-regime
// solutions (two-frequency tangency), which is where the kd sweep stops.
// Validation anchor (book Sec. 7.6): P = e^{-s}, PI: k = 0.1677, ki = 0.4618.

'use strict';

function migoCircle(M) {
  return { c: (2*M*M - 2*M + 1) / (2*M*(M - 1)), r: (2*M - 1) / (2*M*(M - 1)) };
}

function fotd(T, L) {
  return {
    rho:    w => 1 / Math.sqrt(1 + w*w*T*T),
    phi:    w => -w*L - Math.atan(w*T),
    dlnrho: w => -w*T*T / (1 + w*w*T*T),
    dphi:   w => -L - T / (1 + w*w*T*T)
  };
}

function h644(pl, w, kd, cc) {
  const rho = pl.rho(w), phi = pl.phi(w);
  return (cc.r + cc.c*Math.sin(phi)) * (pl.dlnrho(w) - 1/w)
       - cc.c * pl.dphi(w) * Math.cos(phi) + 2*rho*kd;
}

function gains643(pl, w, kd, cc) {
  const rho = pl.rho(w), phi = pl.phi(w);
  return {
    k:  -cc.c * Math.cos(phi) / rho,
    ki: -(w/rho) * (cc.r + cc.c*Math.sin(phi)) + w*w*kd,
    w
  };
}

// First-phase-wrap root of (6.44) with positive gains.
function solveTangency(pl, kd, cc) {
  let prev = null;
  for (let lw = -2; lw <= 1.2; lw += 0.001) {
    const w = Math.pow(10, lw), v = h644(pl, w, kd, cc);
    if (prev && prev.v * v < 0) {
      let lo = prev.w, hi = w;
      for (let i = 0; i < 60; i++) {
        const mid = (lo + hi) / 2;
        if (h644(pl, mid, kd, cc) * h644(pl, lo, kd, cc) > 0) lo = mid; else hi = mid;
      }
      const g = gains643(pl, (lo + hi) / 2, kd, cc);
      if (g.k > 0 && g.ki > 0) return g;
    }
    prev = { w, v };
  }
  return null;
}

function fullMok(pl, k, ki, kd, cc, tol) {
  tol = tol === undefined ? 1e-4 : tol;
  for (let lw = -2.5; lw <= 2; lw += 0.001) {
    const w = Math.pow(10, lw);
    const rho = pl.rho(w), phi = pl.phi(w);
    const Ciq = kd*w - ki/w;
    const Lr = (k*Math.cos(phi) - Ciq*Math.sin(phi)) * rho;
    const Li = (k*Math.sin(phi) + Ciq*Math.cos(phi)) * rho;
    if (Math.hypot(Lr + cc.c, Li) < cc.r - tol) return false;
  }
  return true;
}

// MIGO PI: kd = 0 tangency solution.
function migoPI(T, L, M) {
  const cc = migoCircle(M || 1.4);
  const g = solveTangency(fotd(T, L), 0, cc);
  return g ? { Kp: g.k, Ki: g.ki, wstar: g.w } : null;
}

// MIGO PID, Algorithm 6.2: largest kd whose tangency solution passes the full check.
function migoPID(T, L, M, kdStep) {
  const cc = migoCircle(M || 1.4);
  const pl = fotd(T, L);
  kdStep = kdStep || 0.01;
  let best = null;
  for (let kd = 0; kd <= 2.0001; kd += kdStep) {
    const g = solveTangency(pl, kd, cc);
    if (!g) break;
    if (fullMok(pl, g.k, g.ki, kd, cc)) best = { Kp: g.k, Ki: g.ki, Kd: kd, wstar: g.w };
    else if (best) break;
  }
  return best;
}

// Self-test: book anchor P = e^{-s}.
function selfTest() {
  const a = migoPI(0, 1, 1.4);
  const ok = Math.abs(a.Kp - 0.1677) < 5e-4 && Math.abs(a.Ki - 0.4618) < 5e-4;
  return { ok, Kp: a.Kp, Ki: a.Ki, expected: { Kp: 0.1677, Ki: 0.4618 } };
}

module.exports = { migoCircle, fotd, h644, gains643, solveTangency, fullMok, migoPI, migoPID, selfTest };

// ---- Generalizations added 2026-07-18 (book-table validation) ----
// Circle choice is a specification parameter:
//   combined (c^2-r^2=1, Ch.7 test batch, e^-s anchor) via migoCircle(M)
//   pure sensitivity (Sec. 6.8 loop-shaping tables): c=1, r=1/Ms
function sensCircle(Ms){ return { c: 1, r: 1/Ms }; }

// Lag-chain plants P(s) = prod 1/(1+sT_j) (delay-free allowed).
function lagChain(Ts){
  return {
    rho:    w => Ts.reduce((p,T)=>p/Math.sqrt(1+w*w*T*T),1),
    phi:    w => Ts.reduce((p,T)=>p-Math.atan(w*T),0),
    dlnrho: w => Ts.reduce((p,T)=>p-w*T*T/(1+w*w*T*T),0),
    dphi:   w => Ts.reduce((p,T)=>p-T/(1+w*w*T*T),0)
  };
}

// Range-parametrized variants for wide-band plants.
function solveTangencyR(pl, kd, cc, lwLo, lwHi){
  let prev=null;
  for(let lw=lwLo; lw<=lwHi; lw+=0.0005){
    const w=Math.pow(10,lw), v=h644(pl,w,kd,cc);
    if(prev && prev.v*v<0){
      let lo=prev.w, hi=w;
      for(let i=0;i<70;i++){const mid=(lo+hi)/2;
        if(h644(pl,mid,kd,cc)*h644(pl,lo,kd,cc)>0)lo=mid;else hi=mid;}
      const g=gains643(pl,(lo+hi)/2,kd,cc);
      if(g.k>0&&g.ki>0)return g;
    }
    prev={w,v};
  }
  return null;
}
function fullMokR(pl,k,ki,kd,cc,lwLo,lwHi,tol){
  tol=tol===undefined?1e-4:tol;
  for(let lw=lwLo; lw<=lwHi; lw+=0.001){
    const w=Math.pow(10,lw);
    const rho=pl.rho(w), phi=pl.phi(w);
    const Ciq=kd*w-ki/w;
    const Lr=(k*Math.cos(phi)-Ciq*Math.sin(phi))*rho;
    const Li=(k*Math.sin(phi)+Ciq*Math.cos(phi))*rho;
    if(Math.hypot(Lr+cc.c,Li)<cc.r-tol)return false;
  }
  return true;
}
function migoGeneric(pl, cc, kdMax, kdStep, lwLo, lwHi, chkHi){
  const pi=solveTangencyR(pl,0,cc,lwLo,lwHi);
  let pid=null;
  for(let kd=0;kd<=kdMax+1e-12;kd+=kdStep){
    const g=solveTangencyR(pl,kd,cc,lwLo,lwHi);
    if(!g)break;
    if(fullMokR(pl,g.k,g.ki,kd,cc,lwLo,chkHi))pid={k:g.k,ki:g.ki,kd};
    else if(pid)break;
  }
  return {pi,pid};
}

// Book-table validation anchors (Advanced PID Control):
//   Table 6.11, P=(s+1)^-4, Ms=1.4 sensitivity circle:
//     PI 0.433/0.192, PID 1.305/0.758/1.705
//   Table 6.12, P=1/((s+1)(0.1s+1)(0.01s+1)(0.001s+1)), Ms=1.4:
//     PI 4.21/8.53, PID 85.5/1488/3.87
//   Sec. 7.6, P=e^-s, combined circle M=1.4: PI 0.1677/0.4618
function bookValidation(){
  const out=[];
  const s14=sensCircle(1.4);
  let r=migoGeneric(lagChain([1,1,1,1]),s14,3.0,0.005,-2,1.2,2);
  out.push(['T6.11 PI', r.pi.k, r.pi.ki, null, 0.433, 0.192, null]);
  out.push(['T6.11 PID', r.pid.k, r.pid.ki, r.pid.kd, 1.305, 0.758, 1.705]);
  r=migoGeneric(lagChain([1,0.1,0.01,0.001]),s14,6,0.01,-1,3,4);
  out.push(['T6.12 PI', r.pi.k, r.pi.ki, null, 4.21, 8.53, null]);
  out.push(['T6.12 PID', r.pid.k, r.pid.ki, r.pid.kd, 85.5, 1488, 3.87]);
  return out;
}

module.exports.sensCircle=sensCircle;
module.exports.lagChain=lagChain;
module.exports.solveTangencyR=solveTangencyR;
module.exports.fullMokR=fullMokR;
module.exports.migoGeneric=migoGeneric;
module.exports.bookValidation=bookValidation;


if (require.main === module) {
  const st = selfTest();
  console.log('self-test (e^-s PI vs book):', st.ok ? 'PASS' : 'FAIL',
    'k=' + st.Kp.toFixed(4), 'ki=' + st.Ki.toFixed(4));
  const pi = migoPI(2, 1, 1.4);
  console.log('FOTD T/L=2 MIGO PI :', 'Kp=' + pi.Kp.toFixed(3), 'Ki=' + pi.Ki.toFixed(3));
  const pid = migoPID(2, 1, 1.4, 0.01);
  console.log('FOTD T/L=2 MIGO PID:', 'Kp=' + pid.Kp.toFixed(3), 'Ki=' + pid.Ki.toFixed(3), 'Kd=' + pid.Kd.toFixed(2));
}
