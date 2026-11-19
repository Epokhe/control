// ---------------------------------------------------------------------------
// Small nonlinear model-predictive-control engine (iterative LQR).
//
// Problem:   minimise   sum_k 1/2 |r(x_k, u_k)|^2  +  1/2 |rT(x_N)|^2
//            subject to x_{k+1} = F(x_k, u_k),  uMin <= u <= uMax
// where F is one RK4 step of the continuous model f over the control period.
// Costs are given as residual vectors, so the Gauss-Newton approximation of
// the Hessian is always positive semi-definite. Jacobians come from finite
// differences, so a level only has to supply its equations of motion.
//
// Each control period the controller shifts the previous plan by one step,
// re-optimises it from the measured state and applies the first input. This
// is feedback: a disturbance changes the measured state and with it the plan.
// ---------------------------------------------------------------------------
function makeMPC(spec)
{
  var n = spec.n, m = spec.m, Nmax = spec.N, N = spec.N;
  var sub = spec.sub || 1;              // control periods per plan step (input held constant over them)
  var hs = spec.h || 0.02, h = hs * sub; // integration step and plan step
  var f = spec.f, uMin = spec.uMin, uMax = spec.uMax;
  var nr = spec.nr, nrT = spec.nrT, res = spec.residual, resT = spec.terminalResidual;
  var i, j, k, a, b;

  function vec(len) { var v = new Array(len); for (var q = 0; q < len; q++) v[q] = 0; return v; }
  function mat(r, c) { var M = new Array(r); for (var q = 0; q < r; q++) M[q] = vec(c); return M; }

  var X = [], U = [], Xn = [], Un = [], A = [], B = [], kff = [], Kfb = [];
  for (k = 0; k <= Nmax; k++) { X.push(vec(n)); Xn.push(vec(n)); }
  for (k = 0; k < Nmax; k++) { U.push(vec(m)); Un.push(vec(m)); A.push(mat(n, n)); B.push(mat(n, m)); kff.push(vec(m)); Kfb.push(mat(m, n)); }

  // One plan step: `sub` RK4 steps with the input held.
  var k1 = vec(n), k2 = vec(n), k3 = vec(n), k4 = vec(n), xt = vec(n), xs = vec(n);
  function step(x, u, out)
  {
    var q, i2;
    for (q = 0; q < n; q++) xs[q] = x[q];
    for (i2 = 0; i2 < sub; i2++) {
      f(xs, u, k1);
      for (q = 0; q < n; q++) xt[q] = xs[q] + 0.5 * hs * k1[q];
      f(xt, u, k2);
      for (q = 0; q < n; q++) xt[q] = xs[q] + 0.5 * hs * k2[q];
      f(xt, u, k3);
      for (q = 0; q < n; q++) xt[q] = xs[q] + hs * k3[q];
      f(xt, u, k4);
      for (q = 0; q < n; q++) xs[q] += hs / 6 * (k1[q] + 2 * k2[q] + 2 * k3[q] + k4[q]);
    }
    for (q = 0; q < n; q++) out[q] = xs[q];
  }

  var rbuf = vec(Math.max(nr, nrT)), rp = vec(Math.max(nr, nrT)), rm = vec(Math.max(nr, nrT));
  function stageCost(x, u, kk) { res(x, u, rbuf, kk); var c = 0; for (var q = 0; q < nr; q++) c += rbuf[q] * rbuf[q]; return 0.5 * c; }
  // The terminal residual is a constraint c(x_N) = 0 handled by an augmented
  // Lagrangian: cost 1/2 |sqrt(rho) c + lambda / sqrt(rho)|^2. Between
  // optimiser runs the multipliers lambda are updated and the penalty rho is
  // raised, so the search starts soft and ends exact without ever needing one
  // huge, badly conditioned penalty weight.
  var rho0 = spec.rho || 1, rhoGrow = spec.rhoGrow || 1, rhoMax = spec.rhoMax || rho0;
  var rho = rho0, sr = Math.sqrt(rho), lambda = vec(nrT);
  function resAL(x, out) { resT(x, out); for (var q = 0; q < nrT; q++) out[q] = sr * out[q] + lambda[q] / sr; }
  function termCost(x) { resAL(x, rbuf); var c = 0; for (var q = 0; q < nrT; q++) c += rbuf[q] * rbuf[q]; return 0.5 * c; }
  function updateMultipliers()
  {
    resT(X[N], rbuf);
    for (var q = 0; q < nrT; q++) lambda[q] += rho * rbuf[q];
    rho = Math.min(rhoMax, rho * rhoGrow); sr = Math.sqrt(rho);
  }
  function resetMultipliers() { for (var q = 0; q < nrT; q++) lambda[q] = 0; rho = rho0; sr = Math.sqrt(rho); }

  function rollout(x0, Us, Xs)
  {
    var c = 0, q;
    for (q = 0; q < n; q++) Xs[0][q] = x0[q];
    for (var kk = 0; kk < N; kk++) { c += stageCost(Xs[kk], Us[kk], kk); step(Xs[kk], Us[kk], Xs[kk + 1]); }
    return c + termCost(Xs[N]);
  }

  // Jacobians of one control period at (x, u): differentiate the continuous
  // model by central differences, then discretise with a third-order series of
  // the matrix exponential (plenty for search directions; the line search
  // always evaluates the true cost).
  var xp = vec(n), up = vec(m), fp = vec(n), fm = vec(n), Ac = mat(n, n), Bc = mat(n, m), A2 = mat(n, n), T1 = mat(n, n);
  function jacobians(x, u, Ad, Bd)
  {
    var q, w, e, s, v;
    for (q = 0; q < n; q++) xp[q] = x[q];
    for (q = 0; q < m; q++) up[q] = u[q];
    for (j = 0; j < n; j++) {
      e = 1e-6 * (1 + Math.abs(xp[j]));
      xp[j] += e; f(xp, up, fp); xp[j] -= 2 * e; f(xp, up, fm); xp[j] += e;
      for (q = 0; q < n; q++) Ac[q][j] = (fp[q] - fm[q]) / (2 * e);
    }
    for (j = 0; j < m; j++) {
      e = 1e-6 * (1 + Math.abs(up[j]));
      up[j] += e; f(xp, up, fp); up[j] -= 2 * e; f(xp, up, fm); up[j] += e;
      for (q = 0; q < n; q++) Bc[q][j] = (fp[q] - fm[q]) / (2 * e);
    }
    for (q = 0; q < n; q++) for (w = 0; w < n; w++) { s = 0; for (v = 0; v < n; v++) s += Ac[q][v] * Ac[v][w]; A2[q][w] = s; }
    // T1 = h I + h^2/2 A + h^3/6 A^2  (the integral of exp(A t) over one period)
    for (q = 0; q < n; q++) for (w = 0; w < n; w++) T1[q][w] = (q === w ? h : 0) + h * h / 2 * Ac[q][w] + h * h * h / 6 * A2[q][w];
    for (q = 0; q < n; q++) {
      for (w = 0; w < n; w++) { s = 0; for (v = 0; v < n; v++) s += T1[q][v] * Ac[v][w]; Ad[q][w] = (q === w ? 1 : 0) + s; }
      for (w = 0; w < m; w++) { s = 0; for (v = 0; v < n; v++) s += T1[q][v] * Bc[v][w]; Bd[q][w] = s; }
    }
  }
  function linearise() { for (var kk = 0; kk < N; kk++) jacobians(X[kk], U[kk], A[kk], B[kk]); }

  // Gradient and Gauss-Newton Hessian of 1/2|r(z)|^2 with z = [x; u].
  var nz = n + m, nrMax = Math.max(nr, nrT), J = mat(nrMax, nz), zx = vec(n), zu = vec(m), r0 = vec(nrMax);
  var g = vec(nz), H = mat(nz, nz);
  function costDerivs(x, u, kk, terminal)
  {
    var q, e, cnt = terminal ? nrT : nr, dim = terminal ? n : nz, s;
    for (q = 0; q < n; q++) zx[q] = x[q];
    if (!terminal) for (q = 0; q < m; q++) zu[q] = u[q];
    if (terminal) resAL(zx, r0); else res(zx, zu, r0, kk);
    for (j = 0; j < dim; j++) {
      var arr = j < n ? zx : zu, jj = j < n ? j : j - n;
      e = 1e-6 * (1 + Math.abs(arr[jj]));
      arr[jj] += e; if (terminal) resAL(zx, rp); else res(zx, zu, rp, kk);
      arr[jj] -= 2 * e; if (terminal) resAL(zx, rm); else res(zx, zu, rm, kk);
      arr[jj] += e;
      for (q = 0; q < cnt; q++) J[q][j] = (rp[q] - rm[q]) / (2 * e);
    }
    for (a = 0; a < dim; a++) {
      s = 0; for (q = 0; q < cnt; q++) s += J[q][a] * r0[q]; g[a] = s;
      for (b = a; b < dim; b++) { s = 0; for (q = 0; q < cnt; q++) s += J[q][a] * J[q][b]; H[a][b] = s; H[b][a] = s; }
    }
  }

  var Vx = vec(n), Vxx = mat(n, n), Qx = vec(n), Qu = vec(m), Qxx = mat(n, n), Qux = mat(m, n), Quu = mat(m, m), Qux0 = mat(m, n), Quu0 = mat(m, m);
  var VA = mat(n, n), VB = mat(n, m), free = vec(m), Mi = mat(m, m);

  // Solve the free (unclamped) part of Quu * y = rhs for a 1- or 2-input system.
  function invFree()
  {
    var nf = 0, q, w;
    for (q = 0; q < m; q++) for (w = 0; w < m; w++) Mi[q][w] = 0;
    for (q = 0; q < m; q++) if (free[q]) nf++;
    if (nf === 0) return true;
    if (m === 1 || nf === 1) { for (q = 0; q < m; q++) if (free[q]) { if (Quu[q][q] <= 1e-12) return false; Mi[q][q] = 1 / Quu[q][q]; } return true; }
    var det = Quu[0][0] * Quu[1][1] - Quu[0][1] * Quu[1][0];
    if (Quu[0][0] <= 1e-12 || det <= 1e-12 * Quu[0][0] * Quu[1][1]) return false;
    Mi[0][0] = Quu[1][1] / det; Mi[1][1] = Quu[0][0] / det; Mi[0][1] = -Quu[0][1] / det; Mi[1][0] = -Quu[1][0] / det;
    return true;
  }

  function backward(mu)
  {
    var q, w, s, kk, dV = 0;
    costDerivs(X[N], null, N, true);
    for (a = 0; a < n; a++) { Vx[a] = g[a]; for (b = 0; b < n; b++) Vxx[a][b] = H[a][b]; }
    for (kk = N - 1; kk >= 0; kk--) {
      var Ak = A[kk], Bk = B[kk];
      costDerivs(X[kk], U[kk], kk, false);
      for (a = 0; a < n; a++) {
        for (b = 0; b < n; b++) { s = 0; for (q = 0; q < n; q++) s += Vxx[a][q] * Ak[q][b]; VA[a][b] = s; }
        for (b = 0; b < m; b++) { s = 0; for (q = 0; q < n; q++) s += Vxx[a][q] * Bk[q][b]; VB[a][b] = s; }
      }
      for (a = 0; a < n; a++) {
        s = g[a]; for (q = 0; q < n; q++) s += Ak[q][a] * Vx[q]; Qx[a] = s;
        for (b = 0; b < n; b++) { s = H[a][b]; for (q = 0; q < n; q++) s += Ak[q][a] * VA[q][b]; Qxx[a][b] = s; }
      }
      // The regularisation mu acts on the state: the gains are computed with
      // Vxx + mu I, which keeps the new trajectory close to the old one (the
      // right notion of a small step for a system this sensitive). The value
      // function itself is updated with the unregularised terms Qux0, Quu0.
      for (a = 0; a < m; a++) {
        s = g[n + a]; for (q = 0; q < n; q++) s += Bk[q][a] * Vx[q]; Qu[a] = s;
        for (b = 0; b < n; b++) {
          s = H[n + a][b]; w = 0; for (q = 0; q < n; q++) { s += Bk[q][a] * VA[q][b]; w += Bk[q][a] * Ak[q][b]; }
          Qux0[a][b] = s; Qux[a][b] = s + mu * w;
        }
        for (b = 0; b < m; b++) {
          s = H[n + a][n + b]; w = 0; for (q = 0; q < n; q++) { s += Bk[q][a] * VB[q][b]; w += Bk[q][a] * Bk[q][b]; }
          Quu0[a][b] = s; Quu[a][b] = s + mu * w + (a === b ? 1e-9 : 0);
        }
      }
      // Inputs sitting on a limit that the gradient pushes further out stay there.
      for (a = 0; a < m; a++) free[a] = !((U[kk][a] <= uMin[a] + 1e-9 && Qu[a] > 0) || (U[kk][a] >= uMax[a] - 1e-9 && Qu[a] < 0));
      if (!invFree()) return null;
      for (a = 0; a < m; a++) {
        s = 0; for (q = 0; q < m; q++) s -= Mi[a][q] * Qu[q]; kff[kk][a] = s;
        for (b = 0; b < n; b++) { s = 0; for (q = 0; q < m; q++) s -= Mi[a][q] * Qux[q][b]; Kfb[kk][a][b] = s; }
      }
      var kv = kff[kk], Kk = Kfb[kk];
      for (a = 0; a < m; a++) { s = 0; for (q = 0; q < m; q++) s += Quu0[a][q] * kv[q]; dV += kv[a] * Qu[a] + 0.5 * kv[a] * s; }
      for (a = 0; a < n; a++) {
        s = Qx[a];
        for (q = 0; q < m; q++) { s += Kk[q][a] * Qu[q] + Qux0[q][a] * kv[q]; for (w = 0; w < m; w++) s += Kk[q][a] * Quu0[q][w] * kv[w]; }
        Vx[a] = s;
      }
      for (a = 0; a < n; a++) for (b = 0; b < n; b++) {
        s = Qxx[a][b];
        for (q = 0; q < m; q++) { s += Kk[q][a] * Qux0[q][b] + Qux0[q][a] * Kk[q][b]; for (w = 0; w < m; w++) s += Kk[q][a] * Quu0[q][w] * Kk[w][b]; }
        VA[a][b] = s;
      }
      for (a = 0; a < n; a++) for (b = 0; b < n; b++) Vxx[a][b] = 0.5 * (VA[a][b] + VA[b][a]);
    }
    return dV;
  }

  function forward(x0, alpha)
  {
    var c = 0, q, w, s;
    for (q = 0; q < n; q++) Xn[0][q] = x0[q];
    for (var kk = 0; kk < N; kk++) {
      for (q = 0; q < m; q++) {
        s = U[kk][q] + alpha * kff[kk][q];
        for (w = 0; w < n; w++) s += Kfb[kk][q][w] * (Xn[kk][w] - X[kk][w]);
        Un[kk][q] = Math.max(uMin[q], Math.min(uMax[q], s));
      }
      c += stageCost(Xn[kk], Un[kk], kk);
      step(Xn[kk], Un[kk], Xn[kk + 1]);
    }
    return c + termCost(Xn[N]);
  }

  var mu = 1e-6, cost = Infinity, started = false;

  // Improve the current plan from state x0: `outer` multiplier updates, each
  // after up to maxIter iLQR iterations.
  // Stops early once done(finalState) is satisfied, if such a test is given.
  function optimise(x0, maxIter, outer, done)
  {
    for (var o = 0; o < (outer || 1); o++) {
      if (o > 0) updateMultipliers();
      inner(x0, maxIter);
      if (done && done(X[N])) break;
    }
    return cost;
  }
  function inner(x0, maxIter)
  {
    var it, q, tmp, fresh = false, stalled = 0;
    mu = Math.min(mu, 1);
    cost = rollout(x0, U, X);
    for (it = 0; it < maxIter; it++) {
      if (!fresh) { linearise(); fresh = true; }
      var dV = backward(mu);
      while (dV === null && mu < 1e8) { mu = Math.max(mu * 10, 1e-6); dV = backward(mu); }
      if (dV === null) break;
      var accepted = false, alpha = 1;
      for (q = 0; q < 6; q++) {
        var c = forward(x0, alpha);
        if (c < cost) {
          accepted = true; fresh = false;
          tmp = X; X = Xn; Xn = tmp; tmp = U; U = Un; Un = tmp;
          var gain = cost - c; cost = c; mu = Math.max(mu / (q === 0 ? 4 : 1.5), 1e-8);
          stalled = gain < 1e-6 * (1 + cost) ? stalled + 1 : 0;
          if (stalled >= 3) return cost;
          break;
        }
        alpha *= 0.4;
      }
      if (!accepted) { mu = Math.max(mu * 8, 1e-6); if (mu > 1e8) break; }
    }
    return cost;
  }

  // Shift the plan one period forward. With keepLength the horizon recedes
  // (the new last input repeats the old one); without it the horizon shrinks,
  // i.e. the planned arrival time stays fixed on the clock.
  function shift(keepLength)
  {
    var first = U[0], q, kk;
    for (kk = 0; kk + 1 < N; kk++) U[kk] = U[kk + 1];
    U[N - 1] = first;
    if (keepLength) { for (q = 0; q < m; q++) first[q] = N > 1 ? U[N - 2][q] : 0; }
    else N = Math.max(1, N - 1);
  }

  // Change the horizon length, resampling the current plan in time.
  function setHorizon(newN)
  {
    newN = Math.max(1, Math.min(Nmax, newN));
    var old = [], kk, q;
    for (kk = 0; kk < N; kk++) old.push(U[kk].slice());
    for (kk = 0; kk < newN; kk++) {
      var src = old[Math.min(N - 1, Math.floor(kk * N / newN))];
      for (q = 0; q < m; q++) U[kk][q] = src[q];
    }
    N = newN;
  }

  // Jacobians at a given point, e.g. for a terminal LQR.
  function linearAt(x, u) { var Ad = mat(n, n), Bd = mat(n, m); jacobians(x, u, Ad, Bd); return {A: Ad, B: Bd}; }

  return {
    plan: function () { return U; }, states: function () { return X; }, horizon: function () { return N; },
    final: function () { return X[N]; }, cost: function () { return cost; },
    // Largest terminal constraint violation of the current plan (1 = one tolerance).
    miss: function () { resT(X[N], rbuf); var w = 0; for (var q = 0; q < nrT; q++) w = Math.max(w, Math.abs(rbuf[q])); return w; },
    linearAt: linearAt, step: step, optimise: optimise, shift: shift, updateMultipliers: updateMultipliers, resetMultipliers: resetMultipliers, setHorizon: setHorizon,
    // Freeze the current plan as a trajectory with feedback gains:
    // input(k, x) = U[k] + K[k] (x - X[k]), clipped to the limits.
    tracker: function () {
      var q, w, kk, Xs = [], Us = [], Ks = [];
      linearise(); backward(1e-6);
      for (kk = 0; kk <= N; kk++) Xs.push(X[kk].slice());
      for (kk = 0; kk < N; kk++) { Us.push(U[kk].slice()); Ks.push(Kfb[kk].map(function (row) { return row.slice(); })); }
      return {X: Xs, U: Us, input: function (k, x) {
        var u = [];
        for (q = 0; q < m; q++) {
          var s = Us[k][q];
          for (w = 0; w < n; w++) s += Ks[k][q][w] * (x[w] - Xs[k][w]);
          u.push(Math.max(uMin[q], Math.min(uMax[q], s)));
        }
        return u;
      }};
    },
    // Start from a known trajectory (states Xs[0..N], inputs Us[0..N-1]) that
    // need not begin at x0: one backward pass yields feedback gains around it,
    // and a closed-loop rollout from x0 then gives a consistent plan nearby.
    follow: function (x0, Xs, Us) {
      var q, kk;
      N = Us.length;
      for (kk = 0; kk <= N; kk++) for (q = 0; q < n; q++) X[kk][q] = Xs[kk][q];
      for (kk = 0; kk < N; kk++) for (q = 0; q < m; q++) U[kk][q] = Us[kk][q];
      linearise();
      var reg = spec.followReg || 1; while (backward(reg) === null && reg < 1e8) reg *= 10;
      forward(x0, 0);
      var tmp = X; X = Xn; Xn = tmp; tmp = U; U = Un; Un = tmp;
    },
    // Fill the plan by simulating a feedback policy from x0 (a starting guess).
    guess: function (x0, policy) {
      var q, kk, x = x0.slice(), xn = vec(n);
      for (kk = 0; kk < N; kk++) {
        var u = policy(x);
        for (q = 0; q < m; q++) U[kk][q] = Math.max(uMin[q], Math.min(uMax[q], u[q]));
        step(x, U[kk], xn); for (q = 0; q < n; q++) x[q] = xn[q];
      }
    },
    clear: function () { for (var kk = 0; kk < N; kk++) for (var q = 0; q < m; q++) U[kk][q] = 0; },
    // Append k steps of zero input to the end of the plan.
    extend: function (k) { for (var kk = N; kk < Math.min(Nmax, N + k); kk++) for (var q = 0; q < m; q++) U[kk][q] = 0; N = Math.min(Nmax, N + k); },
    savePlan: function () { var c = []; for (var kk = 0; kk < N; kk++) c.push(U[kk].slice()); return c; },
    loadPlan: function (c) { N = c.length; for (var kk = 0; kk < N; kk++) for (var q = 0; q < m; q++) U[kk][q] = c[kk][q]; }
  };
}

// Minimum-time planning: find (approximately) the shortest horizon for which
// the optimiser still reaches the goal, as judged by reached(finalState).
// opts.seeds is a list of functions (x0, attempt) that load a starting guess
// into the planner and return false when they have nothing more to offer. The
// first guess that leads to the goal is then shortened, each attempt
// warm-started from the last successful plan. opts.valid(states, N)
// may veto a plan (e.g. one that touches an obstacle).
function planMinTime(mpc, x0, reached, opts)
{
  function done(xN) { return reached(xN) && (!opts.valid || opts.valid(mpc.states(), mpc.horizon())); }
  // One attempt at the current horizon. Gives up early when the distance to
  // the goal has stopped shrinking (the usual sign of "not enough time").
  function solve()
  {
    var miss = Infinity, o;
    mpc.resetMultipliers();
    for (o = 0; o < opts.outer; o++) {
      if (o > 0) mpc.updateMultipliers();
      mpc.optimise(x0, opts.iters);
      if (done(mpc.final())) return true;
      var now = mpc.miss();
      if (o >= 4 && now > 3 && now > 0.6 * miss) return false;
      miss = now;
    }
    return false;
  }
  var ok = false, i, attempt;
  for (i = 0; i < opts.seeds.length && !ok; i++)
    for (attempt = 0; attempt < 6 && !ok; attempt++) {
      var seeded = opts.seeds[i](x0, attempt);
      if (seeded === false) break;            // this seed has nothing more to offer
      if (seeded !== 'skip') ok = solve();    // 'skip': this attempt produced no guess, try the next
    }
  if (!ok) return false;
  // Shorten: in moderate steps until an attempt fails (a plan squeezed too
  // much in one go is a poor starting guess), then bisect between that
  // failure and the shortest success.
  var best = mpc.savePlan(), tooShort = 1, factor = opts.shrink === undefined ? 0.8 : opts.shrink;
  while (factor && best.length > tooShort * (opts.resolution || 1.08)) {
    var tryN = tooShort > 1 ? Math.round((best.length + tooShort) / 2) : Math.floor(best.length * factor);
    if (tryN >= best.length || tryN <= tooShort) break;
    mpc.loadPlan(best); mpc.setHorizon(tryN);
    if (solve()) best = mpc.savePlan(); else tooShort = tryN;
  }
  // Keep a little slack over the shortest feasible time: a time-optimal plan
  // uses the whole actuator range and leaves nothing for corrections.
  mpc.loadPlan(best);
  if (opts.margin > 1) mpc.setHorizon(Math.ceil(best.length * opts.margin));
  if (!solve()) { mpc.loadPlan(best); solve(); }
  return true;
}

// A complete "go to the goal as fast as possible and stay there" controller:
//   1. plan a minimum-time trajectory from the measured state,
//   2. follow it with time-varying linear feedback around the plan (the gains
//      fall out of the optimiser's backward pass); if the state is pushed too
//      far off the plan for that, plan again from where it is now,
//   3. once the plan has run out, hold with LQR.
// If no plan is found, cfg.fallback(x) acts for a moment and planning is retried.
// cfg: {mpc, sub, reached(xFinal), strayed(x, xPlan), hold(x), canHold(x), fallback(x), plan: {...}}
function makeManeuver(cfg)
{
  var mpc = cfg.mpc, phase = 'plan', sub = cfg.sub || 1, count = 0, last = null, wait = 0, track = null;
  drive.phase = function () { return phase; };
  return drive;
  function drive(x)
  {
    if (phase === 'hold') { if (cfg.canHold(x)) return cfg.hold(x); phase = 'plan'; }
    if (phase === 'track') {
      if (count % sub !== 0) { count++; return last; }              // inside a plan step: keep the input
      var k = count / sub;
      if (k >= track.U.length) { if (cfg.canHold(x)) { phase = 'hold'; return cfg.hold(x); } phase = 'plan'; }
      else if (cfg.strayed(x, track.X[k])) phase = 'plan';
      else { count++; last = track.input(k, x); return last; }
    }
    // phase === 'plan'
    if (wait > 0) { wait--; return cfg.fallback(x); }
    if (!planMinTime(mpc, x, cfg.reached, cfg.plan)) { wait = cfg.retryAfter || 25; return cfg.fallback(x); }
    track = mpc.tracker(); phase = 'track'; count = 1;
    last = track.input(0, x);
    return last;
  }
}

// Discrete-time LQR gain by Riccati iteration: u = -K (x - xGoal). The
// iteration may be started from the solution of a similar problem (K.P of an
// earlier result) to save time.
function dlqr(Ad, Bd, Q, R, start, tol)
{
  var P = numeric.clone(start || Q), K, it;
  var At = numeric.transpose(Ad), Bt = numeric.transpose(Bd);
  for (it = 0; it < 20000; it++) {
    var BtP = numeric.dot(Bt, P);
    K = numeric.dot(numeric.inv(numeric.add(R, numeric.dot(BtP, Bd))), numeric.dot(BtP, Ad));
    var Pn = numeric.add(Q, numeric.dot(numeric.dot(At, P), numeric.sub(Ad, numeric.dot(Bd, K))));
    var diff = numeric.norminf(numeric.sub(Pn, P)) / (1 + numeric.norminf(P));
    P = Pn;
    if (diff < (tol || 1e-12)) break;
  }
  K.P = P;
  return K;
}
