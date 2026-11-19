// Ball on platform, edge balance: balance the ball on the raised left end of
// the paddle.
//
// 1. The ball is caught on the flat of the paddle ("Catching the ball" below).
// 2. Slide. The ball slides without friction, driven only by tilting the bar.
//    A planner finds the fastest tilting motion that sends it towards the left
//    end at a moderate speed with the left end raised, and feedback around
//    the plan follows it.
// 3. Climb. The last bit is uphill and around the rounded end. The bar is
//    tilted so that the ball's speed is exactly what it takes to reach the top
//    of the end and stop there: steeper if the ball is too fast, flatter if
//    it is too slow.
// 4. Balance. On top of the end the ball is an upside-down pendulum on a
//    pivot that the hinge moves sideways. A linear regulator holds it there,
//    while the piston takes out the up and down motion of the bar's end.

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
// may veto a plan (e.g. one that touches an obstacle); opts.stiffen(more)
// is then called to raise the weight of whatever penalty should have
// prevented that (more = 0 resets it for a new attempt).
function planMinTime(mpc, x0, reached, opts)
{
  function done(xN) { return reached(xN) && (!opts.valid || opts.valid(mpc.states(), mpc.horizon())); }
  // One attempt at the current horizon. Gives up early when the distance to
  // the goal has stopped shrinking (the usual sign of "not enough time").
  function solve()
  {
    var miss = Infinity, o;
    mpc.resetMultipliers();
    if (opts.stiffen) opts.stiffen(0);
    // A few rounds extra for an attempt that has all but arrived.
    for (o = 0; o < opts.outer || (o < opts.outer + 4 && miss < 2); o++) {
      if (o > 0) mpc.updateMultipliers();
      mpc.optimise(x0, opts.iters);
      if (done(mpc.final())) return true;
      // At the goal but vetoed: weigh the keep-out penalties more heavily and go on.
      if (opts.stiffen && reached(mpc.final())) opts.stiffen(1);
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
  // If the slack version does not work out, go back to exactly the shortest plan that did.
  if (!solve()) { mpc.loadPlan(best); mpc.optimise(x0, 0); }
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

// ---------------------------------------------------------------------------
// Ball and paddle: the machine's dimensions and a model of it.
//
// The paddle is a bar on a hinge on top of a piston. The ball bounces off it
// without friction and keeps 90 % of its speed relative to the paddle.
// ---------------------------------------------------------------------------
var PADDLE = {
  g: 9.81, radius: 0.2, baseY: -5, halfWidth: 1.5, thickness: 0.15, restitution: 0.9,
  lengthMin: 1.5, lengthMax: 2.3, angleMax: 0.7, pistonAccel: 10, hingeAccel: 15,
  dt: 0.02, substeps: 5
};

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function paddleState(ball, piston, hinge)
{
  return {x: ball.x, y: ball.y, vx: ball.vx, vy: ball.vy, L: piston.length, v: piston.speed, th: hinge.angle, om: hinge.speed};
}

// Where the ball is relative to the paddle. along: position along the bar from
// its centre; gap: clearance between ball and bar; (nx, ny): direction from
// the nearest point of the bar to the ball; closing: speed at which the gap
// shrinks; (px, py), (pvx, pvy): that nearest point and its velocity.
function paddleContact(s)
{
  var P = PADDLE, C = Math.cos(s.th), S = Math.sin(s.th);
  var cx = -S * P.thickness, cy = P.baseY + s.L + C * P.thickness;
  var along = C * (s.x - cx) + S * (s.y - cy);
  var a = clamp(along, -P.halfWidth, P.halfWidth);
  var px = cx + C * a, py = cy + S * a;
  var pvx = -C * s.om * P.thickness - S * s.om * a, pvy = s.v - S * s.om * P.thickness + C * s.om * a;
  var nx = s.x - px, ny = s.y - py, d = Math.sqrt(nx * nx + ny * ny);
  nx /= d; ny /= d;
  return {along: along, gap: d - P.radius, nx: nx, ny: ny, px: px, py: py, pvx: pvx, pvy: pvy,
          closing: -(nx * (s.vx - pvx) + ny * (s.vy - pvy))};
}

// Advance the model by one control period with constant accelerations, in the
// same small steps as the machine. Returns true if the ball bounced.
function paddleAdvance(s, pistonAccel, hingeAccel)
{
  var P = PADDLE, h = P.dt / P.substeps, bounced = false;
  pistonAccel = clamp(pistonAccel, -P.pistonAccel, P.pistonAccel);
  hingeAccel = clamp(hingeAccel, -P.hingeAccel, P.hingeAccel);
  for (var i = 0; i < P.substeps; i++) {
    s.x += h * s.vx;
    s.y += h * s.vy - 0.5 * h * h * P.g;
    s.vy -= h * P.g;
    if (s.y - P.radius < P.baseY) s.vy = Math.abs(s.vy);
    s.om += h * hingeAccel; s.th += h * s.om;
    s.v += h * pistonAccel; s.L += h * s.v;
    if (s.L > P.lengthMax) { s.L = P.lengthMax; s.v = Math.min(0, s.v); }
    if (s.L < P.lengthMin) { s.L = P.lengthMin; s.v = Math.max(0, s.v); }
    if (s.th > P.angleMax) { s.th = P.angleMax; s.om = Math.min(0, s.om); }
    if (s.th < -P.angleMax) { s.th = -P.angleMax; s.om = Math.max(0, s.om); }
    var c = paddleContact(s);
    if (c.closing > 0 && c.gap < 0) {
      var kick = (1 + P.restitution) * c.closing;
      s.vx += c.nx * kick; s.vy += c.ny * kick;
      s.x = c.px + c.nx * P.radius; s.y = c.py + c.ny * P.radius;
      bounced = true;
    }
  }
  return bounced;
}

// Time until a ball in free flight comes down to height y.
function fallTime(s, y)
{
  var disc = s.vy * s.vy + 2 * PADDLE.g * (s.y - y);
  return disc > 0 ? (s.vy + Math.sqrt(disc)) / PADDLE.g : 0;
}

// Height of the ball's centre when it rests on a level paddle.
function restHeight(length) { return PADDLE.baseY + length + PADDLE.thickness + PADDLE.radius; }

// ---------------------------------------------------------------------------
// Aimed bounces.
//
// Before a bounce the controller picks what the paddle should be doing at the
// moment of contact (piston length, speed and acceleration, and the tilt),
// drives piston and hinge there along smooth curves, and checks the aim by
// running the model forward to the bounce. Newton's method corrects speed and
// tilt until the predicted outcome is the wanted one. This is repeated every
// control period, so errors are corrected for as long as the ball is in the
// air.
// ---------------------------------------------------------------------------

// Shortly before contact the plan is no longer changed.
var AIM_COMMIT = 3.5 * PADDLE.dt;

// Piston and hinge accelerations over time that reach the goal {length, speed,
// accel, angle} (hinge at rest) after time tau, starting from state s.
function aimCurves(s, goal, tau)
{
  tau = Math.max(tau, PADDLE.dt);
  var D = goal.length - s.L - s.v * tau, E = goal.speed - s.v;
  var a0 = (12 * D - 6 * E * tau + goal.accel * tau * tau) / (tau * tau);
  var snap = (6 * (a0 + goal.accel) * tau - 12 * E) / (tau * tau * tau);
  var jerk = (goal.accel - a0 - 0.5 * snap * tau * tau) / tau;
  var Dh = goal.angle - s.th - s.om * tau, Eh = -s.om;
  var h0 = (6 * Dh - 2 * Eh * tau) / (tau * tau), h1 = 2 * (Eh - h0 * tau) / (tau * tau);
  return function (t) {
    if (t >= tau) return [goal.accel, 0];
    return [a0 + jerk * t + 0.5 * snap * t * t, h0 + h1 * t];
  };
}

// Whether those curves stay within what piston and hinge can do.
function aimFeasible(s, goal, tau, margin)
{
  var c = aimCurves(s, goal, tau), L = s.L, v = s.v, dt = PADDLE.dt;
  for (var t = 0; t < tau; t += dt) {
    var u = c(Math.min(t + 0.5 * dt, tau - 1e-9));
    if (Math.abs(u[0]) > PADDLE.pistonAccel * margin || Math.abs(u[1]) > PADDLE.hingeAccel * margin) return false;
    v += u[0] * dt; L += v * dt;
    if (L < PADDLE.lengthMin || L > PADDLE.lengthMax) return false;
  }
  return true;
}

// Run the model forward until the ball bounces. The curves are redrawn from
// the current state every period, as the controller itself does, until
// shortly before contact. Returns the state just after the bounce.
function aimShoot(s0, goal, tau)
{
  var s = {}, k, c, first, since = 0; for (k in s0) s[k] = s0[k];
  for (var i = 0; i < 150; i++) {
    var left = tau - i * PADDLE.dt;
    if (i === 0 || left > AIM_COMMIT) { c = aimCurves(s, goal, left); since = 0; if (i === 0) first = c; }
    var u = c((since + 0.5) * PADDLE.dt);
    since++;
    if (paddleAdvance(s, u[0], u[1])) return {s: s, curve: first, time: (i + 1) * PADDLE.dt};
  }
  return null;
}

// Rough paddle speed and tilt that send a ball, arriving at (vx, vIn), straight
// up to the height apogee from a paddle at the given piston length.
function roughAim(vx, vIn, length, apogee)
{
  var vOut = Math.sqrt(Math.max(0, 2 * PADDLE.g * (apogee - restHeight(length))));
  return {speed: (vOut + PADDLE.restitution * vIn) / (1 + PADDLE.restitution), angle: Math.atan(vx / (vOut - vIn))};
}

// One aimer plans one bounce at a time. Call step every control period with
// the current state and the target:
//   {length, accel: piston length and acceleration at contact,
//    guess(tau): first guess {speed, angle},
//    miss(after): two numbers that are zero when the bounce is as wanted,
//    fixed: true to use the guess as it is}
function makeAimer()
{
  var aim = null, plan = null, left = null;
  function goalOf(target) { return {length: target.length, accel: target.accel, speed: aim.speed, angle: aim.angle}; }
  return {
    reset: function () { aim = null; plan = null; left = null; },
    step: function (s, target) {
      var dt = PADDLE.dt, u;
      if (plan && plan.left <= AIM_COMMIT) {
        plan.step++; plan.left -= dt;
        u = plan.curve((plan.step + 0.5) * dt);
        return {pistonAcceleration: u[0], hingeAcceleration: u[1]};
      }
      // Time to contact: counted down from the first estimate, which is
      // corrected by what the model says.
      var tau = left === null ? fallTime(s, restHeight(target.length)) : left - dt, fresh = left === null ? 2 : 0;
      if (!aim || target.fixed) aim = target.guess(tau);
      var out = null;
      for (var iter = 0; iter < 8; iter++) {
        out = aimShoot(s, goalOf(target), tau);
        if (!out) break;
        if (fresh > 0 && Math.abs(out.time - tau) > 0.5 * dt) { tau = out.time; fresh--; continue; }
        if (target.fixed) break;
        var f = target.miss(out.s);
        if (Math.abs(f[0]) + Math.abs(f[1]) < 1e-10) break;
        var eps = 1e-5, g = goalOf(target);
        g.speed += eps; var a = aimShoot(s, g, tau);
        g.speed -= eps; g.angle += eps; var b = aimShoot(s, g, tau);
        if (!a || !b) break;
        var fa = target.miss(a.s), fb = target.miss(b.s);
        var j00 = (fa[0] - f[0]) / eps, j10 = (fa[1] - f[1]) / eps, j01 = (fb[0] - f[0]) / eps, j11 = (fb[1] - f[1]) / eps;
        var det = j00 * j11 - j01 * j10;
        if (Math.abs(det) < 1e-12) break;
        aim.speed -= clamp((j11 * f[0] - j01 * f[1]) / det, -1, 1);
        aim.angle = clamp(aim.angle - clamp((-j10 * f[0] + j00 * f[1]) / det, -0.2, 0.2), -0.6, 0.6);
      }
      if (!out) return null;
      left = tau;
      plan = {curve: out.curve, step: 0, left: tau};
      u = out.curve(0.5 * dt);
      return {pistonAcceleration: u[0], hingeAcceleration: u[1]};
    }
  };
}

// ---------------------------------------------------------------------------
// Catching the ball.
//
// A ball cannot simply be caught: the piston is hardly quicker than gravity,
// so it cannot match the speed of a fast ball, and every bounce keeps 90 % of
// the speed between ball and paddle. The catcher therefore plays three aimed
// bounces (see "Aimed bounces"):
//   hop    a low bounce that brings the ball down over the wanted spot,
//   pop    a bounce there with a tilted paddle that removes the sideways
//          speed and leaves just enough height to level the paddle again,
//   drop   a bounce off a paddle moving away that takes out most of the rest.
// What is left is a small bounce: the paddle chases the ball upwards and then
// falls with it, so that they meet gently, and finally comes to a stop with
// the ball lying on top. Which step comes next is decided from where the ball
// is heading, so the same rules deal with any start.
//
// makeCatcher(P) returns step(ball, piston, hinge) -> {pistonAcceleration,
// hingeAcceleration, resting}. P.x is where the ball should come to rest.
// step.settle is the piston rule for a ball that is on the paddle.
// ---------------------------------------------------------------------------
var CATCH_DEFAULTS = {
  x: 0,
  hop: 0.5, pop: 0.1,            // heights of the first two aimed bounces above the paddle (the hop at least)
  popTilt: 0.32,                 // largest tilt planned for the pop
  dropSpeed: 0.4,                // upward speed left to the ball by the third
  hopLength: 1.75, popLength: 1.9, dropLength: 1.8,   // piston lengths to bounce at
  small: 0.06,                   // bounces lower than this are not aimed
  near: 0.05,                    // a ball coming down this close to the spot needs no hop
  slow: 0.05,                    // sideways speed below which a bounce with a level paddle will do
  touch: 0.05,                   // closing speed kept until the ball touches
  brake: 8,                      // piston deceleration planned for stopping
  carry: 6,                      // strongest downward piston acceleration with the ball on top
  restGap: 0.004, restSpeed: 0.3,   // the ball counts as lying on the paddle below these
  chase: 0.4,                    // closing speed on small bounces, as a share of free-fall speed over the gap
  slide: 2, soft: 0.5, lead: 0.2, tiltMax: 0.3,       // sliding the resting ball to the spot
  hinge: 12, hingeSoft: 0.8
};

// Speed to approach a target with: brakes at a constant rate, eases off close by.
function approach(err, decel, soft) { return -Math.sign(err) * (Math.sqrt(soft * soft + 2 * decel * Math.abs(err)) - soft); }

function makeCatcher(options)
{
  var P = {}, key;
  for (key in CATCH_DEFAULTS) P[key] = options && key in options ? options[key] : CATCH_DEFAULTS[key];
  var aimer = makeAimer(), mode = null, last = null;
  var CONTACT_ACCEL = -PADDLE.restitution / (1 + PADDLE.restitution) * PADDLE.g;


  function riseOf(after) { return after.vy * after.vy / (2 * PADDLE.g); }

  // The ball's state at its next bounce, if that is aimed at the given target.
  function nextBounce(s, t)
  {
    var tau = fallTime(s, restHeight(t.length)), rough = t.guess(tau);
    return aimShoot(s, {length: t.length, accel: t.accel, speed: rough.speed, angle: rough.angle}, tau);
  }

  // Hop: a low bounce that comes down over the wanted spot. The further the
  // ball has to travel, the higher the hop, so that the tilt needed to stop
  // its sideways motion at the next bounce stays moderate.
  function hopTarget(s)
  {
    var g = PADDLE.g, fall = fallTime(s, restHeight(P.hopLength)), far = Math.abs(s.x + s.vx * fall - P.x);
    var height = P.hop, popSpeed = Math.sqrt(2 * g * P.pop);
    while (height < 2 && far / (2 * Math.sqrt(2 * height / g)) > Math.tan(P.popTilt) * (popSpeed + Math.sqrt(2 * g * height))) height += 0.05;
    return {
      length: P.hopLength, accel: CONTACT_ACCEL,
      guess: function (tau) { return {speed: roughAim(0, s.vy - g * tau, P.hopLength, restHeight(P.hopLength) + height).speed, angle: 0}; },
      miss: function (after) {
        var landing = nextBounce(after, popTarget(after));
        return [riseOf(after) - height, (landing ? landing.s.x : after.x) - P.x];
      }
    };
  }

  // Pop: take out the sideways speed, but for the little that carries the ball
  // exactly to the centre by the next bounce.
  function popTarget(s)
  {
    return {
      length: P.popLength, accel: CONTACT_ACCEL,
      guess: function (tau) { return roughAim(s.vx, s.vy - PADDLE.g * tau, P.popLength, restHeight(P.popLength) + P.pop); },
      miss: function (after) {
        var landing = nextBounce(after, dropTarget(after));
        return [riseOf(after) - P.pop, (landing ? landing.s.x : after.x) - P.x];
      }
    };
  }

  // Drop: with the paddle level again, take nearly all the speed out of the ball.
  function dropTarget(s)
  {
    return {
      length: P.dropLength, accel: CONTACT_ACCEL,
      guess: function (tau) {
        var vIn = s.vy - PADDLE.g * tau;
        return {speed: (P.dropSpeed + PADDLE.restitution * vIn) / (1 + PADDLE.restitution), angle: Math.atan(s.vx / (P.dropSpeed - vIn))};
      },
      miss: function (after) { return [after.vy - P.dropSpeed, after.vx]; }
    };
  }

  function choose(s)
  {
    var landing = nextBounce(s, popTarget(s));
    if (!landing || Math.abs(landing.s.x - P.x) > P.near) return {name: 'hop', target: hopTarget(s)};
    return Math.abs(s.vx) < P.slow ? {name: 'drop', target: dropTarget(s)} : {name: 'pop', target: popTarget(s)};
  }

  // Piston rule for a ball lying on the paddle or bouncing a little.
  // Lying: move at the speed wanted (zero to stop), without throwing the ball
  // off. Bouncing: move with the ball, closing in slowly, so that it lands softly.
  function settle(ball, piston, c, wanted)
  {
    var M = PADDLE, gap = Math.max(0, c.gap), lowest = -M.pistonAccel, want = wanted;
    if (gap < P.restGap && Math.abs(c.closing) < P.restSpeed) lowest = -P.carry;
    else want = ball.vy - (c.pvy - piston.speed) - M.g * M.dt + P.touch + P.chase * Math.sqrt(2 * (M.g + M.pistonAccel) * gap);
    var up = Math.sqrt(2 * P.brake * Math.max(0, M.lengthMax - 0.01 - piston.length));
    var down = -Math.sqrt(2 * P.brake * Math.max(0, piston.length - M.lengthMin - 0.01));
    return clamp((clamp(want, down, up) - piston.speed) / M.dt, lowest, M.pistonAccel);
  }

  function step(ball, piston, hinge)
  {
    var M = PADDLE, s = paddleState(ball, piston, hinge), c = paddleContact(s);
    if (last !== null && Math.abs(ball.vy - (last - M.g * M.dt)) > 1e-6) { mode = null; aimer.reset(); }
    last = ball.vy;
    var gap = Math.max(0, c.gap), resting = gap < P.restGap && Math.abs(c.closing) < P.restSpeed;

    if (!resting) {
      // Bounces too small to aim are followed with the simple rules further down.
      if (!mode) mode = s.y + s.vy * s.vy / (2 * M.g) - (c.py + M.radius) < P.small ? {name: 'small'} : choose(s);
      if (mode.name !== 'small') {
        var u = aimer.step(s, mode.target);
        if (u) { u.resting = false; return u; }
      }
    }

    var pistonAcceleration = settle(ball, piston, c, 0);

    // Hinge. The ball only changes its sideways speed when it presses on a
    // tilted paddle: in one go when it bounces, steadily when it rests.
    var vxWant = approach(ball.x - P.x, P.slide, P.soft);
    var impact = Math.sqrt(Math.max(0, c.closing) * Math.max(0, c.closing) + 2 * M.g * gap);
    var push = Math.max((1 + M.restitution) * impact, M.g * P.lead);
    var tilt = clamp((ball.vx - vxWant) / push, -P.tiltMax, P.tiltMax);
    var rate = approach(hinge.angle - tilt, P.hinge, P.hingeSoft);
    var hingeAcceleration = clamp((rate - hinge.speed) / M.dt, -M.hingeAccel, M.hingeAccel);
    return {pistonAcceleration: pistonAcceleration, hingeAcceleration: hingeAcceleration, resting: resting};
  }

  step.settle = settle;
  return step;
}

var CATCH_X = -0.5;        // where to catch the ball
var ANGLE = -0.4;          // paddle angle to settle at when balancing (left end raised)
var LAUNCH = [0.03, -0.5, -0.35, 0];      // end of the slide: just before the end, moving towards it, left end raised
var TOL = [0.01, 0.05, 0.03, 0.1];
var W = {q: 0.3, dq: 0.05, th: 0.5, dth: 0.05, u: 0.004, limit: 30, press: 3};
var HOLD = {top: 1000, dq: 1, th: 0.01, dth: 100, u: 0.01};   // weights of the regulator that balances the ball
var MID_LENGTH = 1.9;
var MIN_PRESS = 6;         // keep the ball pressed on with at least this acceleration
var CLIMB = {hinge: 12, hingeSoft: 0.8, lost: 0.6};

var catcher = makeCatcher({x: CATCH_X}), drive = null, sliding = false, balancing = false;

// The ball's path in the paddle's own frame (origin at the hinge), by distance
// q travelled from the left end of the flat: along the flat for q > 0, around
// the rounded end for q < 0. Returns position c and unit direction t.
function path(q, o)
{
  var M = PADDLE;
  if (q >= 0) { o.cx = q - M.halfWidth; o.cy = M.thickness + M.radius; o.tx = 1; o.ty = 0; o.curve = 0; }
  else {
    var psi = -q / M.radius, s = Math.sin(psi), c = Math.cos(psi);
    o.cx = -M.halfWidth - M.radius * s; o.cy = M.thickness + M.radius * c; o.tx = c; o.ty = s; o.curve = 1 / M.radius;
  }
  return o;
}

// Sliding ball on the tilting bar. State [q, dq, angle, rate], input: hinge
// acceleration. press: how hard the ball is pressed onto the bar.
var scratch = {};
function slide(x, u, out)
{
  var p = path(x[0], scratch), S = Math.sin(x[2]), C = Math.cos(x[2]), g = PADDLE.g;
  out[0] = x[1];
  out[1] = -g * (p.tx * S + p.ty * C) - u[0] * (-p.tx * p.cy + p.ty * p.cx) + x[3] * x[3] * (p.tx * p.cx + p.ty * p.cy);
  out[2] = x[3];
  out[3] = u[0];
}
function press(x, u)
{
  var p = path(x[0], scratch), S = Math.sin(x[2]), C = Math.cos(x[2]), nx = -p.ty, ny = p.tx;
  return u[0] * (-nx * p.cy + ny * p.cx) - x[3] * x[3] * (nx * p.cx + ny * p.cy) + 2 * x[3] * x[1] - x[1] * x[1] * p.curve + PADDLE.g * (nx * S + ny * C);
}

// Measure the sliding state from ball and paddle.
function measure(s)
{
  var M = PADDLE, S = Math.sin(s.th), C = Math.cos(s.th);
  var px = s.x, py = s.y - (M.baseY + s.L);
  var lx = C * px + S * py, ly = -S * px + C * py;                 // ball in the paddle's frame
  var vx = C * s.vx + S * (s.vy - s.v) + s.om * ly, vy = -S * s.vx + C * (s.vy - s.v) - s.om * lx;   // and its velocity there
  var q = lx + M.halfWidth;
  if (q < 0) q = -M.radius * Math.atan2(-q, ly - M.thickness);
  var p = path(q, scratch);
  return [q, p.tx * vx + p.ty * vy, s.th, s.om];
}

function hingeLimit(v, lo, hi) { return v < lo ? v - lo : v > hi ? v - hi : 0; }

// Angle of the ball's contact normal from vertical: zero on top of the end.
function lean(x) { return x[2] - Math.min(0, x[0]) / PADDLE.radius; }

function build()
{
  var M = PADDLE, h = M.dt;
  var mpc = makeMPC({
    n: 4, m: 1, N: 200, h: h, rho: 1e-4, rhoGrow: 10, rhoMax: 10, uMin: [-M.hingeAccel], uMax: [M.hingeAccel],
    f: slide,
    nr: 8,
    residual: function (x, u, r) {
      r[0] = W.q * (x[0] - LAUNCH[0]); r[1] = W.dq * x[1]; r[2] = W.th * (x[2] - LAUNCH[2]); r[3] = W.dth * x[3]; r[4] = W.u * u[0];
      r[5] = W.limit * hingeLimit(x[2], -0.65, 0.65);                       // stay clear of the hinge's end stops
      r[6] = W.limit * hingeLimit(x[0], 0.01, 2 * M.halfWidth - 0.1);       // stay on the flat of the bar
      r[7] = W.press * Math.min(0, press(x, u) - MIN_PRESS);                // and keep the ball pressed on
    },
    nrT: 4,
    terminalResidual: function (x, r) { for (var i = 0; i < 4; i++) r[i] = (x[i] - LAUNCH[i]) / TOL[i]; }
  });

  // Balancing regulators for a range of tilts. The ball can rest on top of
  // the end at any tilt; the regulator brings it to the top and to rest, and
  // steers the tilt itself back only slowly.
  var gains = [], top = [1, 0, -M.radius, 0], warm = null;
  for (var a = -0.05; a > -0.7; a -= 0.05) {
    var lin = mpc.linearAt([M.radius * a, 0, a, 0], [0]), Q = numeric.diag([0, HOLD.dq, HOLD.th, HOLD.dth]);
    for (var i = 0; i < 4; i++) for (var j = 0; j < 4; j++) Q[i][j] += HOLD.top * top[i] * top[j];
    warm = dlqr(lin.A, lin.B, Q, [[HOLD.u]], warm && warm.P, 1e-9);
    gains.push(warm[0]);
  }
  function balance(x)
  {
    var K = gains[clamp(Math.round(-x[2] / 0.05) - 1, 0, gains.length - 1)];
    return clamp(-(K[0] * (x[0] - M.radius * ANGLE) + K[1] * x[1] + K[2] * (x[2] - ANGLE) + K[3] * x[3]), -M.hingeAccel, M.hingeAccel);
  }
  // After the slide: keep the tilt while the ball runs up to the end, then balance.
  function finish(x)
  {
    balancing = x[0] < 0;
    if (balancing) return [balance(x)];
    var rate = approach(x[2] - LAUNCH[2], CLIMB.hinge, CLIMB.hingeSoft);
    return [clamp((rate - x[3]) / h, -M.hingeAccel, M.hingeAccel)];
  }

  // Slide times to try: scaled with the distance to cover, shortest first.
  var STRETCH = [1, 1.15, 1.4, 1.8];
  function seed(x0, attempt)
  {
    if (attempt >= STRETCH.length) return false;
    var time = 0.45 + 0.75 * Math.sqrt(Math.max(0.05, Math.abs(x0[0] - LAUNCH[0])));
    mpc.setHorizon(Math.round(STRETCH[attempt] * time / h));
    mpc.clear();
  }
  drive = makeManeuver({
    mpc: mpc,
    plan: {iters: 40, outer: 10, shrink: 0, seeds: [seed]},
    reached: function (x) { for (var i = 0; i < 4; i++) if (Math.abs(x[i] - LAUNCH[i]) > 0.5 * TOL[i]) return false; return true; },
    strayed: function (x, xPlan) { return Math.abs(x[0] - xPlan[0]) > 0.08 || Math.abs(x[1] - xPlan[1]) > 0.5 || Math.abs(x[2] - xPlan[2]) > 0.2; },
    // The climb is on as long as the ball is close to the end and still moving up, or on top.
    canHold: function (x) { return x[0] < 0 ? Math.abs(lean(x)) < CLIMB.lost : x[0] < 0.2 && x[1] < -0.05; },
    hold: finish,
    fallback: function (x) { return [clamp(-40 * x[3] - 100 * x[2], -M.hingeAccel, M.hingeAccel)]; }
  });
}

function controlFunction(ball, piston, hinge, T)
{
  var M = PADDLE, s = paddleState(ball, piston, hinge);
  if (!sliding) {
    var u = catcher(ball, piston, hinge);
    sliding = u.resting && Math.abs(ball.x - CATCH_X) < 0.1 && Math.abs(ball.vx) < 0.1 && Math.abs(piston.speed) < 0.05;
    if (!sliding) return {pistonAcceleration: u.pistonAcceleration, hingeAcceleration: u.hingeAcceleration};
  }
  if (!drive) build();
  var x = measure(s);
  var hingeAcceleration = drive(x)[0], pistonAcceleration;
  if (balancing && drive.phase() === 'hold') {
    // Balancing: tilting the bar moves its end sideways, which is what steers
    // the ball, but also up and down, which would make the ball hop. The
    // piston takes out the up and down part.
    var S = Math.sin(hinge.angle), C = Math.cos(hinge.angle);
    var lift = -M.thickness * S - M.halfWidth * C, liftRate = -M.thickness * C + M.halfWidth * S;   // end height per angle, and its slope
    var follow = -lift * hinge.speed;
    pistonAcceleration = -(lift * hingeAcceleration + liftRate * hinge.speed * hinge.speed) + (follow - piston.speed) / 0.1;
  } else {
    // Sliding: drift gently to mid-stroke, to have room for the above.
    pistonAcceleration = clamp((approach(piston.length - MID_LENGTH, 0.8, 0.3) - piston.speed) / 0.1, -1, 1);
  }
  return {pistonAcceleration: pistonAcceleration, hingeAcceleration: hingeAcceleration};
}
