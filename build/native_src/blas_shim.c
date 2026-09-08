/* Minimal double-precision BLAS/LAPACK subset, enough for SCS's aa.c.
 *
 * WHY THIS EXISTS
 * ---------------
 * SCS enables Anderson acceleration by default (ACCELERATION_LOOKBACK = 10) and
 * it is the main thing that keeps the ADMM iteration count down on badly
 * conditioned problems — which every SCvx subproblem is. But the whole of aa.c
 * is wrapped in `#ifndef USE_LAPACK`, and the fallback is not a slower path, it
 * is a NO-OP: aa_init returns NULL and aa_apply returns 0. Building without
 * LAPACK therefore silently ships SCS with its accelerator removed.
 *
 * That is what our build was doing. Measured in closed loop, scs_init is only
 * 2-6% of solve time and the ADMM sweeps are the other 94-98%, so the iteration
 * count is the ONLY thing worth attacking, and acceleration is the mechanism
 * SCS itself provides for it.
 *
 * WHY A SHIM RATHER THAN A REAL BLAS
 * ----------------------------------
 * Linking OpenBLAS would mean shipping a multi-megabyte DLL into the game
 * process for six routines. And the sizes here are tiny: the Anderson memory is
 * `mem` = 10, so every matrix operation is either a length-`dim` vector op or a
 * skinny (dim x 10) product, and the only factorisation is a 10x10 LU. None of
 * it is close to the KKT solve that dominates each ADMM iteration, so a
 * straightforward implementation costs nothing measurable.
 *
 * SCOPE: aa.c ONLY. `struct ACCEL_WORK` is defined inside aa.c and every other
 * translation unit sees AaWork as an opaque pointer (aa.h, scs.h), so aa.c can
 * be compiled with -DUSE_LAPACK while cones.c and linalg.c are not. That keeps
 * the SDP-only symbols (dsyevr_, dgesvd_, dsyrk_) out of the link entirely and
 * leaves linalg.c on its own hand-written loops. Only these six are needed.
 *
 * Fortran conventions throughout: every argument by pointer, matrices in
 * column-major order with a leading dimension, `trans` read from its first
 * character. Increments may be negative, per the BLAS standard.
 */

#include "glbopts.h"
#include "scs_blas.h"

#include <math.h>

/* Start offset for a strided vector, per the BLAS standard: a negative
 * increment walks the vector backwards from the far end. */
static blas_int vec_start(blas_int n, blas_int inc) {
  return inc > 0 ? 0 : (1 - n) * inc;
}

/* Euclidean norm, computed with running rescaling so that intermediate squares
 * cannot overflow or flush to zero — the aa_norm this feeds is compared against
 * max_weight_norm to decide whether an acceleration step is trustworthy, so a
 * spurious inf/0 here would silently disable or wrongly accept a step. */
scs_float BLAS(nrm2)(blas_int *n, scs_float *x, blas_int *incx) {
  blas_int i, nn = *n, inc = *incx, ix;
  scs_float scale = 0.0, ssq = 1.0;
  if (nn < 1 || inc == 0) {
    return 0.0;
  }
  ix = vec_start(nn, inc);
  for (i = 0; i < nn; i++, ix += inc) {
    scs_float ax = fabs(x[ix]);
    if (ax > 0.0) {
      if (scale < ax) {
        scs_float r = scale / ax;
        ssq = 1.0 + ssq * r * r;
        scale = ax;
      } else {
        scs_float r = ax / scale;
        ssq += r * r;
      }
    }
  }
  return scale * sqrt(ssq);
}

/* y := a*x + y */
void BLAS(axpy)(blas_int *n, scs_float *a, const scs_float *x, blas_int *incx,
                scs_float *y, blas_int *incy) {
  blas_int i, nn = *n, ix, iy, cx = *incx, cy = *incy;
  scs_float alpha = *a;
  if (nn <= 0 || alpha == 0.0) {
    return;
  }
  ix = vec_start(nn, cx);
  iy = vec_start(nn, cy);
  for (i = 0; i < nn; i++, ix += cx, iy += cy) {
    y[iy] += alpha * x[ix];
  }
}

/* x := a*x */
void BLAS(scal)(const blas_int *n, const scs_float *a, scs_float *x,
                const blas_int *incx) {
  blas_int i, nn = *n, inc = *incx, ix;
  scs_float alpha = *a;
  if (nn <= 0 || inc <= 0) {
    return;
  }
  ix = vec_start(nn, inc);
  for (i = 0; i < nn; i++, ix += inc) {
    x[ix] *= alpha;
  }
}

/* y := alpha*op(A)*x + beta*y,  A is m-by-n column-major with leading dim lda.
 * op(A) = A for 'N' (x has n entries, y has m), A' for 'T' (x has m, y has n). */
void BLAS(gemv)(const char *trans, const blas_int *m, const blas_int *n,
                const scs_float *alpha, const scs_float *a, const blas_int *lda,
                const scs_float *x, const blas_int *incx, const scs_float *beta,
                scs_float *y, const blas_int *incy) {
  blas_int i, j, mm = *m, nn = *n, ld = *lda, cx = *incx, cy = *incy, ix, iy;
  int notrans = (*trans == 'N' || *trans == 'n');
  blas_int leny = notrans ? mm : nn;
  scs_float al = *alpha, be = *beta;

  if (mm <= 0 || nn <= 0) {
    return;
  }

  /* Scale y first. beta == 0 must ASSIGN rather than multiply: the caller is
   * entitled to pass uninitialised y in that case, and 0 * NaN is NaN. */
  iy = vec_start(leny, cy);
  if (be == 0.0) {
    for (i = 0; i < leny; i++, iy += cy) {
      y[iy] = 0.0;
    }
  } else if (be != 1.0) {
    for (i = 0; i < leny; i++, iy += cy) {
      y[iy] *= be;
    }
  }

  if (al == 0.0) {
    return;
  }

  if (notrans) {
    ix = vec_start(nn, cx);
    for (j = 0; j < nn; j++, ix += cx) {
      scs_float xj = al * x[ix];
      if (xj != 0.0) {
        iy = vec_start(mm, cy);
        for (i = 0; i < mm; i++, iy += cy) {
          y[iy] += a[i + j * ld] * xj;
        }
      }
    }
  } else {
    iy = vec_start(nn, cy);
    for (j = 0; j < nn; j++, iy += cy) {
      scs_float sum = 0.0;
      ix = vec_start(mm, cx);
      for (i = 0; i < mm; i++, ix += cx) {
        sum += a[i + j * ld] * x[ix];
      }
      y[iy] += al * sum;
    }
  }
}

/* C := alpha*op(A)*op(B) + beta*C,  op(A) m-by-k, op(B) k-by-n, C m-by-n. */
void BLAS(gemm)(const char *transa, const char *transb, blas_int *m,
                blas_int *n, blas_int *k, scs_float *alpha, scs_float *a,
                blas_int *lda, scs_float *b, blas_int *ldb, scs_float *beta,
                scs_float *c, blas_int *ldc) {
  blas_int i, j, l, mm = *m, nn = *n, kk = *k;
  blas_int la = *lda, lb = *ldb, lc = *ldc;
  int ta = !(*transa == 'N' || *transa == 'n');
  int tb = !(*transb == 'N' || *transb == 'n');
  scs_float al = *alpha, be = *beta;

  if (mm <= 0 || nn <= 0) {
    return;
  }

  for (j = 0; j < nn; j++) {
    for (i = 0; i < mm; i++) {
      scs_float sum = 0.0;
      if (al != 0.0) {
        for (l = 0; l < kk; l++) {
          /* op(A)(i,l): A is m-by-k for 'N', k-by-m for 'T'. */
          scs_float ail = ta ? a[l + i * la] : a[i + l * la];
          /* op(B)(l,j): B is k-by-n for 'N', n-by-k for 'T'. */
          scs_float blj = tb ? b[j + l * lb] : b[l + j * lb];
          sum += ail * blj;
        }
      }
      /* As in gemv, beta == 0 assigns so uninitialised C stays clean. */
      c[i + j * lc] = be == 0.0 ? al * sum : al * sum + be * c[i + j * lc];
    }
  }
}

/* Solve A*X = B by LU with partial pivoting; A is overwritten by its factors
 * and B by the solution. Matches LAPACK dgesv: ipiv is 1-based, and info = k+1
 * reports an exactly zero pivot at step k.
 *
 * Exact-zero is the right test to match LAPACK, and it is enough here: aa.c
 * treats any nonzero info as a failed acceleration step and resets, and it
 * separately rejects the result when the solution norm exceeds
 * max_weight_norm — which is what catches a merely ill-conditioned pivot. */
void BLAS(gesv)(blas_int *np, blas_int *nrhsp, scs_float *a, blas_int *ldap,
                blas_int *ipiv, scs_float *b, blas_int *ldbp, blas_int *info) {
  blas_int n = *np, nrhs = *nrhsp, lda = *ldap, ldb = *ldbp;
  blas_int i, j, k, piv;

  *info = 0;
  if (n < 0 || nrhs < 0 || lda < (n > 1 ? n : 1) || ldb < (n > 1 ? n : 1)) {
    *info = -1;
    return;
  }

  for (k = 0; k < n; k++) {
    scs_float best = fabs(a[k + k * lda]);
    piv = k;
    for (i = k + 1; i < n; i++) {
      scs_float v = fabs(a[i + k * lda]);
      if (v > best) {
        best = v;
        piv = i;
      }
    }
    ipiv[k] = piv + 1; /* LAPACK reports 1-based pivots */

    if (a[piv + k * lda] == 0.0) {
      *info = k + 1;
      return;
    }

    /* Swap full rows of both A and B, so B carries the permutation with it and
     * the substitutions below need no separate pivot pass. */
    if (piv != k) {
      for (j = 0; j < n; j++) {
        scs_float t = a[k + j * lda];
        a[k + j * lda] = a[piv + j * lda];
        a[piv + j * lda] = t;
      }
      for (j = 0; j < nrhs; j++) {
        scs_float t = b[k + j * ldb];
        b[k + j * ldb] = b[piv + j * ldb];
        b[piv + j * ldb] = t;
      }
    }

    for (i = k + 1; i < n; i++) {
      a[i + k * lda] /= a[k + k * lda];
    }
    for (j = k + 1; j < n; j++) {
      scs_float akj = a[k + j * lda];
      if (akj != 0.0) {
        for (i = k + 1; i < n; i++) {
          a[i + j * lda] -= a[i + k * lda] * akj;
        }
      }
    }
  }

  for (j = 0; j < nrhs; j++) {
    /* forward solve L*y = P*b, L unit lower triangular */
    for (k = 0; k < n; k++) {
      scs_float bkj = b[k + j * ldb];
      if (bkj != 0.0) {
        for (i = k + 1; i < n; i++) {
          b[i + j * ldb] -= a[i + k * lda] * bkj;
        }
      }
    }
    /* back solve U*x = y */
    for (k = n - 1; k >= 0; k--) {
      scs_float bkj = b[k + j * ldb] / a[k + k * lda];
      b[k + j * ldb] = bkj;
      if (bkj != 0.0) {
        for (i = 0; i < k; i++) {
          b[i + j * ldb] -= a[i + k * lda] * bkj;
        }
      }
    }
  }
}
