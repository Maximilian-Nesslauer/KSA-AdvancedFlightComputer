#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "scs.h"
#include "CscMatrix.h"
#include "DefaultSettings.h"
#include "DefaultSolution.h"
#include "SupportedConeT.h"

_Static_assert(sizeof(void *) == 8, "An x64 target is required");
_Static_assert(sizeof(scs_int) == 4, "DLONG must be undefined");
_Static_assert(sizeof(scs_float) == 8, "SFLOAT must be undefined");
_Static_assert(sizeof(bool) == 1, "The Clarabel ABI requires one-byte bool");

#define SIZE(name, type) printf(#name ".size=%zu\n", sizeof(type))
#define FIELD(name, type, managed, native) printf(#name "." #managed "=%zu\n", offsetof(type, native))

int main(void) {
    SIZE(ScsMatrix, ScsMatrix);
    FIELD(ScsMatrix, ScsMatrix, X, x);
    FIELD(ScsMatrix, ScsMatrix, I, i);
    FIELD(ScsMatrix, ScsMatrix, P, p);
    FIELD(ScsMatrix, ScsMatrix, M, m);
    FIELD(ScsMatrix, ScsMatrix, N, n);
    SIZE(ScsData, ScsData);
    FIELD(ScsData, ScsData, M, m);
    FIELD(ScsData, ScsData, N, n);
    FIELD(ScsData, ScsData, A, A);
    FIELD(ScsData, ScsData, P, P);
    FIELD(ScsData, ScsData, B, b);
    FIELD(ScsData, ScsData, C, c);
    SIZE(ScsCone, ScsCone);
    FIELD(ScsCone, ScsCone, Z, z);
    FIELD(ScsCone, ScsCone, L, l);
    FIELD(ScsCone, ScsCone, Bu, bu);
    FIELD(ScsCone, ScsCone, Bl, bl);
    FIELD(ScsCone, ScsCone, Bsize, bsize);
    FIELD(ScsCone, ScsCone, Q, q);
    FIELD(ScsCone, ScsCone, Qsize, qsize);
    FIELD(ScsCone, ScsCone, S, s);
    FIELD(ScsCone, ScsCone, Ssize, ssize);
    FIELD(ScsCone, ScsCone, Cs, cs);
    FIELD(ScsCone, ScsCone, Cssize, cssize);
    FIELD(ScsCone, ScsCone, Ep, ep);
    FIELD(ScsCone, ScsCone, Ed, ed);
    FIELD(ScsCone, ScsCone, P_, p);
    FIELD(ScsCone, ScsCone, Psize, psize);
    SIZE(ScsSettings, ScsSettings);
    FIELD(ScsSettings, ScsSettings, Normalize, normalize);
    FIELD(ScsSettings, ScsSettings, Scale, scale);
    FIELD(ScsSettings, ScsSettings, AdaptiveScale, adaptive_scale);
    FIELD(ScsSettings, ScsSettings, RhoX, rho_x);
    FIELD(ScsSettings, ScsSettings, MaxIters, max_iters);
    FIELD(ScsSettings, ScsSettings, EpsAbs, eps_abs);
    FIELD(ScsSettings, ScsSettings, EpsRel, eps_rel);
    FIELD(ScsSettings, ScsSettings, EpsInfeas, eps_infeas);
    FIELD(ScsSettings, ScsSettings, Alpha, alpha);
    FIELD(ScsSettings, ScsSettings, TimeLimitSecs, time_limit_secs);
    FIELD(ScsSettings, ScsSettings, Verbose, verbose);
    FIELD(ScsSettings, ScsSettings, WarmStart, warm_start);
    FIELD(ScsSettings, ScsSettings, AccelerationLookback, acceleration_lookback);
    FIELD(ScsSettings, ScsSettings, AccelerationInterval, acceleration_interval);
    FIELD(ScsSettings, ScsSettings, WriteDataFilename, write_data_filename);
    FIELD(ScsSettings, ScsSettings, LogCsvFilename, log_csv_filename);
    SIZE(ScsSolution, ScsSolution);
    FIELD(ScsSolution, ScsSolution, X, x);
    FIELD(ScsSolution, ScsSolution, Y, y);
    FIELD(ScsSolution, ScsSolution, S, s);
    SIZE(ScsInfo, ScsInfo);
    FIELD(ScsInfo, ScsInfo, Iter, iter);
    FIELD(ScsInfo, ScsInfo, Status, status);
    FIELD(ScsInfo, ScsInfo, LinSysSolver, lin_sys_solver);
    FIELD(ScsInfo, ScsInfo, StatusVal, status_val);
    FIELD(ScsInfo, ScsInfo, ScaleUpdates, scale_updates);
    FIELD(ScsInfo, ScsInfo, Pobj, pobj);
    FIELD(ScsInfo, ScsInfo, Dobj, dobj);
    FIELD(ScsInfo, ScsInfo, ResPri, res_pri);
    FIELD(ScsInfo, ScsInfo, ResDual, res_dual);
    FIELD(ScsInfo, ScsInfo, Gap, gap);
    FIELD(ScsInfo, ScsInfo, ResInfeas, res_infeas);
    FIELD(ScsInfo, ScsInfo, ResUnbddA, res_unbdd_a);
    FIELD(ScsInfo, ScsInfo, ResUnbddP, res_unbdd_p);
    FIELD(ScsInfo, ScsInfo, SetupTime, setup_time);
    FIELD(ScsInfo, ScsInfo, SolveTime, solve_time);
    FIELD(ScsInfo, ScsInfo, Scale, scale);
    FIELD(ScsInfo, ScsInfo, CompSlack, comp_slack);
    FIELD(ScsInfo, ScsInfo, RejectedAccelSteps, rejected_accel_steps);
    FIELD(ScsInfo, ScsInfo, AcceptedAccelSteps, accepted_accel_steps);
    FIELD(ScsInfo, ScsInfo, LinSysTime, lin_sys_time);
    FIELD(ScsInfo, ScsInfo, ConeTime, cone_time);
    FIELD(ScsInfo, ScsInfo, AccelTime, accel_time);
    SIZE(ClarabelCscMatrix, ClarabelCscMatrix_f64);
    FIELD(ClarabelCscMatrix, ClarabelCscMatrix_f64, M, m);
    FIELD(ClarabelCscMatrix, ClarabelCscMatrix_f64, N, n);
    FIELD(ClarabelCscMatrix, ClarabelCscMatrix_f64, ColPtr, colptr);
    FIELD(ClarabelCscMatrix, ClarabelCscMatrix_f64, RowVal, rowval);
    FIELD(ClarabelCscMatrix, ClarabelCscMatrix_f64, NzVal, nzval);
    SIZE(ClarabelSupportedCone, ClarabelSupportedConeT_f64);
    FIELD(ClarabelSupportedCone, ClarabelSupportedConeT_f64, Tag, tag);
    FIELD(ClarabelSupportedCone, ClarabelSupportedConeT_f64, Dim, zero_cone_t);
    SIZE(ClarabelDefaultSolution, ClarabelDefaultSolution_f64);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, X, x);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, XLength, x_length);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, Z, z);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, ZLength, z_length);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, S, s);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, SLength, s_length);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, Status, status);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, ObjVal, obj_val);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, ObjValDual, obj_val_dual);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, SolveTime, solve_time);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, Iterations, iterations);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, RPrim, r_prim);
    FIELD(ClarabelDefaultSolution, ClarabelDefaultSolution_f64, RDual, r_dual);
    SIZE(ClarabelDefaultSettings, ClarabelDefaultSettings_f64);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, MaxIter, max_iter);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, TimeLimit, time_limit);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, Verbose, verbose);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, MaxStepFraction, max_step_fraction);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, TolGapAbs, tol_gap_abs);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, TolGapRel, tol_gap_rel);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, TolFeas, tol_feas);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, TolInfeasAbs, tol_infeas_abs);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, TolInfeasRel, tol_infeas_rel);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, TolKtratio, tol_ktratio);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, ReducedTolGapAbs, reduced_tol_gap_abs);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, ReducedTolGapRel, reduced_tol_gap_rel);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, ReducedTolFeas, reduced_tol_feas);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, ReducedTolInfeasAbs, reduced_tol_infeas_abs);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, ReducedTolInfeasRel, reduced_tol_infeas_rel);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, ReducedTolKtratio, reduced_tol_ktratio);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, EquilibrateEnable, equilibrate_enable);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, EquilibrateMaxIter, equilibrate_max_iter);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, EquilibrateMinScaling, equilibrate_min_scaling);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, EquilibrateMaxScaling, equilibrate_max_scaling);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, LinesearchBacktrackStep, linesearch_backtrack_step);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, MinSwitchStepLength, min_switch_step_length);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, MinTerminateStepLength, min_terminate_step_length);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, MaxThreads, max_threads);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, DirectKktSolver, direct_kkt_solver);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, DirectSolveMethod, direct_solve_method);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, StaticRegularizationEnable, static_regularization_enable);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, StaticRegularizationConstant, static_regularization_constant);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, StaticRegularizationProportional, static_regularization_proportional);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, DynamicRegularizationEnable, dynamic_regularization_enable);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, DynamicRegularizationEps, dynamic_regularization_eps);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, DynamicRegularizationDelta, dynamic_regularization_delta);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, IterativeRefinementEnable, iterative_refinement_enable);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, IterativeRefinementReltol, iterative_refinement_reltol);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, IterativeRefinementAbstol, iterative_refinement_abstol);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, IterativeRefinementMaxIter, iterative_refinement_max_iter);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, IterativeRefinementStopRatio, iterative_refinement_stop_ratio);
    FIELD(ClarabelDefaultSettings, ClarabelDefaultSettings_f64, PresolveEnable, presolve_enable);
    return 0;
}
