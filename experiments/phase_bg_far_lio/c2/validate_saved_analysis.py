"""Audit assertions on saved aggregates only; no filter or registration execution."""
import ast
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    a = json.loads((root / 'saved_log_aggregate.json').read_text())
    t = json.loads((root / 'temporal_aggregate.json').read_text())
    checks = []

    def check(name, condition):
        if not condition:
            raise AssertionError(name)
        checks.append(name)

    check('saved_K_residual_delta_identity', a['checks']['max_saved_K_times_residual_delta_error'] < 1e-12)
    check('saved_translation_innovation_identity', a['checks']['max_saved_translation_innovation_vs_seed_difference'] < 1e-12)
    check('4214_completion_pairs', t['checks']['saved_completion_pairs'] == 4214)
    check('completion_pair_timing_under_5ms', t['checks']['max_completion_pair_lag_ms'] < 5)
    for name, n in [('normal8', 129), ('right3', 89)]:
        f = a['FAR'][name]
        w = t['windows'][name]
        check(name + '_all_scored_position_residuals_inside_clip_bound', f['position_clip_guaranteed_none_snapshots'] == f['snapshot_count'])
        check(name + '_Rxy_constant', len(w['R_value_counts']['x_m']) == len(w['R_value_counts']['y_m']) == 1)
        check(name + '_yaw_R_floor_delay_formula', max(abs(w['yaw_R_minus_floor_delay_formula'][k]) for k in ['min', 'max']) < 2e-10)
        check(name + '_saved_scoring_sample_count', a['lateral'][name]['prior_lateral']['n'] == n)
    # Prevent accidental algorithm imports in these descriptive readers.
    for filename in ['analyze_existing.py', 'temporal_existing.py']:
        tree = ast.parse((root / filename).read_text())
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(x.name for x in node.names)
            elif isinstance(node, ast.ImportFrom):
                imports.append(node.module or '')
        check(filename + '_no_filter_matcher_ROS_import', not any(s.startswith(('geometry_ieskf', 'ndt_global', 'rclpy', 'ctypes')) for s in imports))
    print(json.dumps({'scope': 'saved algebra/contracts only; not localization validation', 'checks_passed': len(checks), 'checks': checks}, indent=2))


if __name__ == '__main__':
    main()
