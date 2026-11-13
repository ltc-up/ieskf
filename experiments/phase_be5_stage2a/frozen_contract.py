"""Fail-closed numeric-frame contract for existing BE snapshots only."""
import numpy as np
from scipy.spatial.transform import Rotation

def qualify_frame(ndt, prior, seed, stamp, lever, certificate):
    required = ('direct_matching_map', 'base_source', 'native_parameters_and_covariance_same_chart',
                'no_production_PCM_to_ENU_adapter')
    if certificate.get('status') != 'CONFIRMED_SAVED_RESEARCH_NUMERIC_FRAME' or not all(certificate.get(k) is True for k in required):
        raise ValueError('PCM_MAP_CONTRACT_UNCONFIRMED')
    if int(ndt['header']) != int(stamp) or int(prior.t) != int(stamp):
        raise ValueError('FROZEN_TIMESTAMP_MISMATCH')
    expected=np.eye(4);expected[:3,:3]=prior.R;expected[:3,3]=prior.p-prior.R@np.asarray(lever)
    seed=np.asarray(seed,float)
    if seed.shape!=(4,4) or not np.allclose(seed,expected,atol=1e-9,rtol=0):
        raise ValueError('BASE_IMU_SEED_CONTRACT')
    if not np.allclose(np.asarray(ndt['seed_float']),seed.astype(np.float32),atol=1e-7,rtol=0):
        raise ValueError('MATCHER_SEED_FRAME')
    q=np.asarray(ndt['parameters_internal'],float).reshape(6)
    final=np.asarray(ndt['final'],float)
    if final.shape!=(4,4) or not np.isfinite(np.r_[q,final.ravel()]).all() or not np.allclose(final[3],[0,0,0,1],atol=1e-12,rtol=0):
        raise ValueError('NATIVE_FINAL_INVALID')
    if not np.allclose(Rotation.from_euler('XYZ',q[3:]).as_matrix(),final[:3,:3],atol=2e-5,rtol=0) or not np.allclose(q[:3],final[:3,3],atol=2e-5,rtol=2e-7):
        raise ValueError('NATIVE_FINAL_CHART_UNVERIFIED')
    # This is NOT a declaration about the deployed ROS PCM/ENU adapter or datum.
    return np.eye(4)

def qualify_dataset_certificate(certificate, run_name, captured_source_hashes, expected_target_sha):
    """Bind provenance to the actual saved run before any real-data candidate."""
    if (certificate.get('status') != 'CONFIRMED_SAVED_RESEARCH_NUMERIC_FRAME' or
        certificate.get('allowed_run') != run_name or
        certificate.get('target_sha256') != expected_target_sha or
        sorted(certificate.get('source_hashes', {}).values()) != sorted(captured_source_hashes.values())):
        raise ValueError('PCM_MAP_DATASET_BINDING_UNCONFIRMED')
