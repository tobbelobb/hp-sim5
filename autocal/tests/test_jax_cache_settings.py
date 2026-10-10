"""Verify native logging and cache defaults before JAX initializes."""
import json
from importlib.metadata import version
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('mode', ['default', 'disabled', 'custom', 'verbose'])
def test_jax_cache_and_native_logging_settings(tmp_path, mode):
    pytest.importorskip('jax')
    env = dict(os.environ)
    for key in ('JAX_COMPILATION_CACHE_DIR', 'JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS',
                'JAX_ENABLE_COMPILATION_CACHE', 'TF_CPP_MIN_LOG_LEVEL'):
        env.pop(key, None)
    default_dir = Path(__file__).resolve().parents[2] / 'output' / 'autocal-jax-cache'
    expected_dir, expected_threshold = default_dir, 0.0
    if mode == 'disabled':
        env['JAX_ENABLE_COMPILATION_CACHE'] = 'false'
    if mode in ('custom', 'verbose'):
        env['JAX_COMPILATION_CACHE_DIR'] = str(tmp_path)
        env['JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS'] = '0' if mode == 'verbose' else '2.5'
        expected_dir, expected_threshold = tmp_path, (0.0 if mode == 'verbose' else 2.5)
    if mode == 'verbose':
        env['TF_CPP_MIN_LOG_LEVEL'] = '1'
    code = """
from autocal import ellipse_objective_jax
import json, os, warnings
import jax
import jax.numpy as jnp
print(json.dumps(dict(directory=jax.config.jax_compilation_cache_dir,
    threshold=jax.config.jax_persistent_cache_min_compile_time_secs,
    enabled=jax.config.jax_enable_compilation_cache, native_level=os.environ['TF_CPP_MIN_LOG_LEVEL'])))
print(jax.jit(lambda x: jnp.sum(x*x))(jnp.ones(100)))
warnings.warn('Python warning remains visible')
os.write(2, b'Unrelated stderr remains visible\\n')
"""
    for repeat in range(2):
        result = subprocess.run([sys.executable, '-c', code], env=env,
                                capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        assert json.loads(lines[0]) == dict(directory=str(expected_dir), threshold=expected_threshold,
            enabled=mode != 'disabled', native_level='1' if mode == 'verbose' else '2')
        assert lines[1] == '100.0'
        assert 'Python warning remains visible' in result.stderr
        assert 'Unrelated stderr remains visible' in result.stderr
        if mode != 'verbose':
            assert 'PjRt-IFRT' not in result.stderr
        elif repeat and version('jaxlib') == '0.9.2':
            assert 'PjRt-IFRT' in result.stderr
