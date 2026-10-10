"""Verify cache configuration in fresh processes before JAX initializes."""
import os
import subprocess
import sys

import pytest


@pytest.mark.parametrize('disk_cache', [False, True])
def test_persistent_cache_is_opt_in(tmp_path, disk_cache):
    pytest.importorskip('jax')
    env = dict(os.environ)
    env.pop('JAX_COMPILATION_CACHE_DIR', None)
    env.pop('JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS', None)
    if disk_cache:
        env['JAX_COMPILATION_CACHE_DIR'] = str(tmp_path)
        env['JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS'] = '2.5'
    code = '''
from autocal import ellipse_objective_jax
import jax
import jax.numpy as jnp
print(jax.config.jax_compilation_cache_dir)
print(jax.config.jax_persistent_cache_min_compile_time_secs)
print(jax.jit(lambda x: jnp.sum(x*x))(jnp.ones(100)))
'''
    for _ in range(2):
        result = subprocess.run([sys.executable, '-c', code], env=env,
                                capture_output=True, text=True, check=True)
        lines = result.stdout.splitlines()
        assert lines[0] == (str(tmp_path) if disk_cache else 'None')
        assert float(lines[1]) == (2.5 if disk_cache else 1.0)
        assert lines[2] == '100.0'
        if not disk_cache:
            assert 'PjRt-IFRT' not in result.stderr
