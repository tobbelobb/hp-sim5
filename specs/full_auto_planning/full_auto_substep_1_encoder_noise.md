# Substep 1: Add encoder noise measurements to every collected datapoint

## Objective
Capture per-pose encoder noise so residuals can be noise-normalized (whitened). This makes the objective dimensionless and comparable across machines and datasets.

## Required data collection changes
At each static pose:
1. **Hold still**: settle for a fixed time.
2. **Sample encoder**: collect `S` fast encoder readings per cable.
3. **Store**:
   - `mu[i][k]`: mean encoder value per cable.
   - `sigma[i][k]`: robust noise estimate per cable (MAD-based std).
   - Optional: sampling rate, timestamp, `S`.

## Dataset schema additions
For each pose entry:
```json
{
  "pose": {"x": ..., "y": ..., "z": ...},
  "mu": [ ... per cable ... ],
  "sigma": [ ... per cable ... ],
  "sample_count": S,
  "sampling_hz": ...
}
```

## Robust sigma policy
- Use MAD-to-std conversion:
  - `med = median(x)`
  - `MAD = median(|x - med|)`
  - `robust_std(x) = 1.4826 * MAD`
- Apply a sigma floor to avoid infinite weighting:
  - `sigma_eff[i][k] = max(sigma[i][k], sigma_min)`

## Acceptance checks (data quality)
- Flag (but do not delete) poses with:
  - `sigma` that is non-finite or below the floor.
  - Too few samples (below a configured minimum).

## Deliverables
- Updated data collection in the sweep recorder.
- Dataset migration or compatibility logic to handle pre-noise datasets.
- Unit tests for sigma estimation and floor logic.
