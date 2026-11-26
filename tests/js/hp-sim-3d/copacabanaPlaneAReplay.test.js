import { runCopacabanaReplay } from '../../../scripts/reference_data/validate_4614_replay.mjs';

describe('COPacabana PlaneA replay numerical regression', () => {
  test('planeATargets tracks XML targets with moving swivel pulleys and closed-loop motors', async () => {
    const res = await runCopacabanaReplay('planeATargets');
    expect(res.kinematic_motors).toBe(false);
    expect(res.samples).toBe(1214);
    expect(res.settled).toHaveLength(5);
    expect(res.max_spool_tracking_error_rad).toBeLessThan(0.05);
    expect(res.reflector_rms_mm).toBeLessThan(27.0);
    expect(res.cable_force_rms_N).toBeLessThan(185.0);
    expect(res.cable_force_rms_scaled_sqrt2_N).toBeLessThan(270.0);
    for (const hold of res.settled) {
      for (const axisErrMm of hold.simulated_minus_nominal_mm) {
        expect(Math.abs(axisErrMm)).toBeLessThan(2.5);
      }
    }
  });

  test('planeAEncoders animates spools kinematically to machine precision', async () => {
    const res = await runCopacabanaReplay('planeAEncoders');
    expect(res.kinematic_motors).toBe(true);
    expect(res.samples).toBe(1214);
    expect(res.settled).toHaveLength(5);
    expect(res.max_spool_tracking_error_rad).toBeLessThan(1e-9);
    expect(res.reflector_rms_mm).toBeLessThan(17.0);
    expect(res.cable_force_rms_N).toBeLessThan(180.0);
    expect(res.cable_force_rms_scaled_sqrt2_N).toBeLessThan(270.0);
    for (const hold of res.settled) {
      for (const axisErrMm of hold.simulated_minus_nominal_mm) {
        expect(Math.abs(axisErrMm)).toBeLessThan(2.5);
      }
    }
  });
});
