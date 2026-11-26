import { createInspectionToolsController } from '../../../hp-sim-3d/app/inspectionToolsController.js';

test('a pending reference file cannot replace an overlay hidden during reset', async () => {
  const state = { referenceOverlay: { visible: false, segments: null } };
  const controller = createInspectionToolsController({ state, dom: {}, world: { getResource: () => null } });
  let finish;
  const pending = controller.loadFromFile({
    name: 'tracker.gcode', size: 20, lastModified: 0,
    text: () => new Promise(resolve => { finish = resolve; }),
  }, { makeVisible: true });
  controller.setVisibility(false);
  finish('G1 X10 E1');
  expect((await pending).segments).toHaveLength(1);
  expect(state.referenceOverlay).toEqual({ visible: false, segments: null });
});
