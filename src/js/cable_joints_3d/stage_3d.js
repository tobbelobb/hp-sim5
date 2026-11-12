import * as THREE from 'three';

// Photographic studio around the machine: a seamless grey backdrop, a floor
// under the machine that fades into it, and soft studio light. Nothing here
// stands between the camera and the simulated parts.

// Graphite studio (OKLCH hue 255): backdrop L 0.31, floor L 0.34 fading into
// it. A flat floor under directional light is uniformly lit, so it is drawn
// unlit: the cheapest full-screen material, which matters on software GL.
const BACKDROP_COLOR = 0x2d3135;
const FLOOR_COLOR = 0x35383d;

// Reflection environment: a grey studio dome, darker towards the floor, with
// one large overhead softbox and a smaller side fill. Built Y-up.
function createStudioEnvironment() {
  const scene = new THREE.Scene();
  const dome = new THREE.SphereGeometry(10, 32, 16);
  const colors = [];
  const position = dome.attributes.position;
  const color = new THREE.Color();
  for (let i = 0; i < position.count; i += 1) {
    const height = position.getY(i) / 10;
    color.setScalar(THREE.MathUtils.lerp(0.18, 0.75, THREE.MathUtils.smoothstep(height, -0.6, 0.8)));
    colors.push(color.r, color.g, color.b);
  }
  dome.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  scene.add(new THREE.Mesh(dome, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide })));

  const softbox = new THREE.Mesh(
    new THREE.PlaneGeometry(6, 6),
    new THREE.MeshBasicMaterial({ color: new THREE.Color(5, 5, 5), side: THREE.DoubleSide })
  );
  softbox.position.set(0, 7, 2);
  softbox.lookAt(0, 0, 0);
  scene.add(softbox);
  const fill = new THREE.Mesh(
    new THREE.PlaneGeometry(3, 4),
    new THREE.MeshBasicMaterial({ color: new THREE.Color(1.6, 1.6, 1.7), side: THREE.DoubleSide })
  );
  fill.position.set(-8, 2, -3);
  fill.lookAt(0, 0, 0);
  scene.add(fill);
  return scene;
}

// Mixes the lit floor colour into the backdrop with distance from the machine,
// so the floor has no visible edge.
function fadeFloorIntoBackdrop(material, uniforms) {
  material.onBeforeCompile = (shader) => {
    shader.uniforms.uFadeRadius = uniforms.uFadeRadius;
    shader.uniforms.uBackdrop = uniforms.uBackdrop;
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec2 vFloorXY;')
      .replace('#include <worldpos_vertex>', '#include <worldpos_vertex>\nvFloorXY = (modelMatrix * vec4(transformed, 1.0)).xy;');
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', '#include <common>\nvarying vec2 vFloorXY;\nuniform vec2 uFadeRadius;\nuniform vec3 uBackdrop;')
      .replace(
        '#include <dithering_fragment>',
        'float floorFade = smoothstep(uFadeRadius.x, uFadeRadius.y, length(vFloorXY));\n' +
        'gl_FragColor.rgb = mix(gl_FragColor.rgb, uBackdrop, floorFade);\n#include <dithering_fragment>'
      );
  };
}

// CPU rasterizers pay for every full-screen layer; they get the plain backdrop.
function isSoftwareRenderer(renderer) {
  const gl = renderer.getContext();
  const info = gl.getExtension('WEBGL_debug_renderer_info');
  const name = info ? gl.getParameter(info.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
  return /swiftshader|llvmpipe|softpipe|software/i.test(String(name));
}

export class Stage3D {
  constructor(scene, renderer) {
    this.scene = scene;
    this.group = new THREE.Group();
    this.group.name = 'stage';
    scene.add(this.group);

    renderer.toneMapping = THREE.NeutralToneMapping;
    scene.background = new THREE.Color(BACKDROP_COLOR);

    this.renderer = renderer;
    this._buildEnvironment();
    // Render-target textures do not survive a lost WebGL context: forget the
    // dead target on loss and build a new one on restore.
    this._onContextLost = () => {
      this.environmentTarget = null;
    };
    this._onContextRestored = () => this._buildEnvironment();
    renderer.domElement.addEventListener('webglcontextlost', this._onContextLost);
    renderer.domElement.addEventListener('webglcontextrestored', this._onContextRestored);
    scene.environmentIntensity = 0.8;
    scene.environmentRotation.set(Math.PI / 2, 0, 0);

    const hemisphere = new THREE.HemisphereLight(0xf4f6fa, 0x8c8a86, 0.7);
    hemisphere.position.set(0, 0, 1);
    this.group.add(hemisphere);
    const key = new THREE.DirectionalLight(0xfffaf4, 2.0);
    key.position.set(1.2, -2.0, 6.0);
    this.group.add(key);

    this.fadeUniforms = {
      uFadeRadius: { value: new THREE.Vector2(3, 6) },
      // Mixed after tone mapping and output encoding, so it is stored in sRGB
      // to match the untone-mapped background exactly.
      uBackdrop: { value: new THREE.Color(BACKDROP_COLOR).convertLinearToSRGB() },
    };
    // Not tone mapped, like the background it fades into: the tone curve's toe
    // would crush this dark colour towards black. Drawn first without depth,
    // so it is part of the background and can never hide simulated geometry.
    this.floorMaterial = new THREE.MeshBasicMaterial({
      color: FLOOR_COLOR,
      toneMapped: false,
      depthTest: false,
      depthWrite: false,
    });
    fadeFloorIntoBackdrop(this.floorMaterial, this.fadeUniforms);
    this.floor = new THREE.Mesh(new THREE.CircleGeometry(1, 96), this.floorMaterial);
    this.floor.renderOrder = -1000;
    this.floor.visible = !isSoftwareRenderer(renderer);
    this.group.add(this.floor);

    this._layoutKey = '';
  }

  _buildEnvironment() {
    this.environmentTarget?.dispose();
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    const studio = createStudioEnvironment();
    this.environmentTarget = pmrem.fromScene(studio, 0.02);
    studio.traverse((object) => {
      object.geometry?.dispose();
      object.material?.dispose();
    });
    pmrem.dispose();
    this.scene.environment = this.environmentTarget.texture;
  }

  setLayout({ reach, floorZ }) {
    const key = `${reach.toFixed(4)}|${floorZ.toFixed(4)}`;
    if (key === this._layoutKey) {
      return;
    }
    this._layoutKey = key;

    const floorRadius = reach * 6;
    this.floor.scale.setScalar(floorRadius);
    this.floor.position.set(0, 0, floorZ);
    this.fadeUniforms.uFadeRadius.value.set(reach * 1.35, reach * 3.2);
  }

  dispose() {
    this.scene.remove(this.group);
    this.group.traverse((object) => {
      object.geometry?.dispose?.();
      if (object.material) {
        object.material.dispose();
      }
    });
    this.renderer.domElement.removeEventListener('webglcontextlost', this._onContextLost);
    this.renderer.domElement.removeEventListener('webglcontextrestored', this._onContextRestored);
    this.environmentTarget?.dispose();
    this.scene.environment = null;
  }
}
