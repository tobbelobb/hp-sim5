"""Prepare a matched native engine/wrapper for bounded research collection.

The Node loader substitutes archived wrapper source at its original module URL,
so relative imports retain their meaning without changing the working tree.
This isolates the selected Rust engine and wrapper, not an entire historical app.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ENGINE = 'src/wasm/cable_solver/src/engine.rs'
WRAPPER = 'src/js/cable_joints_3d/rust_physics_engine.js'


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def validate_native(artifact, metadata, engine_source):
    if metadata.get('artifactSha256') != sha256(artifact):
        raise ValueError('Native binary does not match its build metadata')
    if metadata.get('sources', {}).get('src/engine.rs') != sha256(engine_source):
        raise ValueError('Native engine source does not match the requested revision')


def prepare_snapshot(root: Path, revision: str, native: Path, output: Path):
    root, native, output = root.resolve(), native.resolve(), output.resolve()
    revision = subprocess.check_output(['git', 'rev-parse', '--verify', revision + '^{commit}'], cwd=root, text=True).strip()
    source = lambda path: subprocess.check_output(['git', 'show', f'{revision}:{path}'], cwd=root)
    engine, wrapper = source(ENGINE), source(WRAPPER)
    metadata = json.loads(Path(str(native) + '.json').read_text())
    validate_native(native.read_bytes(), metadata, engine)
    # Refuse reuse: an existing archive may belong to another running process.
    output.mkdir(parents=True, exist_ok=False)
    (output/'wrapper.js').write_bytes(wrapper)
    (output/'engine.rs').write_bytes(engine)
    (output/'native-build.json').write_text(json.dumps(metadata, indent=2)+'\n')
    manifest = dict(revision=revision, native_artifact=str(native), native_sha256=sha256(native.read_bytes()),
                    engine_sha256=sha256(engine), wrapper_sha256=sha256(wrapper),
                    original_wrapper_url=(root/WRAPPER).as_uri(), scope='engine.rs and wrapper only; current surrounding app')
    (output/'snapshot.json').write_text(json.dumps(manifest, indent=2)+'\n')
    # Node's load hook preserves the original URL, including relative imports.
    # Check archive identity at process startup, before loading any engine.
    loader = """import {registerHooks} from 'node:module';
import fs from 'node:fs';
import {createHash} from 'node:crypto';
const spec=JSON.parse(fs.readFileSync(new URL('./snapshot.json',import.meta.url)));
const source=fs.readFileSync(new URL('./wrapper.js',import.meta.url),'utf8');
if(createHash('sha256').update(source).digest('hex')!==spec.wrapper_sha256) throw Error('Snapshot wrapper was modified');
registerHooks({load(url,context,nextLoad){
 if(url===spec.original_wrapper_url){
  fs.appendFileSync(new URL('./loads.jsonl',import.meta.url),JSON.stringify({pid:process.pid,url,wrapper_sha256:spec.wrapper_sha256})+'\\n');
  return {format:'module',source,shortCircuit:true};
 }
 return nextLoad(url,context);
}});
"""
    (output/'loader.mjs').write_text(loader)
    return manifest


def snapshot_environment(output: Path):
    output = output.resolve()
    manifest = json.loads((output/'snapshot.json').read_text())
    native = Path(manifest['native_artifact'])
    metadata = json.loads((output/'native-build.json').read_text())
    validate_native(native.read_bytes(), metadata, (output/'engine.rs').read_bytes())
    if sha256(native.read_bytes()) != manifest['native_sha256']:
        raise ValueError('Snapshot native binary was modified')
    if os.environ.get('NODE_OPTIONS'):
        raise ValueError('Unset NODE_OPTIONS before a controlled snapshot run')
    return dict(os.environ, HP_SIM5_NATIVE_ENGINE=manifest['native_artifact'],
                NODE_OPTIONS='--import='+(output/'loader.mjs').as_uri())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision',required=True)
    parser.add_argument('--native',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    print(json.dumps(prepare_snapshot(Path(__file__).resolve().parents[2], args.revision,args.native,args.output),indent=2))

if __name__=='__main__':main()
