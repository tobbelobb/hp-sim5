"""Protect historical comparisons against native/wrapper masking."""
import json
from pathlib import Path
import subprocess
import pytest
from autocal.tools.native_snapshot import validate_native,prepare_snapshot,snapshot_environment,sha256,ENGINE,WRAPPER


def test_reject_native_from_other_revision():
    native=b'binary';engine=b'historical engine'
    metadata={'artifactSha256':sha256(native),'sources':{'src/engine.rs':sha256(b'current engine')}}
    with pytest.raises(ValueError,match='requested revision'):
        validate_native(native,metadata,engine)


def test_reject_modified_binary():
    with pytest.raises(ValueError,match='binary'):
        validate_native(b'changed',{'artifactSha256':sha256(b'original')},b'engine')


def test_archived_wrapper_is_used_without_editing_checkout(tmp_path):
    root=tmp_path/'repo';root.mkdir()
    subprocess.run(['git','init','-q'],cwd=root,check=True)
    for name,data in [(ENGINE,b'engine'),(WRAPPER,b'export const slot=1;')]:
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
    subprocess.run(['git','add','.'],cwd=root,check=True)
    subprocess.run(['git','-c','user.name=Test','-c','user.email=test@example.com','commit','-qm','old'],cwd=root,check=True)
    native=tmp_path/'old.node';native.write_bytes(b'native')
    Path(str(native)+'.json').write_text(json.dumps({'artifactSha256':sha256(b'native'),'sources':{'src/engine.rs':sha256(b'engine')}}))
    (root/WRAPPER).write_text('export const slot=0;')
    output=tmp_path/'snapshot'
    spec=prepare_snapshot(root,'HEAD',native,output)
    script=f"import {{slot}} from {json.dumps((root/WRAPPER).as_uri())};if(slot!==1)throw Error('historical wrapper masked');"
    subprocess.run(['node','--import',(output/'loader.mjs').as_uri(),'--input-type=module','-e',script],cwd=root,check=True)
    receipt=json.loads((output/'loads.jsonl').read_text().strip())
    assert receipt['wrapper_sha256']==spec['wrapper_sha256']
    assert (root/WRAPPER).read_text()=='export const slot=0;'
    assert spec['wrapper_sha256']==sha256(b'export const slot=1;')
    native.write_bytes(b'tampered')
    with pytest.raises(ValueError,match='binary'):
        snapshot_environment(output)
    (output/'wrapper.js').write_text('export const slot=0;')
    result=subprocess.run(['node','--import',(output/'loader.mjs').as_uri(),'-e',''],capture_output=True,text=True)
    assert result.returncode != 0
    assert 'Snapshot wrapper was modified' in result.stderr
