"""Build a deterministic source archive using an explicit source allowlist.

No Git history, SDK, resources, private configurations or runtime state is exported.
"""
import hashlib
import json
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
NAME='aimooe-navigation-0.1.0'
TOP={'README.md','README.zh-CN.md','LICENSE','CITATION.cff','THIRD_PARTY_NOTICES.md',
     'CHANGELOG.md','CONTRIBUTING.md','VERSION','.gitignore','LaunchOptimized082.cmd',
     'requirements-test.txt','requirements-metadata.txt'}
CONFIG={'environment.cmd','open_optimized082.py','translations-en.json','version.json','tracking.example.json'}
def files():
    chosen=[]
    for path in sorted(ROOT.rglob('*')):
        if not path.is_file() or path.is_symlink(): continue
        rel=path.relative_to(ROOT)
        if '__pycache__' in rel.parts: continue
        allowed=(len(rel.parts)==1 and rel.name in TOP
          or rel.parts[0]=='Config' and len(rel.parts)==2 and rel.name in CONFIG
          or rel.parts[0]=='AimooeNavigation' and path.suffix=='.py' and
             (len(rel.parts)==2 or len(rel.parts)==3 and rel.parts[1]=='AimooeLib')
          or rel.parts[0] in {'tests','examples','scripts','integration'} and len(rel.parts)==2 and path.suffix=='.py'
          or rel.parts[0]=='docs' and len(rel.parts)==2 and path.suffix=='.md'
          or rel.as_posix()=='.github/workflows/ci.yml')
        if allowed: chosen.append(path)
    return chosen

def build():
    output=ROOT/'dist'; output.mkdir(exist_ok=True)
    manifest=[]
    archive=output/(NAME+'.zip')
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for path in files():
            data=path.read_bytes(); rel=path.relative_to(ROOT).as_posix()
            manifest.append({'path':rel,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
            info=zipfile.ZipInfo(NAME+'/'+rel,date_time=(2026,11,1,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED; info.external_attr=0o100644<<16
            z.writestr(info,data)
    (output/'MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    (output/'SHA256SUMS').write_text(f'{digest}  {archive.name}\n',encoding='ascii')
    print(f'SOURCE ARCHIVE: {len(manifest)} files, {digest}  {archive.name}')

if __name__=='__main__': build()
