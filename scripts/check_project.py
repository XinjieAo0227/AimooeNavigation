"""Offline structural checks; incomplete licensing/CFF is reported, never silently passed."""
import ast
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
def check():
    count = 0
    for folder in ['AimooeNavigation', 'scripts', 'tests', 'examples', 'Config', 'integration']:
        for path in (ROOT / folder).rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
            count += 1
    for path in (ROOT / 'Config').glob('*.json'):
        json.loads(path.read_text(encoding='utf-8'))
    assert (ROOT/'VERSION').read_text().strip() == '0.1.0'
    assert json.loads((ROOT/'Config/version.json').read_text())['version'] == '0.1.0'
    for path in ROOT.rglob('*.md'):
        if 'dist' in path.relative_to(ROOT).parts: continue
        for link in re.findall(r'\]\(([^)]+)\)',path.read_text(encoding='utf-8')):
            if '://' in link or link.startswith('#'): continue
            assert (path.parent/link.split('#')[0]).exists(), (path,link)
    workflow=(ROOT/'.github/workflows/ci.yml').read_text()
    assert 'contents: read' in workflow and 'pull_request_target' not in workflow
    assert 'git push' not in workflow and 'gh release' not in workflow
    assert 'LaunchOptimized082.cmd' in (ROOT/'README.md').read_text()
    print(f'PASS: parsed {count} Python files, JSON, relative doc links, versions, CI guards.')
    assert (ROOT/'LICENSE').read_text(encoding='utf-8').startswith('MIT License\n')
    print('PASS: MIT license present; validate CFF separately with the official schema.')

if __name__ == '__main__': check()
