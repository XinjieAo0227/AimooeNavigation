"""Validate citation metadata against a locally supplied official CFF schema.

Install requirements-metadata.txt; download schema independently from
https://raw.githubusercontent.com/citation-file-format/citation-file-format/1.2.0/schema.json
Run: python scripts/validate_metadata.py path/to/schema.json
Missing metadata or a pending license fails this publication gate.
"""
import json
import sys
from pathlib import Path
import yaml
import jsonschema

root=Path(__file__).resolve().parents[1]
schema=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
document=yaml.safe_load((root/'CITATION.cff').read_text(encoding='utf-8'))
errors=list(jsonschema.Draft7Validator(schema).iter_errors(document))
for error in errors: print('CFF:',error.message)
print('CFF schema:', 'FAIL' if errors else 'PASS')
if 'LICENSE SELECTION PENDING' in (root/'LICENSE').read_text(encoding='utf-8'):
    print('LICENSE: PENDING (not an open-source grant)')
    raise SystemExit(1)
raise SystemExit(bool(errors))
