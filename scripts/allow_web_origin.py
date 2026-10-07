"""Authorize one named website to use the local loopback engine."""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from atmosphere.web_access import normalize_origin

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('origin',help='Your production website address, for example https://your-project.vercel.app')
args=parser.parse_args()
try:origin=normalize_origin(args.origin)
except ValueError as exc:parser.error(str(exc))
path=ROOT/'data'/'web-origins.json'
values=json.loads(path.read_text(encoding='utf-8')) if path.exists() else []
values=list(dict.fromkeys([*values,origin]))
path.parent.mkdir(parents=True,exist_ok=True)
path.write_text(json.dumps(values,indent=2)+'\n',encoding='utf-8')
print(f'Local engine browser access enabled for {origin}. Restart the engine to apply.')
