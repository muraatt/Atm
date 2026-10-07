"""Identify non-native JSON leaves without concealing them with a serializer."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
from verify_system_matrix import scenario_for,ROOT
from atmosphere.optimization import optimize


def numpy_leaves(value,path='root'):
    if isinstance(value,dict):
        return sum((numpy_leaves(v,path+'.'+k) for k,v in value.items()),[])
    if isinstance(value,(list,tuple)):
        return sum((numpy_leaves(v,f'{path}[{i}]') for i,v in enumerate(value)),[])
    return [{'path':path,'type':type(value).__name__,'value':str(value)}] if isinstance(value,np.generic) else []


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--case',default='leo-cape');parser.add_argument('--budget',type=float,default=5);args=parser.parse_args()
    s=scenario_for(args.case,args.budget)
    result=optimize(s)
    print(json.dumps([leaf for leaf in numpy_leaves(result) if leaf['type']!='float64'],indent=2),flush=True)
    dest=ROOT/'data/system-validation/json-diagnostic';dest.mkdir(parents=True,exist_ok=True)
    (dest/'flight-plan.json').write_text(json.dumps(result['flight_plan']),encoding='utf-8')
    (dest/'numpy-leaves.json').write_text(json.dumps(numpy_leaves(result),indent=2),encoding='utf-8')
    (dest/'result.json').write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
