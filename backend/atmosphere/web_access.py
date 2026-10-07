"""Explicit browser origins for the loopback calculation service."""
import json
import os
from pathlib import Path
from urllib.parse import urlsplit
from fastapi.middleware.cors import CORSMiddleware
from starlette.datastructures import Headers

LOCAL_ORIGINS=['http://127.0.0.1:5173','http://localhost:5173','http://127.0.0.1:8000','http://127.0.0.1:8001']

def normalize_origin(value):
    url=urlsplit(value.strip())
    if url.scheme!='https' or not url.hostname or url.username or url.password or url.path not in ('','/') or url.query or url.fragment:
        raise ValueError('Enter the HTTPS website origin, without a path, credentials or query.')
    return f'https://{url.netloc.lower()}'

def allowed_origins(root=None):
    root=Path(root or Path(__file__).resolve().parents[2])
    values=[v for v in os.environ.get('ATMOSPHERE_WEB_ORIGINS','').split(',') if v.strip()]
    path=root/'data'/'web-origins.json'
    if path.exists():values.extend(json.loads(path.read_text(encoding='utf-8')))
    return list(dict.fromkeys([*LOCAL_ORIGINS,*(normalize_origin(v) for v in values)]))

class LocalEngineCORS(CORSMiddleware):
    def preflight_response(self,request_headers):
        # Starlette versions differ in built-in PNA handling. Validate ordinary
        # CORS first, then grant private-network access only to accepted origins.
        ordinary=Headers({k:v for k,v in request_headers.items() if k!='access-control-request-private-network'})
        response=super().preflight_response(ordinary)
        if (response.status_code==200 and self.is_allowed_origin(request_headers.get('origin',''))
                and request_headers.get('access-control-request-private-network')=='true'):
            response.headers['Access-Control-Allow-Private-Network']='true'
        return response
