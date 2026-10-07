import {afterEach,describe,expect,it,vi} from 'vitest';
import {engineApiUrl,engineBase,normalizeEngineUrl} from './connection';

afterEach(()=>vi.unstubAllGlobals());
describe('loopback calculation connection',()=>{
  it('allows explicit local ports and rejects remote or credential-bearing addresses',()=>{
    expect(normalizeEngineUrl('http://127.0.0.1:8001/')).toBe('http://127.0.0.1:8001');
    expect(normalizeEngineUrl('http://localhost:8000')).toBe('http://localhost:8000');
    for(const url of ['https://example.com','http://127.0.0.1.example.com','http://user:secret@localhost','http://localhost/api','http://localhost?token=abc','file:///etc/passwd'])expect(()=>normalizeEngineUrl(url)).toThrow();
  });
  it('does not send API calls to the hosted website when no engine is configured',()=>{
    vi.stubGlobal('location',{hostname:'atm-test.vercel.app'});vi.stubGlobal('localStorage',{getItem:()=>null});
    expect(engineBase()).toBe(null);expect(()=>engineApiUrl('/api/analyses')).toThrow('Connect your local engine');
  });
  it('routes calculations directly to the configured loopback engine',()=>{
    vi.stubGlobal('location',{hostname:'atm-test.vercel.app'});vi.stubGlobal('localStorage',{getItem:(key:string)=>key==='atmosphere-engine-url'?'http://127.0.0.1:8001':null});
    expect(engineApiUrl('/api/analyses')).toBe('http://127.0.0.1:8001/api/analyses');
  });
  it('preserves same-origin local operation and supports explicit review mode',()=>{
    vi.stubGlobal('location',{hostname:'127.0.0.1'});vi.stubGlobal('localStorage',{getItem:()=>null});expect(engineBase()).toBe('');
    vi.stubGlobal('localStorage',{getItem:(key:string)=>key==='atmosphere-review-only'?'true':null});expect(engineBase()).toBe(null);
  });
});
