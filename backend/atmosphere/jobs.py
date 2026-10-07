from __future__ import annotations

import json
import multiprocessing as mp
import os
import threading
import time
from pathlib import Path
from uuid import uuid4

from .models import Scenario
from .terrain import ROOT

JOBS=ROOT / "data" / "analyses"


def atomic_json(path:Path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value,allow_nan=False),encoding="utf-8")
    # Windows readers and file indexers can briefly deny replacement while a
    # status snapshot is open. Keep the atomic protocol and retry sharing locks.
    for attempt in range(12):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt==11:
                raise
            time.sleep(min(.25,.01*2**attempt))


def analysis_worker(job_id,scenario_data,cancel,root=None):
    from .dynamics import Cancelled
    from .environment import EnvironmentError
    from .optimization import optimize
    directory=Path(root or JOBS)/job_id
    started=time.time()
    def progress(value):
        atomic_json(directory/"status.json",{"id":job_id,"state":"running","started_at":started,
                                              "elapsed_s":time.time()-started,**value})
    try:
        result=optimize(Scenario.model_validate(scenario_data),cancel.is_set,progress)
        atomic_json(directory/"result.json",result)
        atomic_json(directory/"status.json",{"id":job_id,"state":"completed","started_at":started,
                    "elapsed_s":time.time()-started,"phase":"Analysis complete","outcome":result["status"]})
    except Cancelled:
        atomic_json(directory/"status.json",{"id":job_id,"state":"cancelled","started_at":started,
                    "elapsed_s":time.time()-started,"phase":"Cancelled","outcome":"Cancelled"})
    except EnvironmentError as exc:
        atomic_json(directory/"status.json",{"id":job_id,"state":"failed","started_at":started,
                    "elapsed_s":time.time()-started,"phase":"Environment data unavailable",
                    "outcome":"Environment Error","error":str(exc)})
    except Exception as exc:
        atomic_json(directory/"status.json",{"id":job_id,"state":"failed","started_at":started,
                    "elapsed_s":time.time()-started,"phase":"Analysis failed","outcome":"Numerical Failure",
                    "error":str(exc) or type(exc).__name__})


class JobManager:
    def __init__(self,root=None):
        self.root=Path(root or os.environ.get('ATMOSPHERE_ANALYSES_DIR') or JOBS)
        self.context=mp.get_context("spawn")
        self.jobs={}
        self.lock=threading.RLock()

    def start(self,scenario:Scenario):
        with self.lock:
            self.reap()
            if any(p.is_alive() for p,e in self.jobs.values()):
                raise ValueError("Another analysis is running. Cancel or wait for it before starting a new one.")
            job_id=uuid4().hex
            directory=self.root/job_id
            atomic_json(directory/'scenario.json',scenario.model_dump(mode='json'))
            atomic_json(directory/"status.json",{"id":job_id,"state":"queued","started_at":time.time(),"phase":"Queued","elapsed_s":0})
            cancel=self.context.Event()
            process=self.context.Process(target=analysis_worker,args=(job_id,scenario.model_dump(mode="json"),cancel,str(self.root)),daemon=True)
            process.start()
            self.jobs[job_id]=(process,cancel)
            return {"id":job_id,"state":"queued"}

    def reap(self):
        with self.lock:
            self._reap()

    def _reap(self):
        for job_id,(process,_) in list(self.jobs.items()):
            if not process.is_alive():
                process.join(timeout=0)
                path=self.root/job_id/"status.json"
                if path.exists():
                    state=json.loads(path.read_text())["state"]
                    if state in ("running","queued"):
                        atomic_json(path,{"id":job_id,"state":"failed","phase":"Analysis process stopped",
                                          "outcome":"Numerical Failure","error":"The analysis process exited unexpectedly."})
                del self.jobs[job_id]

    def status(self,job_id):
        with self.lock:
            return self._status(job_id)

    def _status(self,job_id):
        self._reap()
        path=self.path(job_id)/"status.json"
        if not path.exists():
            raise FileNotFoundError("Analysis not found.")
        value=json.loads(path.read_text(encoding="utf-8"))
        if value["state"]=="running":
            value["elapsed_s"]=time.time()-value["started_at"]
        # Recover unfinished jobs after an application restart.
        if value["state"] in ("running","queued") and job_id not in self.jobs:
            value.update(state="failed",phase="Interrupted by application restart",error="This analysis was interrupted. Run it again.")
        return value

    def cancel(self,job_id):
        with self.lock:
            if job_id in self.jobs:
                self.jobs[job_id][1].set()
            return self.status(job_id)

    def path(self,job_id):
        if len(job_id)!=32 or any(c not in "0123456789abcdef" for c in job_id):
            raise FileNotFoundError("Analysis not found.")
        return self.root/job_id

    def close(self):
        with self.lock:
            for process,cancel in self.jobs.values():
                cancel.set()
                process.join(timeout=3)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2)
            self._reap()


manager=JobManager()
