import time
import json
import threading

from atmosphere.jobs import JobManager
from atmosphere.models import default_scenario


def wait_terminal(manager,job_id,timeout=25):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        status=manager.status(job_id)
        if status['state'] in ('completed','failed','cancelled'):
            return status
        time.sleep(.15)
    raise AssertionError('Analysis worker did not finish in time')


def test_worker_cancellation(tmp_path):
    manager=JobManager(tmp_path)
    scenario=default_scenario();scenario.environment.model='standard'
    try:
        job=manager.start(scenario)
        manager.cancel(job['id'])
        assert wait_terminal(manager,job['id'])['state']=='cancelled'
    finally:
        manager.close()


def test_worker_completes_and_persists_no_liftoff_result(tmp_path):
    import json
    manager=JobManager(tmp_path)
    scenario=default_scenario();scenario.environment.model='standard'
    scenario.vehicle.stages[0].vacuum_thrust_n=1000
    scenario.constraints.solver_budget_s=5
    try:
        job=manager.start(scenario)
        assert wait_terminal(manager,job['id'])['state']=='completed'
        result=json.loads((manager.path(job['id'])/'result.json').read_text())
        assert result['status']=='No Liftoff'
        assert result['metadata']['verified_with_strict_integration']
    finally:
        manager.close()


def test_missing_environment_data_is_not_a_numerical_failure(tmp_path,monkeypatch):
    from atmosphere.jobs import analysis_worker
    from atmosphere.environment import EnvironmentError
    from atmosphere import optimization
    def unavailable(*args,**kwargs):
        raise EnvironmentError('Solar data unavailable. Select nominal conditions or enter manual values.')
    monkeypatch.setattr(optimization,'optimize',unavailable)
    job_id='e'*32
    analysis_worker(job_id,default_scenario().model_dump(mode='json'),threading.Event(),tmp_path)
    status=json.loads((tmp_path/job_id/'status.json').read_text())
    assert status['state']=='failed'
    assert status['outcome']=='Environment Error'
    assert 'nominal' in status['error']
    assert not (tmp_path/job_id/'result.json').exists()


def test_restart_recovers_interrupted_isolated_job(tmp_path):
    from atmosphere.jobs import atomic_json
    job_id='a'*32
    atomic_json(tmp_path/job_id/'status.json',{'id':job_id,'state':'running','started_at':time.time()})
    manager=JobManager(tmp_path)
    assert manager.status(job_id)['state']=='failed'
    assert 'interrupted' in manager.status(job_id)['error']
