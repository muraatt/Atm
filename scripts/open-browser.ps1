for ($attempt=0; $attempt -lt 30; $attempt++) {
    try {
        $health = Invoke-RestMethod -Uri 'http://127.0.0.1:8000/api/health' -TimeoutSec 1
        if ($health.status -eq 'ok') { Start-Process 'http://127.0.0.1:8000'; exit 0 }
    } catch { }
    Start-Sleep -Seconds 1
}
