# Restarts the ingest when InfluxDB stops receiving data.
#
# Why: after a LAN blip (switch restart, 2026-10-05 15:44) Telegraf inside WSL2
# kept a dead MQTT session and wrote nothing for 90 minutes, while the Pi kept
# publishing. Nothing in the stack notices that on its own.
#
# What it does, every run:
#   1. asks InfluxDB for the age of the newest "boiler" point
#   2. age <= MaxAgeMinutes            -> OK, reset the failure counter
#   3. first failure                   -> docker compose restart telegraf
#   4. second consecutive failure      -> wsl --shutdown, wait for Docker, compose up -d
# State lives in scripts/.watchdog-state, log in scripts/watchdog.log.
#
# Register (run once, from an elevated PowerShell in the project folder):
#   schtasks /Create /TN "HomeEnergy Telegraf watchdog" /SC MINUTE /MO 5 ^
#     /TR "wscript.exe \"C:\Users\janis\home-energy-monitor\scripts\telegraf-watchdog-hidden.vbs\"" ^
#     /RU "janis" /RL HIGHEST /F
#   (the .vbs wrapper starts PowerShell with a hidden window; calling
#    powershell.exe directly flashes a console every 5 min)
# Test by hand:
#   powershell -ExecutionPolicy Bypass -File .\scripts\telegraf-watchdog.ps1 -Verbose

[CmdletBinding()]
param(
    [int]$MaxAgeMinutes = 3,
    [string]$InfluxUrl = "http://localhost:8086"
)

$ErrorActionPreference = "Stop"
$root  = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$state = Join-Path $root "scripts\.watchdog-state"
$log   = Join-Path $root "scripts\watchdog.log"

function Log($msg) {
    $line = "{0:yyyy-MM-dd HH:mm:ss}  {1}" -f (Get-Date), $msg
    Add-Content -Path $log -Value $line
    Write-Verbose $line
}

# .env -> token and org
$envFile = Join-Path $root ".env"
$kv = @{}
Get-Content $envFile | Where-Object { $_ -match '^\s*([A-Z_]+)\s*=\s*(.*)$' } | ForEach-Object {
    $kv[$matches[1]] = $matches[2].Trim()
}
$token = $kv["INFLUX_TOKEN"]; $org = $kv["INFLUX_ORG"]; $bucket = $kv["INFLUX_BUCKET"]
if (-not $token -or -not $org -or -not $bucket) { Log "ERROR .env missing INFLUX_TOKEN/ORG/BUCKET"; exit 1 }

# Newest boiler point
$flux = "from(bucket: `"$bucket`") |> range(start: -6h) |> filter(fn: (r) => r._measurement == `"boiler`") |> group() |> last() |> keep(columns: [`"_time`"])"
$ageMin = $null
try {
    $resp = Invoke-WebRequest -Uri "$InfluxUrl/api/v2/query?org=$org" -Method Post -TimeoutSec 20 `
        -Headers @{ Authorization = "Token $token"; Accept = "application/csv" } `
        -ContentType "application/vnd.flux" -Body $flux -UseBasicParsing
    $row = ($resp.Content -split "`r?`n") | Where-Object { $_ -match '\d{4}-\d{2}-\d{2}T' } | Select-Object -First 1
    if ($row) {
        $ts = ($row -split ",")[3]
        $ageMin = ((Get-Date).ToUniversalTime() - [datetime]::Parse($ts).ToUniversalTime()).TotalMinutes
    } else {
        $ageMin = 999   # no point in the last 6 h
    }
} catch {
    Log "WARN InfluxDB query failed: $($_.Exception.Message)"
    $ageMin = 999
}

$fails = 0
if (Test-Path $state) { $fails = [int](Get-Content $state) }

if ($ageMin -le $MaxAgeMinutes) {
    if ($fails -gt 0) { Log ("OK data flowing again, newest point {0:N1} min old" -f $ageMin) }
    Set-Content $state "0"
    Write-Verbose ("OK newest point {0:N1} min old" -f $ageMin)
    exit 0
}

$fails++
Set-Content $state "$fails"
Log ("STALE newest boiler point {0:N1} min old (failure #{1})" -f $ageMin, $fails)

Push-Location $root
try {
    if ($fails -eq 1) {
        Log "ACTION docker compose restart telegraf"
        docker compose restart telegraf 2>&1 | ForEach-Object { Log "  $_" }
    } else {
        Log "ACTION wsl --shutdown, then docker compose up -d"
        wsl --shutdown 2>&1 | ForEach-Object { Log "  $_" }
        $deadline = (Get-Date).AddMinutes(4)
        do {
            Start-Sleep -Seconds 10
            $ok = $false
            try { docker info 2>$null | Out-Null; $ok = ($LASTEXITCODE -eq 0) } catch {}
        } until ($ok -or (Get-Date) -gt $deadline)
        if (-not $ok) { Log "ERROR Docker did not come back within 4 min" ; exit 1 }
        docker compose up -d 2>&1 | ForEach-Object { Log "  $_" }
        Set-Content $state "0"
    }
} finally { Pop-Location }
