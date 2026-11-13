param(
    [string]$ConfigPath = "/workspace/data/handeye_1031_right12/cal_data.yaml"
)

$ErrorActionPreference = 'Stop'

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "  Starting Hand-Eye Calibration GUI in Docker '1030eyehand'" -ForegroundColor Cyan
Write-Host "  Auto-loading config: $ConfigPath" -ForegroundColor Yellow
Write-Host "==========================================================" -ForegroundColor Cyan

# Ensure container is started
$state = docker inspect 1030eyehand --format '{{.State.Running}}' 2>$null
if ($state -ne "true") {
    Write-Host "Starting container 1030eyehand..." -ForegroundColor Green
    docker start 1030eyehand | Out-Null
}

$cmd = "source /opt/industrial_calibration/setup.bash && export LD_LIBRARY_PATH=/opt/calibration-local/lib:`$LD_LIBRARY_PATH && /opt/calibration-local/bin/industrial_calibration_extrinsic_hand_eye_calibration_app $ConfigPath $ConfigPath"
docker exec -d 1030eyehand /bin/bash -lc $cmd
Write-Host "GUI App launched in WSLg! Please check your desktop." -ForegroundColor Green
