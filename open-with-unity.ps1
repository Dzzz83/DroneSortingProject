$unity = "D:\TeraBoxDownload\6000.6.0f1\Editor\Unity.exe"
$project = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not (Test-Path $unity)) {
    Write-Error "Unity editor not found at $unity"
    exit 1
}
Start-Process -FilePath $unity -ArgumentList "-projectPath", "`"$project`""
