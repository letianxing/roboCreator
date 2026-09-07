# Read-only identity/driver/event collection. No audio/serial stream or device settings opened.
param(
    [ValidateSet('unknown','driver_only_array_disconnected','array_connected')]
    [string]$ConnectionMode = 'unknown',
    [string]$DescriptorSummaryPath
)
$ErrorActionPreference = 'Stop'
$captureStamp = Get-Date
$captureDirectory = Join-Path $PSScriptRoot ('usb_checks\' + $captureStamp.ToString('yyyyMMdd_HHmmss_fff'))
New-Item -ItemType Directory -Path $captureDirectory -Force | Out-Null
$latestPath = Join-Path $PSScriptRoot 'mausb8_usb_snapshot.json'
if (Test-Path -LiteralPath $latestPath) {
    Copy-Item -LiteralPath $latestPath -Destination (Join-Path $captureDirectory 'previous_snapshot.json')
}
$keys = @('DEVPKEY_Device_ProblemCode','DEVPKEY_Device_ProblemStatus','DEVPKEY_Device_DriverProvider','DEVPKEY_Device_DriverVersion','DEVPKEY_Device_DriverInfPath','DEVPKEY_Device_Service','DEVPKEY_Device_BusReportedDeviceDesc','DEVPKEY_Device_Parent','DEVPKEY_Device_PDOName','DEVPKEY_Device_ContainerId')
$devices = @(Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match 'VID_359F&PID_3400' })
$details = @()
foreach ($device in $devices) {
    $props = @{}
    foreach ($prop in @(Get-PnpDeviceProperty -InstanceId $device.InstanceId -KeyName $keys -ErrorAction SilentlyContinue)) { $props[$prop.KeyName] = $prop.Data }
    $statusHex = $null
    if ($null -ne $props['DEVPKEY_Device_ProblemStatus']) { $statusHex = '0x{0:X8}' -f [uint32]$props['DEVPKEY_Device_ProblemStatus'] }
    $details += [pscustomobject]@{
        name = $device.FriendlyName; class = [string]$device.Class; status = [string]$device.Status
        instance_id = $device.InstanceId; problem_code = $props['DEVPKEY_Device_ProblemCode']; problem_status_hex = $statusHex
        driver_provider = $props['DEVPKEY_Device_DriverProvider']; driver_version = $props['DEVPKEY_Device_DriverVersion']
        driver_inf = $props['DEVPKEY_Device_DriverInfPath']; service = $props['DEVPKEY_Device_Service']
        bus_description = $props['DEVPKEY_Device_BusReportedDeviceDesc']; parent = $props['DEVPKEY_Device_Parent']
        pdo_name = $props['DEVPKEY_Device_PDOName']; container_id = [string]$props['DEVPKEY_Device_ContainerId']
    }
}
$pdoNames = @($details | ForEach-Object { $_.pdo_name } | Where-Object { $_ })
$pnpEvents = @(Get-WinEvent -FilterHashtable @{LogName='Microsoft-Windows-Kernel-PnP/Configuration';StartTime=$captureStamp.AddHours(-2)} -ErrorAction SilentlyContinue | Where-Object { $_.Message -match 'VID_359F&PID_3400' } | Select-Object -First 12 TimeCreated,Id,Message)
$audioFailureTimes = @($pnpEvents | Where-Object { $_.Id -eq 411 -and $_.Message -match 'usbaudio2' } | ForEach-Object {$_.TimeCreated})
$audioEvents = @(Get-WinEvent -FilterHashtable @{LogName='System';ProviderName='usbaudio2';StartTime=$captureStamp.AddHours(-2)} -ErrorAction SilentlyContinue | ForEach-Object {
    $eventMessage=$_.Message; $eventTime=$_.TimeCreated
    $pdoMatch=@($pdoNames | Where-Object { $eventMessage.Contains($_) }).Count -gt 0
    $timeMatch=@($audioFailureTimes | Where-Object { [Math]::Abs(($eventTime-$_).TotalSeconds) -le 2 }).Count -gt 0
    if ($pdoMatch -or $timeMatch) {
        [pscustomobject]@{TimeCreated=$eventTime.ToString('o');ProviderName=$_.ProviderName;Id=$_.Id;Message=$eventMessage;Association=$(if($pdoMatch){'PDO name match'}else{'Within2s of target audio start failure; audio log may use an FDO name'})}
    }
} | Select-Object -First 12)
$descriptorVerified = $false
if ($DescriptorSummaryPath) {
    $descriptorRecord=Get-Content -LiteralPath $DescriptorSummaryPath -Raw | ConvertFrom-Json
    $descriptorVerified=($descriptorRecord.read_succeeded -and $descriptorRecord.instance_id -in $details.instance_id)
}
$endpoints = @(Get-PnpDevice -PresentOnly -Class AudioEndpoint -ErrorAction SilentlyContinue | Where-Object { $_.FriendlyName -match 'MicArray|MA-USB\s*8|Maix.Audio' } | Select-Object Status,FriendlyName,InstanceId)
$record = [ordered]@{
    observed_at = $captureStamp.ToString('o'); matching_device_count = $details.Count; matching_device_present = ($details.Count -gt 0)
    devices = $details; matching_audio_endpoints = $endpoints; audio_driver_events = $audioEvents; pnp_events = $pnpEvents
    user_reported_connection = $ConnectionMode
    audio_stream_tested = $false; serial_stream_tested = $false; raw_usb_descriptors_dumped = $descriptorVerified
    descriptor_summary = $DescriptorSummaryPath
    firmware_modified = $false; driver_or_registry_modified = $false
    distinction = 'USB/COM enumeration and audio driver initialization are separate outcomes. USB serial string is not a verified firmware version.'
}
$json = $record | ConvertTo-Json -Depth 7
$encoding = [Text.UTF8Encoding]::new($false)
[IO.File]::WriteAllText((Join-Path $captureDirectory 'connection.json'), $json, $encoding)
[IO.File]::WriteAllText($latestPath, $json, $encoding)
Write-Output ('Saved: ' + $captureDirectory)
$details | Select-Object name,class,status,problem_code,problem_status_hex,pdo_name | ConvertTo-Json -Depth 3
$audioEvents | ConvertTo-Json -Depth 3
