# Read only Windows PnP identity information. Does not open audio or serial streams.
$ErrorActionPreference = 'Stop'
$observedDevices = @(Get-PnpDevice -PresentOnly | Where-Object {
    $_.FriendlyName -match 'MicArray|MA-USB\s*8|Maix.Audio' -or
    $_.InstanceId -match 'VID_359F&PID_3400'
} | Select-Object Status,Class,FriendlyName,InstanceId)
$report = [ordered]@{
    observed_at = (Get-Date).ToString('o')
    detection_basis = 'Manufacturer documented VID_359F&PID_3400 and MicArray/MA-USB8/Maix-Audio names'
    matching_device_count = $observedDevices.Count
    matching_devices = $observedDevices
    matching_device_present = ($observedDevices.Count -gt 0)
    audio_stream_tested = $false
    serial_stream_tested = $false
    firmware_modified = $false
    note = 'No match means no documented identity was observed at this time; it does not establish a hardware fault.'
}
$reportPath = Join-Path $PSScriptRoot 'mausb8_usb_snapshot.json'
[IO.File]::WriteAllText($reportPath, ($report | ConvertTo-Json -Depth 5), [Text.UTF8Encoding]::new($false))
$report | ConvertTo-Json -Depth 5
