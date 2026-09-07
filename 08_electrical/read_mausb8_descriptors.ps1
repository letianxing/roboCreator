# Microsoft USB hub GET_DESCRIPTOR requests only; no configuration, reset or firmware writes.
$ErrorActionPreference = 'Stop'
if (-not ('MaUsbDescriptorReader' -as [type])) {
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
using System.ComponentModel;
using Microsoft.Win32.SafeHandles;
public static class MaUsbDescriptorReader {
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern SafeFileHandle CreateFile(string n,uint access,uint share,IntPtr sa,uint creation,uint flags,IntPtr template);
  [DllImport("kernel32.dll", SetLastError=true)]
  static extern bool DeviceIoControl(SafeFileHandle h,uint code,byte[] input,uint insize,byte[] output,uint outsize,out uint returned,IntPtr ov);
  public static byte[] Read(string hub,uint port,byte type,ushort length) {
    using(SafeFileHandle h=CreateFile(hub,0x40000000,3,IntPtr.Zero,3,0,IntPtr.Zero)) {
      if(h.IsInvalid) throw new Win32Exception(Marshal.GetLastWin32Error(),"Open USB hub for descriptor query");
      byte[] buffer=new byte[12+length];Array.Copy(BitConverter.GetBytes(port),0,buffer,0,4);
      buffer[4]=0x80;buffer[5]=6;buffer[6]=0;buffer[7]=type;
      Array.Copy(BitConverter.GetBytes(length),0,buffer,10,2);uint returned;
      if(!DeviceIoControl(h,0x220410,buffer,(uint)buffer.Length,buffer,(uint)buffer.Length,out returned,IntPtr.Zero))
        throw new Win32Exception(Marshal.GetLastWin32Error(),"USB GET_DESCRIPTOR");
      if(returned<12) throw new Exception("Descriptor request returned no payload");
      int count=Math.Min((int)returned-12,(int)length);byte[] data=new byte[count];Array.Copy(buffer,12,data,0,count);return data;
    }
  }
}
'@
}
$roots = @(Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match '^USB\\VID_359F&PID_3400\\' })
if ($roots.Count -ne 1) { throw 'Expected exactly one connected MA-USB8 composite device' }
$root = $roots[0]
$props = @{}
Get-PnpDeviceProperty -InstanceId $root.InstanceId -KeyName 'DEVPKEY_Device_Parent','DEVPKEY_Device_LocationInfo','DEVPKEY_Device_Address' | ForEach-Object { $props[$_.KeyName]=$_.Data }
$location = [string]$props['DEVPKEY_Device_LocationInfo']
if ($location -notmatch 'Port_#(\d+)') { throw 'No explicit USB port index in location information' }
$portIndex = [uint32]$Matches[1]
if ($portIndex -ne [uint32]$props['DEVPKEY_Device_Address']) { throw 'Port location and address disagree; refusing to query an ambiguous port' }
$hub = '\\?\' + ([string]$props['DEVPKEY_Device_Parent']).Replace('\','#') + '#{f18a0e88-c30c-11d0-8815-00a0c906bed8}'
$dest = Join-Path $PSScriptRoot ('usb_checks\descriptors_' + (Get-Date).ToString('yyyyMMdd_HHmmss_fff'))
New-Item -ItemType Directory -Path $dest -Force | Out-Null
try {
  [byte[]]$device = [MaUsbDescriptorReader]::Read($hub,$portIndex,1,18)
  if ($device.Length -ne 18 -or [BitConverter]::ToUInt16($device,8) -ne 0x359f -or [BitConverter]::ToUInt16($device,10) -ne 0x3400) { throw 'Queried port identity does not match MA-USB8' }
  [byte[]]$head = [MaUsbDescriptorReader]::Read($hub,$portIndex,2,9)
  if ($head.Length -lt 9 -or $head[1] -ne 2) { throw 'Invalid configuration descriptor header' }
  $total = [BitConverter]::ToUInt16($head,2)
  if ($total -lt 9) { throw 'Invalid configuration length' }
  [byte[]]$config = [MaUsbDescriptorReader]::Read($hub,$portIndex,2,$total)
  if ($config.Length -ne $total) { throw 'Truncated configuration descriptor' }
  [IO.File]::WriteAllBytes((Join-Path $dest 'device.bin'),$device)
  [IO.File]::WriteAllBytes((Join-Path $dest 'configuration.bin'),$config)
  $interfaces=@();$current=$null;$offset=0
  while ($offset -lt $config.Length) {
    $length=[int]$config[$offset];$kind=[int]$config[$offset+1]
    if ($length -lt 2 -or $offset+$length -gt $config.Length) { throw 'Malformed descriptor length' }
    if ($kind -eq 4 -and $length -ge 9) {
      $current=[ordered]@{number=[int]$config[$offset+2];alternate=[int]$config[$offset+3];declared_endpoints=[int]$config[$offset+4];class=[int]$config[$offset+5];subclass=[int]$config[$offset+6];protocol=[int]$config[$offset+7];endpoints=@()}
      $interfaces += $current
    } elseif ($kind -eq 5 -and $length -ge 7 -and $null -ne $current) {
      $address=[int]$config[$offset+2];$attributes=[int]$config[$offset+3]
      $current.endpoints += [ordered]@{address=('0x{0:X2}' -f $address);direction=(@{0='OUT';128='IN'}[$address -band 128]);transfer_type=($attributes -band 3);sync_type=(($attributes -shr 2) -band 3);usage_type=(($attributes -shr 4) -band 3);max_packet_size=[int][BitConverter]::ToUInt16($config,$offset+4);interval=[int]$config[$offset+6]}
    }
    $offset += $length
  }
  $missing=@()
  foreach ($interface in $interfaces) {
    if ($interface.class -ne 1 -or $interface.subclass -ne 2) { continue }
    $asynchronousOut=@($interface.endpoints | Where-Object { $_.direction -eq 'OUT' -and $_.transfer_type -eq 1 -and $_.sync_type -eq 1 -and $_.usage_type -eq 0 })
    $feedback=@($interface.endpoints | Where-Object { $_.direction -eq 'IN' -and $_.transfer_type -eq 1 -and $_.usage_type -eq 1 })
    if ($asynchronousOut.Count -gt 0 -and $feedback.Count -eq 0) { $missing += [ordered]@{interface=$interface.number;alternate=$interface.alternate;async_out_addresses=@($asynchronousOut | ForEach-Object {$_.address})} }
  }
  $record=[ordered]@{read_succeeded=$true;instance_id=$root.InstanceId;port_index=$portIndex;configuration_bytes=$total;bcd_device=('0x{0:X4}' -f [BitConverter]::ToUInt16($device,12));interfaces=$interfaces;async_out_without_explicit_feedback=$missing;scope='Standard GET_DESCRIPTOR only; no audio/serial stream,device configuration or firmware modified'}
} catch {
  $record=[ordered]@{read_succeeded=$false;instance_id=$root.InstanceId;error=$_.Exception.Message;scope='Read-only descriptor query failed; driver logs remain available'}
}
$json=$record | ConvertTo-Json -Depth 8
[IO.File]::WriteAllText((Join-Path $dest 'summary.json'),$json,[Text.UTF8Encoding]::new($false))
Write-Output ('Saved: '+$dest)
Write-Output $json
