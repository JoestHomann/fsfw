# USBtoWSL.ps1
# --- self-elevate ---
$curr=[Security.Principal.WindowsIdentity]::GetCurrent()
$principal=New-Object Security.Principal.WindowsPrincipal($curr)
if(-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)){
  Write-Host "Re-launching with Administrator privileges..."
  Start-Process powershell.exe -Verb RunAs -ArgumentList `
    "-NoProfile","-ExecutionPolicy","Bypass","-File","`"$PSCommandPath`""
  exit
}

$BusId  = '1-6'
$Distro = 'ArchLinux'      # or "Arch Linux" etc.

Write-Host "Unbinding USB device from Windows..."
usbipd unbind --busid $BusId
Start-Sleep 1

Write-Host "Binding USB device for WSL..."
usbipd bind --busid $BusId
Start-Sleep 1

Write-Host "Attaching USB device to WSL ($Distro)..."
usbipd attach --busid $BusId --wsl $Distro
# (Optional) persist re-attaching when unplugged/replugged:
#usbipd attach --auto-attach --busid $BusId --wsl $Distro

Write-Host "Done. Device should now be available in WSL."
