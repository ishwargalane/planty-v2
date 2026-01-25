    # C:\Scripts\attach-usb-to-wsl.ps1

    param(
        [string]$DeviceName = "Silicon Labs",  # Change to your device
        [int]$MaxRetries = 5,
        [int]$RetryDelay = 10
    )

    function Attach-USBDevice {
        param([string]$BusId)
        
        try {
            usbipd attach --wsl --busid $BusId 2>&1 | Out-Null
            return $?
        } catch {
            return $false
        }
    }

    # Wait for WSL to be ready
    $retryCount = 0
    while ($retryCount -lt $MaxRetries) {
        try {
            # Get running distributions
            $wslList = wsl --list --running 2>&1 | Out-String
            
            # Check if any distribution is running (not just the header)
            $lines = $wslList -split "`n" | Where-Object { $_.Trim() -ne "" }
            
            # check if lines contain "Ubuntu" or any other distro name if needed
            $distroFound = $false
            foreach ($line in $lines) {
                if ($line -match "Ubuntu|Debian|Kali") {
                    $distroFound = $true
                    Write-Host "WSL is running"
                    break
                }
            }

            # If more than just the header line exists, WSL is running
            if ($distroFound) {
                break
            }
        } catch {
            Write-Host "Error checking WSL status: $_"
        }
        Write-Host "Waiting for WSL... ($retryCount/$MaxRetries)"
        Start-Sleep -Seconds $RetryDelay
        $retryCount++
    }

    if ($retryCount -eq $MaxRetries) {
        Write-Host "WSL did not start in time. Exiting."
        exit 1
    }

    # Find and attach device
    $retryCount = 0
    while ($retryCount -lt $MaxRetries) {
        $deviceList = usbipd list 2>&1
        $device = $deviceList | Select-String -Pattern $DeviceName
        
        if ($device) {
            $busid = ($device -split '\s+')[0]
            
            # Check if already attached
            if ($device -match "Attached") {
                Write-Host "Device already attached: $busid"
                exit 0
            }
            
            Write-Host "Attaching device $busid..."
            if (Attach-USBDevice -BusId $busid) {
                Write-Host "Successfully attached device $busid"
                exit 0
            }
        }
        
        Write-Host "Device not found or attachment failed. Retry $retryCount/$MaxRetries"
        Start-Sleep -Seconds $RetryDelay
        $retryCount++
    }

    Write-Host "Failed to attach USB device after $MaxRetries attempts"
    exit 1