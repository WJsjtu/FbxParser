#Requires -Version "6.1"

$is64BitOperatingSystem = [System.Environment]::Is64BitOperatingSystem

Write-Host @"
IsWindows: $IsWindows
IsMacOS: $IsMacOS
Is64BitOperatingSystem: $is64BitOperatingSystem
Current working directory: $(Get-Location)
"@

function GetTriplet {
    if ($IsWindows) {
        return "x64-windows"
    } elseif ($IsMacOS) {
        return "x64-osx"
    } else {
        Write-Error "vcpkg is not available on target platform."
    }
}

function InstallFbxSdk {
    New-Item -ItemType Directory -Path "fbxsdk" | Out-Null
    if ($IsWindows) {
        $FBXSDK_2020_2_1_VS2017 = "https://damassets.autodesk.net/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_vs2017_win.exe"
        $FbxSdkWindowsInstaller = Join-Path "fbxsdk" "fbxsdk.exe"
        (New-Object System.Net.WebClient).DownloadFile($FBXSDK_2020_2_1_VS2017, $FbxSdkWindowsInstaller)
        $fbxSdkHome = [System.IO.Path]::Combine((Get-Location), "fbxsdk", "Home")
        $fbxSdkInstallDir = [System.IO.Path]::Combine((Get-Location), "fbxsdk", "Home", "2020.2.1")
        Start-Process -Wait -FilePath $FbxSdkWindowsInstaller -ArgumentList "/S","/D=$fbxSdkInstallDir"
    } elseif ($IsMacOS) {
        $FBXSDK_2020_2_1_CLANG = "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_clang_mac.pkg.tgz"
        $FBXSDK_2020_2_1_CLANG_VERSION = "2020.2.1"
        $fbxSdkMacOSTarball = Join-Path "fbxsdk" "fbxsdk.pkg.tgz"
        (New-Object System.Net.WebClient).DownloadFile($FBXSDK_2020_2_1_CLANG, $fbxSdkMacOSTarball)
        $fbxSdkMacOSPkgFileDir = "fbxsdk"
        & tar -zxvf $fbxSdkMacOSTarball -C $fbxSdkMacOSPkgFileDir | Out-Host
        $fbxSdkMacOSPkgFile = (Get-ChildItem -Path "$fbxSdkMacOSPkgFileDir/*" -Include "*.pkg").FullName
        Write-Host "FBX SDK MacOS pkg: $fbxSdkMacOSPkgFile"
        sudo installer -pkg $fbxSdkMacOSPkgFile -target / | Out-Host
        $fbxSdkHome = [System.IO.Path]::Combine((Get-Location), "fbxsdk", "Home")
        # Node gyp incorrectly handle spaces in path
        New-Item -ItemType SymbolicLink -Path "fbxsdk" -Name Home -Value "/Applications/Autodesk/FBX SDK/$FBXSDK_2020_2_1_CLANG_VERSION" | Out-Host
    } else {
        Write-Error "FBXSDK is not available on target platform."
    }
    return $fbxSdkHome
}

# https://stackoverflow.com/questions/54372601/running-git-clone-from-powershell-giving-errors-even-when-it-seems-to-work
$env:GIT_REDIRECT_STDERR = "2>&1"

$fbxSdkHome = InstallFbxSdk

Write-Host "FBX SDK: $fbxSdkHome"

"$fbxSdkHome" | Out-File "fbx.txt"
