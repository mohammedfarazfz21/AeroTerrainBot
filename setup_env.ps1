$ws = "c:\Users\DELL\Desktop\Anitgravity\AeroTerraBot\aeroterrabot_ws"
$src = "$ws\src"

$packages = @(
    "aeroterrabot_interfaces",
    "aeroterrabot_description",
    "aeroterrabot_hardware",
    "aeroterrabot_control",
    "aeroterrabot_gazebo",
    "aeroterrabot_navigation",
    "aeroterrabot_bringup"
)

foreach ($pkg in $packages) {
    New-Item -ItemType Directory -Force -Path "$src\$pkg" | Out-Null
}

New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_interfaces\msg" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_description\urdf" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_description\meshes" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_description\rviz" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_hardware\include\aeroterrabot_hardware" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_hardware\src" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_control\config" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_gazebo\worlds" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_gazebo\launch" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_navigation\config" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_navigation\launch" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_bringup\launch" | Out-Null
New-Item -ItemType Directory -Force -Path "$src\aeroterrabot_bringup\config" | Out-Null

Write-Host "Basic ROS2 Workspace generated"
