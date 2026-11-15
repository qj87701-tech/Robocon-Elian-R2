param([string]$Suite = 'all')
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Force build/host | Out-Null
    $chassis = @('3_Module/3swerve_chassis/tsk_chassis.cpp','4_Task/tsk_motor_init/motor_init.cpp','3_Module/3swerve_chassis/tsk_chassis_resolve.cpp')
    $arm = @('3_Module/Arm/mod_arm_kinematics.cpp','3_Module/Arm/mod_arm_control.cpp')
    $debug = @('4_Task/tsk_debug/tsk_debug.cpp','2_Device/Plotter/Vofa/dvc_vofa.cpp','1_Middleware/Protocol/pro_upper/pro_upper.cpp')
    $suites = [ordered]@{
        arm = $arm + @('4_Task/tsk_motor_init/motor_init.cpp')
        chassis_swerve = @('3_Module/3swerve_chassis/tsk_chassis_resolve.cpp')
        chassis_task = $chassis
        u8_debug = $chassis
        motor_init = @('4_Task/tsk_motor_init/motor_init.cpp')
        bsp_key = @('2_Device/BSP/Key/bsp_key.cpp')
        pro_sbus = @('1_Middleware/Protocol/pro_sbus/pro_sbus.cpp')
        pro_upper = @('1_Middleware/Protocol/pro_upper/pro_upper.cpp')
        remote_receive_task = @('4_Task/tsk_remote/tsk_remote.cpp','1_Middleware/Protocol/pro_sbus/pro_sbus.cpp')
        vofa = @('2_Device/Plotter/Vofa/dvc_vofa.cpp')
        task_debug = $debug
        task_control = $chassis + $debug + @('4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp','4_Task/tsk_remote/tsk_remote.cpp','2_Device/BSP/Key/bsp_key.cpp','1_Middleware/Protocol/pro_sbus/pro_sbus.cpp','1_Middleware/Protocol/pro_upper/pro_upper.cpp','1_Middleware/Driver/UART/drv_uart.cpp')
        drv_uart = @('1_Middleware/Driver/UART/drv_uart.cpp')
        drv_can = @('1_Middleware/Driver/CAN/drv_can.cpp')
        motor_vesc = @('2_Device/Motor/Motor_VESC/dvc_motor_vesc.cpp')
        motor_dm_enable = @('2_Device/Motor/Motor_DM/dvc_motor_dm.cpp')
        imu_hi06 = @('2_Device/IMU/HI06/dvc_imu_hi06.cpp','1_Middleware/Hipnuc/hipnuc_dec.c','1_Middleware/Hipnuc/hipnuc_sample.c')
    }
    $selected = if ($Suite -eq 'all') { @($suites.Keys) } else { @($Suite.Split(',') | ForEach-Object { $_.Trim() }) }
    foreach ($name in $selected) { if (!$suites.Contains($name)) { throw "Unknown suite: $name" } }
    foreach ($name in $selected) {
        $includes = @('-Itests/host/algorithm_stubs','-Itests/host/stubs','-IUser_File','-ICore/Inc','-I.','-IUser_File/1_Middleware/Hipnuc')
        if ($name -in @('drv_can','motor_dm_enable')) { $includes = @('-Itests/host/motor_stubs') + $includes }
        if ($name -in @('motor_vesc','imu_hi06')) { $includes = @('-IUser_File') + $includes }
        $sources = @($suites[$name] | Select-Object -Unique | ForEach-Object { 'User_File/' + $_ })
        $exe = 'build/host/test_' + $name + '.exe'
        if ($name -eq 'u8_debug') {
            # Every selection runs with the firmware's normal ID1 mask, so ID2/3 cannot accidentally depend on it.
            $cases = @(
                @{IDs='{1U}';Mask=1}, @{IDs='{2U}';Mask=2}, @{IDs='{3U}';Mask=4},
                @{IDs='{1U,2U}';Mask=3}, @{IDs='{1U,3U}';Mask=5}, @{IDs='{2U,3U}';Mask=6}, @{IDs='{1U,2U,3U}';Mask=7},
                @{IDs='{}';Mask=0}, @{IDs='{0U}';Mask=0}, @{IDs='{4U}';Mask=0}, @{IDs='{1U,4U}';Mask=0}, @{IDs='{1U,1U}';Mask=0})
            foreach ($case in $cases) {
                & g++ -std=c++17 -Wall -Wextra -Werror "-DU8_TEST_MOTOR_IDS=$($case.IDs)" "-DTEST_EXPECTED_U8_MASK=$($case.Mask)" @includes ('tests/host/test_' + $name + '.cpp') @sources -o $exe
                if ($LASTEXITCODE -ne 0) { throw "Compile failed: U8 IDs $($case.IDs)" }
                & (Join-Path $repoRoot $exe)
                if ($LASTEXITCODE -ne 0) { throw "Test failed: U8 IDs $($case.IDs)" }
            }
            Write-Output "PASS $name ($($cases.Count) ID configurations)"
            continue
        }
        # Keep all-wheel regression coverage; task_control uses the firmware's default ID1 configuration.
        if ($name -eq 'chassis_task') {
            & g++ -std=c++17 -Wall -Wextra -Werror -DCHASSIS_SWERVE_ACTIVE_WHEEL_MASK=7U @includes ('tests/host/test_' + $name + '.cpp') @sources -o $exe
        } else {
            & g++ -std=c++17 -Wall -Wextra -Werror @includes ('tests/host/test_' + $name + '.cpp') @sources -o $exe
        }
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $name" }
        & (Join-Path $repoRoot $exe)
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $name" }
        Write-Output "PASS $name"
    }
    Write-Output "Result: $($selected.Count)/$($selected.Count) passed"
} finally { Pop-Location }
