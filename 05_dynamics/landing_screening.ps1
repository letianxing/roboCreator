# Simplified vertical landing energy screen, not full flip dynamics.
# h: COM descent from apex to initial contact; s: COM braking displacement.
$gravity = 9.81
$results = @(foreach ($descent in @(0.05, 0.10, 0.15)) {
    foreach ($braking in @(0.01, 0.03, 0.05)) {
        [pscustomobject]@{
            com_descent_m = $descent
            braking_displacement_m = $braking
            contact_speed_mps = [math]::Round([math]::Sqrt(2 * $gravity * $descent), 3)
            displacement_average_force_over_weight = [math]::Round(1 + $descent / $braking, 2)
        }
    }
})
$results | ConvertTo-Json
