param([Parameter(Mandatory=$true)][string]$Results)

$ErrorActionPreference = 'Stop'
$culture = [Globalization.CultureInfo]::InvariantCulture

function Value($inputValue) {
    if ([string]::IsNullOrWhiteSpace($inputValue)) { return [double]::NaN }
    return [double]::Parse($inputValue, $culture)
}

function Median($values) {
    $clean = @($values | Where-Object { -not [double]::IsNaN($_) } | Sort-Object)
    if ($clean.Count -eq 0) { return [double]::NaN }
    $middle = [int][Math]::Floor($clean.Count / 2)
    if ($clean.Count % 2) { return $clean[$middle] }
    return ($clean[$middle-1] + $clean[$middle]) / 2
}

function RunSummary($rows, $firstName, $secondName, $label) {
    $pairs = @($rows | Group-Object case | Where-Object Count -eq 2)
    if ($pairs.Count -eq 0) { return }
    $comparisons = foreach ($pair in $pairs) {
        $a = $pair.Group | Where-Object variant -eq $firstName
        $b = $pair.Group | Where-Object variant -eq $secondName
        [pscustomobject]@{
            N = [int]$a.customers
            FirstVerified = [int]$a.verified
            SecondVerified = [int]$b.verified
            FirstCertified = [int]($a.grb_status -eq '2')
            SecondCertified = [int]($b.grb_status -eq '2')
            FirstGap = Value $a.gap_pct
            SecondGap = Value $b.gap_pct
            GapDifference = (Value $b.gap_pct) - (Value $a.gap_pct)
            BoundDifference = (Value $b.lower_bound) - (Value $a.lower_bound)
            IncumbentDifference = (Value $b.objective) - (Value $a.objective)
            LpDifference = (Value $b.lp_relaxation_objective) - (Value $a.lp_relaxation_objective)
        }
    }
    foreach ($slice in @('all',16,25)) {
        $comparisonsOfSize = @($comparisons | Where-Object { $slice -eq 'all' -or $_.N -eq $slice })
        if ($comparisonsOfSize.Count -eq 0) { continue }
        [pscustomobject]@{
            phase=$label; n=$slice; pairs=$comparisonsOfSize.Count
            base_verified=($comparisonsOfSize.FirstVerified | Measure-Object -Sum).Sum
            full_verified=($comparisonsOfSize.SecondVerified | Measure-Object -Sum).Sum
            base_certified=($comparisonsOfSize.FirstCertified | Measure-Object -Sum).Sum
            full_certified=($comparisonsOfSize.SecondCertified | Measure-Object -Sum).Sum
            base_finite_gaps=@($comparisonsOfSize | Where-Object { -not [double]::IsNaN($_.FirstGap) }).Count
            full_finite_gaps=@($comparisonsOfSize | Where-Object { -not [double]::IsNaN($_.SecondGap) }).Count
            paired_finite_gaps=@($comparisonsOfSize | Where-Object { -not [double]::IsNaN($_.GapDifference) }).Count
            base_median_gap_pct=(Median $comparisonsOfSize.FirstGap)
            full_median_gap_pct=(Median $comparisonsOfSize.SecondGap)
            median_paired_gap_diff_pp=(Median $comparisonsOfSize.GapDifference)
            full_lower_gap=@($comparisonsOfSize | Where-Object GapDifference -LT -1e-8).Count
            full_higher_gap=@($comparisonsOfSize | Where-Object GapDifference -GT 1e-8).Count
            full_better_bound=@($comparisonsOfSize | Where-Object BoundDifference -GT 1e-8).Count
            full_worse_bound=@($comparisonsOfSize | Where-Object BoundDifference -LT -1e-8).Count
            full_better_incumbent=@($comparisonsOfSize | Where-Object IncumbentDifference -LT -1e-8).Count
            full_worse_incumbent=@($comparisonsOfSize | Where-Object IncumbentDifference -GT 1e-8).Count
            paired_optimal_lps=@($comparisonsOfSize | Where-Object { -not [double]::IsNaN($_.LpDifference) }).Count
            full_better_lp=@($comparisonsOfSize | Where-Object LpDifference -GT 1e-8).Count
            full_worse_lp=@($comparisonsOfSize | Where-Object LpDifference -LT -1e-8).Count
        }
    }
}

$warm = Import-Csv (Join-Path $Results 's1_warm_base_full.csv')
$coldPath = Join-Path $Results 's1_cold_base_full.csv'
$summary = @(
    RunSummary $warm 'base' 'full' 'warm'
    if (Test-Path -LiteralPath $coldPath) {
        RunSummary (Import-Csv $coldPath) 'base' 'full' 'cold'
    }
)
$summary | Export-Csv -LiteralPath (Join-Path $Results 's1_pair_summary.csv') -NoTypeInformation -Encoding UTF8
$summary | ConvertTo-Json -Depth 5

$familiesPath = Join-Path $Results 's1_cut_families.csv'
if (Test-Path -LiteralPath $familiesPath) {
    $families = Import-Csv $familiesPath
    $records = @{}
    foreach ($row in $families) { $records["$($row.case)/$($row.variant)"] = $row }
    foreach ($row in $warm) { $records["$($row.case)/$($row.variant)"] = $row }
    $familyComparisons = foreach ($group in ($families | Group-Object variant)) {
        $comparison = if ($group.Name.EndsWith('_only')) { 'base' } else { 'full' }
        foreach ($size in 16,25) {
            $selected = @($group.Group | Where-Object customers -eq "$size")
            $difference = foreach ($row in $selected) {
                $ref = $records["$($row.case)/$comparison"]
                [pscustomobject]@{
                    Incumbent = (Value $row.objective) - (Value $ref.objective)
                    Bound = (Value $row.lower_bound) - (Value $ref.lower_bound)
                    Gap = (Value $row.gap_pct) - (Value $ref.gap_pct)
                }
            }
            [pscustomobject]@{
                variant=$group.Name; reference=$comparison; n=$size; pairs=$selected.Count
                verified=@($selected | Where-Object verified -eq '1').Count
                certified=@($selected | Where-Object grb_status -eq '2').Count
                pairs_with_finite_bounds=@($difference | Where-Object { -not [double]::IsNaN($_.Bound) }).Count
                median_paired_bound_diff=(Median $difference.Bound)
                median_paired_incumbent_diff=(Median $difference.Incumbent)
                median_paired_gap_diff_pp=(Median $difference.Gap)
            }
        }
    }
    $familyComparisons | Export-Csv -LiteralPath (Join-Path $Results 's1_cut_summary.csv') -NoTypeInformation -Encoding UTF8
}

$lpPath = Join-Path $Results 's1_extended_lp.csv'
if (Test-Path -LiteralPath $lpPath) {
    $lp = Import-Csv $lpPath
    $lpSummary = foreach ($pair in ($lp | Group-Object case | Where-Object Count -eq 2)) {
        $base = $pair.Group | Where-Object variant -eq 'base'
        $full = $pair.Group | Where-Object variant -eq 'full'
        $delta = (Value $full.objective) - (Value $base.objective)
        [pscustomobject]@{
            case=$pair.Name; n=$base.customers; budget_s=$base.budget_s
            base_status=$base.status; full_status=$full.status
            base_objective=$base.objective; full_objective=$full.objective
            full_minus_base=if ([double]::IsNaN($delta)) { '' } else { $delta }
            base_elapsed_s=$base.elapsed_s; full_elapsed_s=$full.elapsed_s
        }
    }
    $lpSummary | Export-Csv -LiteralPath (Join-Path $Results 's1_extended_lp_pairs.csv') -NoTypeInformation -Encoding UTF8
}
