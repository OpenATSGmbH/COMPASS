# Probability of Detection in ATC Surveillance

This document summarises how Probability of Detection (PD) is defined and
computed for Air Traffic Control surveillance systems, and then shows the
COMPASS detection requirement implementation as a concrete example of one
of the established methods (the time difference method).

## 1. What PD Means

Two definitions of "probability of detection" are commonly encountered
in the surveillance domain. They look similar but answer different
questions.

### 1.1 Radar-engineering PD (per-scan / per-turn)

In classical radar usage, PD is the probability that a single scan or
antenna turn produces a detection of a target that should be detected
within the radar's coverage. It is a **per-scan** quantity - see for
example the Radartutorial entry referenced below - defined as the
ratio of detected targets to the targets that should have been
detected on that scan. A typical operational requirement is "≥ 90 %
PD".

This kind of PD is a property of the sensor itself and is not directly
observable from a recorded ASTERIX target report stream alone - the
ground truth of "what was actually present on the scan" must come from
elsewhere (a higher-quality reference). Evaluation tools such as
SASS-C and COMPASS therefore use the operational definition below,
which approximates per-scan PD by per-update-interval PD.

### 1.2 Operational PD ("Probability of Update")

For surveillance verification against operational requirements
(e.g. EUROCAE ED-117 for SMR/MLAT, EUROCAE ED-116, ICAO Annex 10), PD
is the **probability that the system delivers a target report when one
is expected**. EUROCONTROL's specification calls this "Probability of
Update of Horizontal Position" and the SASS-C verification tool
implements it as the principal performance indicator.

The general form is:

```
PD = (number of update intervals with detections)
     / (number of expected update intervals)
```

The "expected" denominator is derived from a reference
(an independent, higher-quality source - typically a multi-sensor tracker
output, test flight/drive or reconstructed reference from traffic of opportunity).
The period at which the system under test
is expected to update is the **update interval** - for an ASR it can be a
4–5 s scan period; for MLAT and ADS-B 1 s, etc.

EUROCONTROL ESASSP typically requires PD ≥ 97 % per measurement-interval
class. ICAO Annex 10 / EUROCAE thresholds apply per surveillance type.

## 2. How PD is Counted in Practice

There are two established ways to compute the operational PD from
recorded reference and test data: the **period-based method** and the
**time-difference method**. Which one is applicable depends on whether
the exact update-interval boundaries of the system under test are
recorded in the data.

### 2.1 Period-based method

Used when the exact start/end timestamps of each update interval are
known. In that case PD reduces to a simple per-interval question.

Sources of exact update-interval timing:

- **Radars** - the *north marker* messages (e.g. CAT034 / CAT002 sector
  crossing or north-crossing messages) mark the end of each antenna
  rotation. The interval `[north_n, north_{n+1}]` is one expected
  update interval of the radar.
- **MLAT, system trackers, fused trackers** - the cadence is artificial
  (no rotating antenna), but the data source can relay it through
  status messages such as **CAT019** (MLAT system status), **CAT021**
  status records, or **CAT010 / CAT023 "Start of Update Cycle"**.

When such cycle boundaries are present:

```
For each update interval [t_start, t_end]:
    if reference exists in [t_start, t_end] (the target's reference
    trajectory covers the interval):
        #EUI += 1
        if at least one test report exists in [t_start, t_end]:
            detected
        else:
            #MUI += 1
PD = (#EUI − #MUI) / #EUI
```

### 2.2 Time-difference method

Used when no exact cycle timing is recorded - for example when status
messages are missing, not transmitted by the source, or filtered out
upstream - only the configured nominal update interval is available.

In this case the **update interval of the sensor must be known and
supplied externally**, typically from the configuration of the data
source (e.g. a radar's antenna rotation period, an MLAT/tracker
configured cycle, an ADS-B nominal report rate). Without this value
the method cannot produce a result, since it is the only available
yardstick against which gaps are evaluated.

The reference trajectory for the target is first split into
**reference periods** (also called *legs*) - continuous time windows
during which the reference is tracking the target. Walking the
reference timestamps, consecutive samples are merged into the same
period as long as their spacing stays small relative to the reference
cadence (a typical rule of thumb: gap > ~2× the reference update
period starts a new period). The result is the set of windows during
which the system under test is *expected* to deliver reports. Edge
cases (minimum period length, exclusion of operator-flagged
intervals) are implementation choices - see §3.2 for the COMPASS
specifics.

The total reference coverage time is binned into **expected update
intervals** (#EUI), based on the configured update interval. The test
data is then walked and three kinds of gap are checked against each
reference period:

- from the period start to the first test report inside it,
- between consecutive test reports inside the period,
- from the last test report inside the period to the period end.

When a gap exceeds the update interval,
`floor(gap / update_interval)` missed update intervals (#MUI) are
attributed to that gap. A reference period with no test report at all
contributes its full duration `(period.end − period.begin)` as a gap.
This edge handling is essential - without it a target with no test
data at all would yield zero #MUI, which is obviously wrong.

```
PD = (#EUI − #MUI) / #EUI
```

This is the method used by SASS-C (probability-of-update class of
indicators) and by the COMPASS detection requirement described in §3.

### 2.3 Why Both Methods Have Their Advantages

Neither method dominates the other in practice; they answer the same
question but make different assumptions about the data.

**The period-based method is preferable when cycle messages are
available**, because:

- It matches the system's **real cadence**, not a configured nominal
  one - a radar that runs slightly fast or slow, or a tracker whose
  cycle drifts, is still measured against its actual update intervals.
- A single test report inside an interval is enough to count as a
  detection. There is no ambiguity about how many "missed slots" sit
  inside a long gap; the slots are explicitly defined by the source.
- Coverage-boundary handling is exact: an interval is either entirely
  inside a reference period or it is not. No special-case logic for
  the start or end of a coverage period.
- Edge cases like target re-entry into coverage do not require
  configurable filters - if no interval is expected, no miss can be
  recorded.

**The time-difference method is preferable when cycle messages are
not available, or when the system's true cadence is irregular**,
because:

- It only needs the test report stream and a configured nominal update
  interval - no auxiliary status messages have to be present, decoded
  correctly, or trustworthy.
- It is robust to **non-uniform cadence**: ADS-B reports, fused
  trackers without start-of-cycle records, or recordings where status
  messages were filtered out upstream can still be evaluated.
- It tolerates small reference/test sample-rate mismatches - the
  metric is driven by the spacing of test reports, not by per-interval
  matching.
- It can model "soft" tolerances naturally (miss tolerance,
  min/max gap length), which lets the user separate genuine detection
  failures from coverage-edge artifacts without changing the data.

In short: the period-based method is more **truthful** when ground
truth on cycle boundaries exists; the time-difference method is more
**resilient** when it does not. Most evaluation frameworks support
both and select the one applicable per data source / surveillance
type.

### 2.4 Data Requirements

PD is generally computed **per target**. For each target there is a
timestamped test report stream and a timestamped reference trajectory;
both methods walk these two streams together. For the period-based
method the test data source additionally supplies status messages
from which the exact update-interval boundaries of the system under
test are derived - those intervals are sensor-wide and apply to every
target the sensor covers.

Both methods share a common minimum: a timestamped **reference
trajectory** (so reference periods can be derived) and the
timestamped **test target reports**. Beyond that, the two methods
diverge in what additional inputs they need.

| Input | Period-based | Time-difference |
|---|---|---|
| Timestamped reference trajectory | required | required |
| Test target reports (timestamped) | required | required |
| Exclusion windows | optional | optional |
| **Exact update-interval boundaries from the system under test** (north markers, CAT019 / CAT021 status, CAT010-CAT023 start-of-cycle) | **required** | not used |
| Configured nominal update interval | optional (only as a sanity cross-check against the recorded cycles) | **required** (sole source of cadence) |
| Miss tolerance / min-gap / max-gap parameters | not needed (intervals are crisply defined by the source) | typically used to filter coverage-edge artifacts |

Implications:

- If the cycle-boundary stream is **partially missing** (gaps in north
  markers, dropped status messages), the period-based method will
  under-count #EUI in the affected windows, biasing PD upward. In that
  situation the time-difference method is the safer fallback.
- If only the test stream is available and **no nominal update interval
  is known** (e.g. an unfamiliar source), neither method can produce
  PD without the user supplying a cadence.
- Reference quality matters identically for both methods: if the
  reference is sparse or has gaps in coverage, the corresponding
  windows are excluded from #EUI either way.

## 3. COMPASS Implementation (Example)

Files:

- [eval/requirement/detection/detection.h](detection.h)
- [eval/requirement/detection/detection.cpp](detection.cpp)
- User manual section: [doc/user_manual/eval/eval_req_det.tex](../../../../doc/user_manual/eval/eval_req_det.tex)

The COMPASS "Detection" requirement implements both methods. The
`pd_calculation_method` parameter selects which one applies:

- `time_difference` (default) - the walk described in this section.
- `status_message` - the period-based method of section 2.1, using the
  update cycles the test data source reports. `EvaluationManager`
  loads the start-of-update-cycle messages of the test data sources
  through `DBContentStatusInfo` (CAT019 message type 001 for MLAT
  systems, CAT010 message type 002 for SMRs), restricted to the test
  sources and the test line.
  `Detection::evaluateStatusCycles()` then groups
  `round(update_interval / median_cycle_length)` consecutive reported
  cycles into one expected period, so a requirement asking for
  detection within 2 s on a 1 s cycle source groups 2 cycles. A period
  counts as expected when it lies fully inside a reference period, and
  as a miss when the target has no test report inside it. Miss
  tolerance, minimum and maximum gap length and the stationary update
  interval do not apply, the period boundaries come from the source.
  Without cycles, with more than one active test data source, or with
  fewer than 2 cycles, the time-difference walk below applies instead.

Independent of that switch, `use_gap_count` selects what the walk
counts. It is described in section 3.6 and always uses the
time-difference walk, so a `status_message` setting on a gap count
instance has no effect.

The rest of this section describes the time-difference walk.

### 3.1 Inputs

- Reference chain (`target_data.refChain()`) - a high-quality reference
  trajectory.
- Test chain (`target_data.tstChain()`) - the system under test.
- Sector layer geometry, exclusion intervals.
- Configurable parameters:
  - `Update Interval [s]` - expected cadence of the system under test.
  - `Probability` (threshold) and check type (`>=`).
  - `Use Miss Tolerance` / `Miss Tolerance [s]` - slack added to the
    update interval before a gap counts as a miss.
  - `Use Minimum Gap Length [s]` - gaps below this are ignored.
  - `Use Maximum Gap Length [s]` - gaps above this are ignored
    (avoids counting target-out-of-coverage periods).
  - `Use Gap Count` - count gaps over test reports instead of missed
    update intervals over expected update intervals, see section 3.6.
  - `Hold for any Target` - threshold must hold per individual target,
    not only at the sector aggregate.

### 3.2 Phase 1 - Build reference periods

Walk all reference timestamps. At each timestamp the sample is
considered *inside* if (a) the reference position lies inside the
sector polygon (per-sample point-in-polygon - no interpolation or
boundary-crossing computation between samples) and (b) the timestamp
is not in an exclusion interval. While samples stay inside,
consecutive timestamps are merged into the current period as long as
their spacing stays below the maximum reference time difference
(configurable per evaluation standard); a larger spacing, an excluded
sample, or a sample outside the sector closes the period, and a later
inside sample starts a new one. Periods shorter than one second
(hardcoded threshold) are discarded. The result is a set of windows
during which the test system is *expected* to detect.

### 3.3 Phase 2 - Count expected update intervals (#EUI)

#EUI is the total reference-period duration inside the sector divided
by the configured update interval, **floored to an integer**.

### 3.4 Phase 3 - Walk test timestamps and count missed update intervals (#MUI)

The algorithm walks the **test report stream** (not the periods),
keeping per-period state for the last test timestamp seen inside each
period. For every test report:

- Any reference periods that lie strictly before the test report's
  timestamp are finalized - closed against their end. The gap used
  for each such period is `period.end − last_test_ts_in_period`, or
  the full period duration `period.end − period.begin` if no test
  report was ever seen inside it.
- If the test report falls outside every reference period, it is
  skipped (test reports outside reference coverage do not contribute).
- If it falls inside a reference period and is the first test report
  in that period, the gap is `test_ts − period.begin`.
- If it is a subsequent test report in the same period, the gap is
  `test_ts − previous_test_ts_in_period`.

A test report inside a reference period is **additionally skipped**
if its timestamp falls in an exclusion interval, or if the
reference-interpolated position at that timestamp is outside the
sector. Two subtleties matter here:

- The sector check uses the **reference's position interpolated to
  the test timestamp**, not the test report's own reported position
  - easy to misread when reimplementing, and a common source of bugs.
- After such a skip, the next valid in-sector test report in the
  same period is treated as a fresh entry: no gap is computed for it
  and the in-period state simply advances to the new timestamp.
  Within-period outside-and-back excursions are therefore effectively
  forgiven. If the target stays outside until the period ends, the
  closure gap is measured from the last in-sector test report to
  `period.end` and may register as a miss (subject to the
  max-gap-length filter).

After the loop, any reference periods still open are finalized the
same way as the first bullet. If there were no test reports at all,
every reference period contributes its full duration as one gap.

For each gap the **miss test** is applied:

1. If the **raw** gap is below the configured minimum gap length, it
   is not a miss.
2. If the **raw** gap is above the configured maximum gap length, it
   is not a miss (treated as out-of-coverage).
3. Otherwise, if miss tolerance is enabled, subtract the tolerance
   from the gap. The gap is a miss iff the adjusted gap exceeds the
   update interval.

The order matters. The standards state the gap length thresholds on
the measured gap, so the tolerance must not shift them: with a 3 s
minimum gap length and a 0.5 s tolerance, gaps of 3 s and more count,
not gaps of 3.5 s and more. The tolerance covers cadence jitter of the
system under test and therefore belongs to the update interval test
only.

When a gap is classified as a miss, the number of missed UIs added to
#MUI is `floor(adjusted_gap / update_interval)`. The miss-count step
is only applied to gaps that pass the miss test.

### 3.5 Result

Per target:

```
PD = (#EUI − #MUI) / #EUI
```

A per-target result is produced; sector-aggregate and per-target
tables are reported (see the user manual section).

**Sector aggregation.** Sector PD is computed by summing across
targets, *not* by averaging per-target PDs:

```
PD_sector = (Σ #EUI − Σ #MUI) / Σ #EUI
```

Targets with more reference coverage therefore carry proportionally
more weight.

**Threshold check.** `PD >= configured probability`. With
`Hold for any Target` disabled (default), only the aggregate PD is
checked against the threshold. With it enabled, the requirement
passes only if **every individual target** meets the threshold; a
single failing target fails the requirement regardless of the
aggregate value.

### 3.6 Gap count mode

EUROCAE ED-117A section 6.4.8 and EUROCAE ED-87E section 5.3.14 state
their gap requirements as a **number of gaps over a number of target
reports**, not as missed time over expected time. `use_gap_count`
switches the walk to that form. Everything above stays as it is, only
the two sums change:

- Every gap that passes the miss test of section 3.4 adds **1**,
  independent of its length and of the update interval. A 3 s gap and
  a 30 s gap count the same. The update interval therefore no longer
  influences the result, except through the miss test itself.
- The expected total is the **number of test reports the walk
  accepted**: reports inside a reference period, inside the sector,
  and not excluded. Reference duration and update interval do not
  enter it.

Leading gaps, trailing gaps and reference periods without any test
report count as one gap each, for the same reason a mid-period gap
does: the reference shows the target existed and the system under test
reported nothing.

Two consequences follow, and both are handled in the result classes:

- More gaps than accepted reports are possible, so the check that
  misses never exceed the expected total is off in this mode.
- A target whose reports all fall outside the sector has no accepted
  report and therefore no probability of its own. Its gaps still
  belong to the sector, so the sector sum takes them even though the
  per-target result carries no value.

The report labels follow the mode: `#Reports` and `#Gaps` instead of
`#EUIs` and `#MUIs`.

## 4. Differences vs the General Definitions

The COMPASS implementation is faithful to the operational
"probability-of-update" definition (§1.2). It uses the time-difference
method (§2.2) by default and the period-based method (§2.1) when
`pd_calculation_method` is `status_message` and the test data source
reports its update cycles. The points below describe the
time-difference walk, which is the more intricate of the two:

- **Reference time-based denominator.** #EUI is derived from reference
  coverage time, not from a count of reference reports. The reference
  sample rate therefore does not directly inflate or deflate PD as long
  as it is dense enough to define continuous time periods. In gap count
  mode (§3.6) the denominator is a report count instead, which is what
  the gap requirements of ED-117A and ED-87E ask for.
- **No per-report association.** A test report is not paired against an
  individual reference report. Misses are inferred from gaps; matched
  detections are not labeled (only "no miss in this gap").
- **Min/Max gap length filtering.** These are practical filters not
  always present in textbook formulations - they let the user exclude
  short transients (small gaps) and structural exits/re-entries (large
  gaps) from the miss count.
- **Per-period boundary handling.** Gaps are evaluated separately at
  the start of each reference period, between consecutive test reports
  inside a period, and at the end of each period. Long gaps that span
  multiple periods are not summed across the boundary; each period is
  closed against its own end timestamp.
- **No per-scan PD.** COMPASS does not compute the classical per-scan
  radar PD of §1.1 directly; it approximates it via the
  per-update-interval probability of update.

## 5. Sources

- "Probability of Detection" (per-scan radar definition):
  <https://www.radartutorial.eu/01.basics/Probability%20of%20Detection.en.html>
- EUROCONTROL Specification for ATM Surveillance System Performance
  (ESASSP), Volumes 1 and 2:
  <https://www.eurocontrol.int/sites/default/files/2023-06/eurocontrol-spec-esassp-vol-1.pdf>
  <https://www.eurocontrol.int/sites/default/files/2023-06/eurocontrol-spec-esassp-vol-2.pdf>
- EUROCONTROL SASS-C - Surveillance Analysis Support System for ATC
  Centres:
  <https://www.eurocontrol.int/online-tool/surveillance-analysis-support-system-atc-centres>
- ICAO SURICG/6 IP/04, *Practical approach to assess the performance
  of surveillance systems*:
  <https://www.icao.int/sites/default/files/APAC/Meetings/2021/2021%20SURICG%206/4-Information%20Papers/IP04_ROK-AI.7-Practical-approach-to-assess-the-performance-of-surveillance-systems-r1.pdf>
- COMPASS user manual, "Detection" requirement:
  [doc/user_manual/eval/eval_req_det.tex](../../../../doc/user_manual/eval/eval_req_det.tex)
- COMPASS source: [detection.h](detection.h),
  [detection.cpp](detection.cpp)
