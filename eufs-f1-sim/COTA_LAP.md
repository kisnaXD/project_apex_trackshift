# COTA lap runner

With the existing `eufs-f1-sim` container running, launch one lap from the
host with:

```bash
./scripts/drive_cota_lap.sh
```

Extra runner options are passed through, for example:

```bash
./scripts/drive_cota_lap.sh --max-wall 1800 --verify-acceleration
```

The launcher copies the current `cota_path.py` and `cota_lap.py` into the
running container, rejects a second lap process, and writes `/tmp/cota_lap_run.json`
plus `/tmp/cota_lap_run.csv` inside the container. Stop an active lap from a
second host terminal with:

```bash
docker exec eufs-f1-sim pkill -INT -f '[p]ython3 /tmp/cota_lap.py'
```

The runner performs its bounded brake/final-status path. The container
lifecycle remains controlled by the normal stack scripts.

The forward-drive and wheel-rolling repairs used by this runner have been
user-confirmed in the current stack. Native full-lap validation is accepted
for the measured run in `.tmp/lap_validation/cota_lap_full.json`:
`5513.0986 m` completed in `545.687 s`, with final longitudinal velocity
`0.0 m/s`. Independent footprint validation found zero violations across all
`4808` trace samples using separate left/right boundary-loop ray casts; see
`.tmp/lap_validation/live_trace_footprint_summary.json`. Offline controller
and echo-ownership tests pass, and the runner compiles successfully.
