from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

import fastf1
import numpy as np
import pandas as pd

from f1sim.data.schema import (
    ERSTelemetry,
    EgoTelemetry,
    KinematicsTelemetry,
    OpponentTelemetry,
    TelemetryFrame,
    TireChannel,
)


LOGGER = logging.getLogger(__name__)

SILVERSTONE_LENGTH_M = 5891.0
DRS_ACTIVE_VALUES = {10, 12, 14}

FALLBACK_TEAM_COLORS = {
    "Ferrari": "#e10600",
    "Red Bull Racing": "#1e41ff",
    "McLaren": "#ff8000",
    "Mercedes": "#00d2be",
    "Aston Martin": "#229971",
    "Alpine": "#0090ff",
    "Williams": "#005aff",
    "Haas F1 Team": "#b6babd",
    "Haas": "#b6babd",
    "Kick Sauber": "#52e252",
    "Sauber": "#52e252",
    "RB": "#6692ff",
    "Racing Bulls": "#6692ff",
}

DRIVER_TEAM_FALLBACKS = {
    "BEA": ("Haas F1 Team", "#b6babd"),
    "OCO": ("Haas F1 Team", "#b6babd"),
    "VER": ("Red Bull Racing", "#1e41ff"),
    "TSU": ("Red Bull Racing", "#1e41ff"),
    "LEC": ("Ferrari", "#e10600"),
    "HAM": ("Ferrari", "#e10600"),
    "NOR": ("McLaren", "#ff8000"),
    "PIA": ("McLaren", "#ff8000"),
    "RUS": ("Mercedes", "#00d2be"),
    "ANT": ("Mercedes", "#00d2be"),
    "ALO": ("Aston Martin", "#229971"),
    "STR": ("Aston Martin", "#229971"),
    "GAS": ("Alpine", "#0090ff"),
    "COL": ("Alpine", "#0090ff"),
    "ALB": ("Williams", "#005aff"),
    "SAI": ("Williams", "#005aff"),
    "HUL": ("Kick Sauber", "#52e252"),
    "BOR": ("Kick Sauber", "#52e252"),
}


class FastF1DataUnavailable(RuntimeError):
    """Raised when the requested FastF1 race cannot provide usable telemetry."""


@dataclass(frozen=True)
class DriverInfo:
    code: str
    driver_number: str
    full_name: str
    team_name: str
    team_color: str
    position: Optional[int] = None


def _series_seconds(values: pd.Series) -> pd.Series:
    if pd.api.types.is_timedelta64_dtype(values):
        return values.dt.total_seconds()
    if pd.api.types.is_datetime64_any_dtype(values):
        start = values.dropna().min()
        return (values - start).dt.total_seconds()
    return values.apply(_seconds_value)


def _seconds_value(value: Any) -> float:
    if value is None or pd.isna(value):
        return np.nan
    if isinstance(value, pd.Timedelta):
        return float(value.total_seconds())
    if hasattr(value, "total_seconds"):
        return float(value.total_seconds())
    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


def _clean_color(color: Any, fallback: str = "#ffffff") -> str:
    if color is None or pd.isna(color):
        return fallback
    value = str(color).strip()
    if not value:
        return fallback
    if not value.startswith("#"):
        value = f"#{value}"
    if len(value) != 7:
        return fallback
    return value.lower()


def _first_present(row: Any, keys: Iterable[str], default: Any = None) -> Any:
    for key in keys:
        try:
            value = row.get(key, default)
        except AttributeError:
            value = default
        if value is not None and not pd.isna(value):
            return value
    return default


class F1SessionReplayer:
    """
    Converts a FastF1 race session into deterministic replay frames.

    Streamlit can call get_next_frame() on every refresh and publish the returned
    TelemetryFrame through F1TelemetryHub. The replay is "live" in the dashboard
    sense: recorded FastF1 telemetry is played back frame-by-frame at app speed.
    """

    def __init__(
        self,
        year: int = 2026,
        circuit: str = "Silverstone",
        session_type: str = "R",
        ego_driver: str = "BEA",
        dt: float = 0.1,
        cache_dir: Optional[str] = None,
        track_length_m: float = SILVERSTONE_LENGTH_M,
    ):
        self.year = int(year)
        self.circuit = circuit
        self.session_type = session_type
        self.ego_driver = ego_driver.upper()
        self.dt = float(dt)
        self.track_length_m = float(track_length_m)
        self.cache_dir = cache_dir or os.getenv(
            "FASTF1_CACHE_DIR", "telemetry_logs/fastf1_cache"
        )

        os.makedirs(self.cache_dir, exist_ok=True)
        fastf1.Cache.enable_cache(self.cache_dir)

        self.session = fastf1.get_session(self.year, self.circuit, self.session_type)
        self._load_session()

        self.driver_info = self._build_driver_info()
        self.driver_telemetry: Dict[str, pd.DataFrame] = {}

        for code in self.driver_info:
            try:
                driver_frame = self._build_driver_frame(code)
            except Exception as exc:  # Keep one bad car from killing the field.
                LOGGER.warning("Could not build telemetry for %s: %s", code, exc)
                continue
            if not driver_frame.empty:
                self.driver_telemetry[code] = driver_frame

        if self.ego_driver not in self.driver_telemetry:
            available = ", ".join(sorted(self.driver_telemetry)) or "none"
            raise FastF1DataUnavailable(
                f"{self.ego_driver} telemetry is unavailable for "
                f"{self.year} {self.circuit} {self.session_type}. "
                f"Available drivers: {available}."
            )

        self.drivers = list(self.driver_telemetry)
        self.opponent_drivers = [d for d in self.drivers if d != self.ego_driver]
        self.track_map = self._build_track_map()

        ego_times = self.driver_telemetry[self.ego_driver]["time_s"].to_numpy(dtype=float)
        self.start_time_s = float(np.nanmin(ego_times))
        self.end_time_s = float(np.nanmax(ego_times))
        if not np.isfinite(self.start_time_s) or not np.isfinite(self.end_time_s):
            raise FastF1DataUnavailable("Ego telemetry has no valid session time axis.")

        self.time_steps = np.arange(self.start_time_s, self.end_time_s, self.dt)
        self.step_idx = 0

    @property
    def session_label(self) -> str:
        try:
            event_name = self.session.event.get("EventName", self.circuit)
        except Exception:
            event_name = self.circuit
        return f"{self.year} {event_name} {self.session_type}"

    def _load_session(self) -> None:
        try:
            self.session.load(telemetry=True, laps=True, weather=False, messages=False)
        except TypeError:
            self.session.load(telemetry=True, laps=True, weather=False)
        except Exception as exc:
            raise FastF1DataUnavailable(
                f"FastF1 could not load {self.year} {self.circuit} {self.session_type}: {exc}"
            ) from exc

        if getattr(self.session, "laps", None) is None or self.session.laps.empty:
            raise FastF1DataUnavailable("FastF1 returned no lap data for this session.")

    def _build_driver_info(self) -> Dict[str, DriverInfo]:
        info: Dict[str, DriverInfo] = {}

        results = getattr(self.session, "results", None)
        if results is not None and not results.empty:
            rows = [row for _, row in results.iterrows()]
        else:
            rows = []
            for driver_no in getattr(self.session, "drivers", []):
                try:
                    rows.append(self.session.get_driver(driver_no))
                except Exception:
                    continue

        for row in rows:
            code = str(_first_present(row, ["Abbreviation", "BroadcastName"], "")).upper()
            if not code:
                continue

            fallback_team, fallback_color = DRIVER_TEAM_FALLBACKS.get(
                code, ("Unknown", "#94a3b8")
            )
            team_name = str(_first_present(row, ["TeamName", "Team"], fallback_team))
            team_color = _clean_color(
                _first_present(row, ["TeamColor"], None),
                FALLBACK_TEAM_COLORS.get(team_name, fallback_color),
            )
            driver_number = str(
                _first_present(row, ["DriverNumber", "RacingNumber"], "")
            )
            full_name = str(
                _first_present(row, ["FullName", "FirstName", "LastName"], code)
            )
            position_raw = _first_present(row, ["Position", "ClassifiedPosition"], None)
            try:
                position = int(position_raw)
            except (TypeError, ValueError):
                position = None

            info[code] = DriverInfo(
                code=code,
                driver_number=driver_number,
                full_name=full_name,
                team_name=team_name,
                team_color=team_color,
                position=position,
            )

        if not info:
            raise FastF1DataUnavailable("FastF1 did not expose driver metadata.")
        return info

    def _pick_driver_laps(self, code: str) -> pd.DataFrame:
        laps = self.session.laps
        if hasattr(laps, "pick_driver"):
            return laps.pick_driver(code)
        return laps.pick_drivers(code)

    def _build_driver_frame(self, code: str) -> pd.DataFrame:
        laps = self._pick_driver_laps(code)
        if laps is None or laps.empty:
            return pd.DataFrame()

        telemetry = laps.get_telemetry().add_distance()
        if telemetry is None or telemetry.empty:
            return pd.DataFrame()
        if "X" not in telemetry.columns or "Y" not in telemetry.columns:
            raise FastF1DataUnavailable(f"{code} telemetry has no X/Y position data.")

        if "SessionTime" in telemetry.columns:
            time_s = _series_seconds(telemetry["SessionTime"])
        elif "Time" in telemetry.columns:
            time_s = _series_seconds(telemetry["Time"])
        elif "Date" in telemetry.columns:
            time_s = _series_seconds(telemetry["Date"])
        else:
            time_s = pd.Series(np.arange(len(telemetry)) * self.dt, index=telemetry.index)

        frame = pd.DataFrame(index=telemetry.index)
        frame["time_s"] = pd.to_numeric(time_s, errors="coerce")
        frame["x"] = pd.to_numeric(telemetry["X"], errors="coerce")
        frame["y"] = pd.to_numeric(telemetry["Y"], errors="coerce")
        frame["z"] = pd.to_numeric(telemetry.get("Z", 0.0), errors="coerce")
        frame["speed_kmh"] = pd.to_numeric(telemetry.get("Speed", 0.0), errors="coerce")
        frame["rpm"] = pd.to_numeric(telemetry.get("RPM", 0), errors="coerce").fillna(0)
        frame["gear"] = pd.to_numeric(telemetry.get("nGear", 0), errors="coerce").fillna(0)
        frame["throttle_pct"] = pd.to_numeric(
            telemetry.get("Throttle", 0.0), errors="coerce"
        ).fillna(0.0)
        brake = telemetry.get("Brake", False)
        frame["brake_pressure_bar"] = (
            pd.Series(brake, index=telemetry.index).astype(float).fillna(0.0) * 100.0
        )
        frame["drs_value"] = pd.to_numeric(telemetry.get("DRS", 0), errors="coerce").fillna(0)
        frame["distance_raw"] = pd.to_numeric(
            telemetry.get("Distance", 0.0), errors="coerce"
        ).fillna(0.0)

        frame = frame.dropna(subset=["time_s", "x", "y", "speed_kmh"])
        frame = frame.sort_values("time_s").drop_duplicates("time_s")
        if frame.empty:
            return frame

        lap_meta = self._lap_metadata_for_times(frame["time_s"].to_numpy(dtype=float), laps)
        frame["lap_number"] = lap_meta["lap_number"]
        frame["compound"] = lap_meta["compound"]
        frame["tyre_life"] = lap_meta["tyre_life"]

        distance_raw = frame["distance_raw"].to_numpy(dtype=float)
        if np.nanmax(distance_raw) > self.track_length_m * 1.5:
            distance_total = distance_raw
            s_mod = np.mod(distance_total, self.track_length_m)
        else:
            s_mod = np.mod(distance_raw, self.track_length_m)
            distance_total = (frame["lap_number"].to_numpy(dtype=float) - 1.0) * self.track_length_m + s_mod

        frame["s"] = s_mod
        frame["s_total"] = distance_total

        dx = np.gradient(frame["x"].to_numpy(dtype=float))
        dy = np.gradient(frame["y"].to_numpy(dtype=float))
        frame["heading_rad"] = np.arctan2(dy, dx)

        return frame.reset_index(drop=True)

    def _lap_metadata_for_times(self, times: np.ndarray, laps: pd.DataFrame) -> Dict[str, np.ndarray]:
        laps = laps.copy()
        if "LapStartTime" in laps.columns:
            lap_start_s = _series_seconds(laps["LapStartTime"]).to_numpy(dtype=float)
        else:
            lap_start_s = np.zeros(len(laps), dtype=float)

        valid = np.isfinite(lap_start_s)
        laps = laps.loc[valid].copy()
        lap_start_s = lap_start_s[valid]
        if laps.empty:
            return {
                "lap_number": np.ones(len(times), dtype=int),
                "compound": np.array(["UNKNOWN"] * len(times), dtype=object),
                "tyre_life": np.zeros(len(times), dtype=int),
            }

        order = np.argsort(lap_start_s)
        lap_start_s = lap_start_s[order]
        laps = laps.iloc[order]
        lap_idx = np.searchsorted(lap_start_s, times, side="right") - 1
        lap_idx = np.clip(lap_idx, 0, len(laps) - 1)

        lap_numbers = pd.to_numeric(laps["LapNumber"], errors="coerce").fillna(1).astype(int).to_numpy()
        compounds = laps.get("Compound", pd.Series(["UNKNOWN"] * len(laps))).fillna("UNKNOWN").astype(str).to_numpy()
        tyre_life = pd.to_numeric(
            laps.get("TyreLife", pd.Series([0] * len(laps))), errors="coerce"
        ).fillna(0).astype(int).to_numpy()

        return {
            "lap_number": lap_numbers[lap_idx],
            "compound": compounds[lap_idx],
            "tyre_life": tyre_life[lap_idx],
        }

    def _build_track_map(self) -> pd.DataFrame:
        try:
            fastest = self.session.laps.pick_fastest()
            telemetry = fastest.get_telemetry().add_distance()
            track = telemetry[["X", "Y", "Distance"]].dropna().copy()
            track = track.rename(columns={"X": "x", "Y": "y", "Distance": "s"})
            return track.sort_values("s").reset_index(drop=True)
        except Exception:
            ego = self.driver_telemetry[self.ego_driver]
            return ego[["x", "y", "s"]].sort_values("s").reset_index(drop=True)

    def reset(self) -> None:
        self.step_idx = 0

    def is_finished(self) -> bool:
        return self.step_idx >= len(self.time_steps)

    def get_next_frame(self) -> Optional[TelemetryFrame]:
        if self.is_finished():
            return None

        replay_time_s = float(self.time_steps[self.step_idx])
        frame_index = self.step_idx
        self.step_idx += 1

        ego_sample = self._sample_driver(self.ego_driver, replay_time_s)
        if ego_sample is None:
            return None

        ego_info = self.driver_info[self.ego_driver]
        ego_kin = self._kinematics_from_sample(ego_sample)
        ego_tel = EgoTelemetry(
            session_time=replay_time_s,
            lap_number=int(ego_sample["lap_number"]),
            track_status="GREEN",
            kinematics=ego_kin,
            ers=self._synthetic_ers(ego_sample, replay_time_s),
            tires=self._tires_from_sample(ego_sample),
            throttle_pct=float(ego_sample["throttle_pct"]),
            brake_pressure_bar=float(ego_sample["brake_pressure_bar"]),
            gear=int(ego_sample["gear"]),
            drs_active=bool(int(ego_sample["drs_value"]) in DRS_ACTIVE_VALUES),
            fuel_remaining_kg=max(0.0, 110.0 - (ego_sample["lap_number"] - 1) * 1.75),
            driver_code=ego_info.code,
            driver_number=ego_info.driver_number,
            team_name=ego_info.team_name,
            team_color=ego_info.team_color,
            position=ego_info.position,
            rpm=int(ego_sample["rpm"]),
        )

        opponents: Dict[str, OpponentTelemetry] = {}
        for code in self.opponent_drivers:
            sample = self._sample_driver(code, replay_time_s)
            if sample is None:
                continue
            info = self.driver_info[code]
            gap_m = float(sample["s_total"] - ego_sample["s_total"])
            speed_gap = float(ego_sample["speed_kmh"] - sample["speed_kmh"])
            gap_s = gap_m / max(1.0, sample["speed_kmh"] / 3.6)

            opponents[code] = OpponentTelemetry(
                car_id=code,
                driver_code=code,
                kinematics=self._kinematics_from_sample(sample),
                gap_to_ego_m=gap_m,
                gap_to_ego_s=gap_s,
                speed_gap_kmh=speed_gap,
                compound=str(sample["compound"]),
                laps_old=int(sample["tyre_life"]),
                inferred_deg_delta_s=float(max(0.0, sample["tyre_life"] * 0.025)),
                drs_active=bool(int(sample["drs_value"]) in DRS_ACTIVE_VALUES),
                is_clipping=bool(sample["speed_kmh"] > 330.0 and sample["throttle_pct"] > 95.0),
                driver_number=info.driver_number,
                team_name=info.team_name,
                team_color=info.team_color,
                position=info.position,
                rpm=int(sample["rpm"]),
                throttle_pct=float(sample["throttle_pct"]),
                brake_pressure_bar=float(sample["brake_pressure_bar"]),
                gear=int(sample["gear"]),
                active=bool(sample["active"]),
            )

        return TelemetryFrame(
            timestamp=time.time(),
            ego=ego_tel,
            opponents=opponents,
            source="fastf1",
            session_label=self.session_label,
            replay_time_s=replay_time_s - self.start_time_s,
            metadata={
                "year": self.year,
                "circuit": self.circuit,
                "session_type": self.session_type,
                "ego_driver": self.ego_driver,
                "frame_index": frame_index,
                "field_size": len(opponents) + 1,
                "fastf1_cache_dir": self.cache_dir,
            },
        )

    def _sample_driver(self, code: str, time_s: float) -> Optional[Dict[str, Any]]:
        frame = self.driver_telemetry.get(code)
        if frame is None or frame.empty:
            return None

        times = frame["time_s"].to_numpy(dtype=float)
        active = bool(times[0] <= time_s <= times[-1])
        clipped_time = float(np.clip(time_s, times[0], times[-1]))

        nearest_idx = int(np.searchsorted(times, clipped_time, side="left"))
        if nearest_idx >= len(frame):
            nearest_idx = len(frame) - 1
        if nearest_idx > 0 and abs(times[nearest_idx - 1] - clipped_time) < abs(times[nearest_idx] - clipped_time):
            nearest_idx -= 1

        row = frame.iloc[nearest_idx]
        numeric_cols = [
            "x",
            "y",
            "z",
            "speed_kmh",
            "s",
            "s_total",
            "heading_rad",
            "throttle_pct",
            "brake_pressure_bar",
            "rpm",
            "gear",
            "drs_value",
        ]
        sample: Dict[str, Any] = {}
        for col in numeric_cols:
            values = frame[col].to_numpy(dtype=float)
            sample[col] = float(np.interp(clipped_time, times, values))

        sample["lap_number"] = int(row["lap_number"])
        sample["compound"] = str(row["compound"])
        sample["tyre_life"] = int(row["tyre_life"])
        sample["active"] = active
        return sample

    def _kinematics_from_sample(self, sample: Dict[str, Any]) -> KinematicsTelemetry:
        return KinematicsTelemetry(
            s=float(sample["s"]),
            y=0.0,
            x_world=float(sample["x"]),
            y_world=float(sample["y"]),
            heading_rad=float(sample["heading_rad"]),
            speed_kmh=float(sample["speed_kmh"]),
            accel_long_g=0.0,
            accel_lat_g=0.0,
            yaw_rate_rads=0.0,
        )

    def _synthetic_ers(self, sample: Dict[str, Any], replay_time_s: float) -> ERSTelemetry:
        throttle = float(sample["throttle_pct"])
        brake = float(sample["brake_pressure_bar"])
        mguk_kw = 260.0 if throttle > 90.0 else (-120.0 if brake > 10.0 else 40.0)
        soc = float(np.clip(78.0 - 0.006 * (replay_time_s - self.start_time_s), 18.0, 82.0))
        return ERSTelemetry(
            soc_pct=soc,
            soh_pct=99.0,
            pack_temp_c=47.5,
            max_cell_temp_c=49.0,
            current_demand_a=float(abs(mguk_kw) * 1000.0 / 750.0),
            voltage_v=750.0,
            mguk_power_kw=mguk_kw,
            lap_deployed_mj=float(max(0.0, (82.0 - soc) * 0.08)),
        )

    def _tires_from_sample(self, sample: Dict[str, Any]) -> TireChannel:
        tyre_life = int(sample["tyre_life"])
        wear = float(np.clip(tyre_life * 2.2, 0.0, 85.0))
        return TireChannel(
            compound=str(sample["compound"]),
            laps_old=tyre_life,
            wear_pct=wear,
            deg_rate_pct_per_lap=0.45,
            lap_time_deg_penalty_s=wear * 0.025,
            mu_friction_coeff=float(np.clip(1.75 - wear * 0.006, 1.05, 1.75)),
            surface_temp_c={"FL": 94.0, "FR": 95.0, "RL": 92.0, "RR": 93.0},
            carcass_temp_c={"FL": 86.0, "FR": 87.0, "RL": 84.0, "RR": 85.0},
        )
