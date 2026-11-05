import math
from textual.widgets import Static
from main import QuadcopterMissionApp

class TelemetryCard(Static):
    """A styled widget card for displaying a group of telemetry metrics."""

    def __init__(self, content: str = "", *, title: str = "", **kwargs) -> None:
        super().__init__(content, **kwargs)
        self.border_title = title


async def stream_attitude(self: QuadcopterMissionApp) -> None:
    card = self.query_one("#card-attitude", TelemetryCard)
    async for euler in self.drone.telemetry.attitude_euler():
        card.update(
            f"Roll:  {euler.roll_deg:+6.2f}°\n"
            f"Pitch: {euler.pitch_deg:+6.2f}°\n"
            f"Yaw:   {euler.yaw_deg:6.2f}°"
        )

async def stream_altitude(self: QuadcopterMissionApp) -> None:
    card = self.query_one("#card-altitude", TelemetryCard)
    async for pos in self.drone.telemetry.position():
        card.update(
            f"Relative (AGL):  {pos.relative_altitude_m:6.2f} m\n"
            f"Absolute (AMSL): {pos.absolute_altitude_m:6.2f} m"
        )

async def stream_ground_speed(self: QuadcopterMissionApp) -> None:
    card = self.query_one("#card-speed", TelemetryCard)
    async for vel in self.drone.telemetry.velocity_ned():
        ground_speed = math.hypot(vel.north_m_s, vel.east_m_s)
        card.update(
            f"Ground Speed: {ground_speed:5.2f} m/s\n"
            f"N: {vel.north_m_s:+5.2f} m/s | E: {vel.east_m_s:+5.2f} m/s"
        )

async def stream_battery(self: QuadcopterMissionApp) -> None:
    card = self.query_one("#card-battery", TelemetryCard)
    async for bat in self.drone.telemetry.battery():
        # Handle MAVSDK v1 (0.0-1.0) vs v2/v3 (0.0-100.0) percentage ranges
        pct = bat.remaining_percent
        if not math.isnan(pct) and pct <= 1.0:
            pct *= 100.0
        pct_str = f"{pct:5.1f} %" if not math.isnan(pct) else "N/A"

        volt_str = f"{bat.voltage_v:5.2f} V" if not math.isnan(bat.voltage_v) else "N/A"

        consumed_ah = getattr(bat, "capacity_consumed_ah", float("nan"))
        if not math.isnan(consumed_ah):
            cons_str = f"{consumed_ah:.3f} Ah ({consumed_ah * 1000:.0f} mAh)"
        else:
            cons_str = "N/A"

        card.update(
            f"Remaining:   {pct_str}\n"
            f"Voltage:     {volt_str}\n"
            f"Consumption: {cons_str}"
        )

async def stream_mission_progress(self: QuadcopterMissionApp) -> None:
    current_progress = None
    previous_progress = None
    async for progress in self.drone.mission.mission_progress():
        current_progress = progress.current
        if current_progress != previous_progress and current_progress > 0:
            self.log_step(f"Reached Waypoint {progress.current}/{progress.total}")
            previous_progress = current_progress