from mavsdk_grpc import System
from textual.app import App, ComposeResult
from textual.containers import Grid, Vertical
from textual.widgets import Footer, Header, Log, Static

# Configuration Constants
SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"  # Standard PX4 SITL UDP port


class QuadcopterMissionApp(App):
    """Textual TUI application for running a MAVSDK square mission and viewing telemetry."""

    TITLE = "Quadcopter Mission & Telemetry Console"
    CSS = """
    Screen {
        layout: vertical;
    }
    #telemetry-grid {
        grid-size: 2 2;
        grid-gutter: 1 2;
        padding: 1 2;
        height: 15;
    }
    TelemetryCard {
        border: round $primary;
        padding: 0 2;
        height: 100%;
        content-align: left middle;
    }
    #mission-container {
        padding: 0 2 1 2;
        height: 1fr;
    }
    #status-banner {
        background: $boost;
        color: $text;
        padding: 0 2;
        height: 3;
        content-align: left middle;
        border: solid $accent;
        margin-bottom: 1;
    }
    Log {
        border: round $secondary;
        height: 1fr;
    }
    """

    BINDINGS = [
        ("q", "quit", "Quit"),
        ("l", "emergency_land", "Emergency Land"),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.drone = System()

    def compose(self) -> ComposeResult:
        import telemetry_stream

        yield Header(show_clock=True)
        with Grid(id="telemetry-grid"):
            yield telemetry_stream.TelemetryCard(
                "Roll:  --\nPitch: --\nYaw:   --",
                title="Attitude (Euler)",
                id="card-attitude",
            )
            yield telemetry_stream.TelemetryCard(
                "Relative (AGL): --\nAbsolute (AMSL): --",
                title="Altitude",
                id="card-altitude",
            )
            yield telemetry_stream.TelemetryCard(
                "Ground Speed: --\nNorth / East: --",
                title="Velocity",
                id="card-speed",
            )
            yield telemetry_stream.TelemetryCard(
                "Remaining:   --\nVoltage:     --\nConsumption: --",
                title="Battery",
                id="card-battery",
            )
        with Vertical(id="mission-container"):
            yield Static("Mission Status: Initializing...", id="status-banner")
            yield Log(id="mission-log", highlight=True)
        yield Footer()

    async def on_mount(self) -> None:
        """Start the MAVSDK connection, telemetry listeners, and mission workflow."""
        
        self.run_worker(self.initialize_and_run(), exclusive=True)

    def log_step(self, message: str) -> None:
        """Update both the status banner and the scrolling mission log."""

        self.query_one("#status-banner", Static).update(f"Mission Status: [bold]{message}[/bold]")
        self.query_one("#mission-log", Log).write_line(message)

    async def initialize_and_run(self) -> None:
        """Connect to the quadcopter, spawn telemetry tasks, and run the square mission."""

        import mission_executor, telemetry_stream

        self.log_step(f"Connecting to drone at {SYSTEM_ADDRESS}...")
        await self.drone.connect(system_address=SYSTEM_ADDRESS)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log_step("Connected to quadcopter!")
                break

        # Start concurrent background workers for each telemetry stream
        self.run_worker(telemetry_stream.stream_attitude(self))
        self.run_worker(telemetry_stream.stream_altitude(self))
        self.run_worker(telemetry_stream.stream_ground_speed(self))
        self.run_worker(telemetry_stream.stream_battery(self))
        self.run_worker(telemetry_stream.stream_mission_progress(self))

        # Execute the flight mission
        await mission_executor.execute_mission(self)

    async def action_emergency_land(self) -> None:
        """Keybinding handler ('l') to pause the mission and command an immediate landing."""
        
        self.log_step("EMERGENCY LAND triggered by user!")
        try:
            await self.drone.mission.pause_mission()
            await self.drone.action.land()
        except Exception as exc:
            self.log_step(f"Emergency land failed: {exc}")


if __name__ == "__main__":
    app = QuadcopterMissionApp()
    app.run()