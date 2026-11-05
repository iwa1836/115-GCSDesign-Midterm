import asyncio
import math
from mavsdk_grpc import System
from mavsdk_grpc.mission import MissionItem, MissionPlan
from mavsdk_grpc.offboard import OffboardError, PositionNedYaw, VelocityNedYaw, VelocityBodyYawspeed
from main import QuadcopterMissionApp
import mission_builder

# Configuration Constants
ALTITUDE_M = 10.0                           # Target mission altitude (meters)
SPEED_M_S = 5.0                             # Target flight speed (m/s)
CIRCLE_RADIUS_M = 15.0                      # Circle radius (m)
HEXAGON_SIDE_LENGTH_M = CIRCLE_RADIUS_M * 2 # Hexagon side length (m)


async def execute_mission(self: QuadcopterMissionApp) -> None:
    try:
        # Wait for the drone to have a valid global position and home position
        self.log_step("Waiting for global position estimate and home lock...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.log_step("Global position & home position OK.")
                break

        # Fetch current GPS coordinates as the square's origin
        home_lat, home_lon = float('nan'), float('nan')
        async for pos in self.drone.telemetry.position():
            home_lat = pos.latitude_deg
            home_lon = pos.longitude_deg
            break

        # Build a hexagon mission plan around the home position
        mission_items = []
        current_lat, current_lon = home_lat, home_lon
        next_lat, next_lon = mission_builder.get_destination_coordinate(current_lat, current_lon, HEXAGON_SIDE_LENGTH_M, 240)
        mission_items.append(mission_builder.make_wp(next_lat, next_lon, ALTITUDE_M, SPEED_M_S, MissionItem.VehicleAction.NONE))
        current_lat, current_lon = next_lat, next_lon
        for i in range(6):
            next_lat, next_lon = mission_builder.get_destination_coordinate(current_lat, current_lon, HEXAGON_SIDE_LENGTH_M, 60 * i)
            mission_items.append(mission_builder.make_wp(next_lat, next_lon, ALTITUDE_M, SPEED_M_S, MissionItem.VehicleAction.NONE))
            current_lat, current_lon = next_lat, next_lon
        next_lat, next_lon = mission_builder.get_destination_coordinate(current_lat, current_lon, HEXAGON_SIDE_LENGTH_M, 60)
        mission_items.append(mission_builder.make_wp(next_lat, next_lon, ALTITUDE_M, SPEED_M_S, MissionItem.VehicleAction.NONE))
        current_lat, current_lon = next_lat, next_lon

        # Upload the mission to the drone
        self.log_step("Uploading mission...")
        await self.drone.mission.set_return_to_launch_after_mission(False)
        await self.drone.mission.upload_mission(MissionPlan(mission_items))
        await asyncio.sleep(1)  # Allow some time for the mission to be processed

        # Set the return-to-launch altitude to ensure a safe return
        await self.drone.action.set_return_to_launch_altitude(ALTITUDE_M)

        # 1. Arm
        self.log_step("Arming quadcopter...")
        await self.drone.action.arm()
        await asyncio.sleep(2)  # Allow some time for arming to complete

        # 2. Take off
        self.log_step(f"Taking off to {ALTITUDE_M} m...")
        await self.drone.action.set_takeoff_altitude(ALTITUDE_M)
        await self.drone.action.takeoff()
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= ALTITUDE_M * 0.95:
                self.log_step(f"Reached target altitude: {ALTITUDE_M} m")
                await asyncio.sleep(2)  # Allow some time to stabilize at altitude
                break

        # 3. Fly in mission mode
        self.log_step("Starting hexagon mission...")
        await self.drone.mission.start_mission()

        async for progress in self.drone.mission.mission_progress():
            if progress.current == progress.total and progress.total > 0:
                await asyncio.sleep(1)  # Allow some time to stabilize after mission completion
                self.log_step("Hexagon mission completed!")
                await asyncio.sleep(1)  # Allow some time to stabilize after mission completion
                break

        # 4. Go to the starting point of the circle pattern
        self.log_step("Navigating to starting point of circle pattern...")
        start_lat, start_lon = mission_builder.get_destination_coordinate(home_lat, home_lon, CIRCLE_RADIUS_M, 180)  # 180 degrees = South
        await self.drone.action.goto_location(start_lat, start_lon, ALTITUDE_M, 0.0)
        async for position in self.drone.telemetry.position():
            if (abs(position.latitude_deg - start_lat) < 0.00001 and abs(position.longitude_deg - start_lon) < 0.00001):
                self.log_step("Reached starting point of circle pattern.")
                await asyncio.sleep(2)  # Allow some time to stabilize at the starting point
                break

        # 5. Fly in offboard mode
        self.log_step("Starting offboard mode...")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.log_step(f"Starting offboard mode failed: {error}")
        await asyncio.sleep(1)  # Allow some time for offboard mode to stabilize

        if self.drone.offboard.is_active():
            self.log_step("Offboard mode active. Flying circle pattern...")
            half_detected = False
            current_heading_deg = 0.0
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, SPEED_M_S, 0.0, -math.degrees(SPEED_M_S / CIRCLE_RADIUS_M)))
            async for attitude in self.drone.telemetry.attitude_euler():
                current_heading_deg = attitude.yaw_deg % 360.0
                if not half_detected and 175.0 <= current_heading_deg <= 185.0:
                    half_detected = True
                    self.log_step("Halfway through circle pattern.")
                if half_detected and 355.0 >= current_heading_deg <= 5.0:
                    self.log_step("Completed circle pattern.")
                    # Stop the drone's motion by setting all velocities to zero
                    await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    await asyncio.sleep(2)  # Allow some time to stabilize after stopping
                    break

            # Stop offboard mode
            self.log_step("Stopping offboard mode...")
            try:
                await self.drone.offboard.stop()
            except OffboardError as error:
                self.log_step(f"Stopping offboard mode failed: {error}")

        # 6. Return to launch
        self.log_step("Returning to launch...")
        await self.drone.action.return_to_launch()

        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                self.log_step("Touchdown confirmed.")
                await asyncio.sleep(2)  # Allow some time to stabilize on the ground
                break

        # 7. Disarm
        self.log_step("Disarming quadcopter...")
        try:
            await self.drone.action.disarm()
        except Exception:
            # Autopilot auto-disarm on touchdown may have already disarmed the vehicle
            pass
        await asyncio.sleep(2)  # Allow some time for disarming to complete

        self.log_step("Mission Complete — Quadcopter disarmed safely.")

    except Exception as exc:
        self.log_step(f"Mission Error: {exc}")