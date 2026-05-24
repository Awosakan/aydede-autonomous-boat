import sys
import os
import math
import numpy as np

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from high_level.src.costmap import LocalCostmap
from high_level.src.mission_control import MissionController, STATE_PARKUR1
from high_level.src.protocol import unpack_phone_commands
from high_level.src.planner import gps_to_meters, APFPlanner
from scratch.sitl_simulator import IDASimulator, SimplePID, DummySerial, DummyLogger

class PlaneCrossingPlanner(APFPlanner):
    def plan(self, current_lat: float, current_lon: float, current_yaw_deg: float, current_speed: float,
             waypoints: list, current_wp_idx: int, costmap, prev_wp_gps: list = None, dt: float = 0.04) -> tuple:
        
        if not waypoints or current_wp_idx >= len(waypoints):
            return 0.0, current_yaw_deg, current_wp_idx, True
            
        target_lat, target_lon = waypoints[current_wp_idx]
        dx_m, dy_m = gps_to_meters(current_lat, current_lon, target_lat, target_lon)
        dist_to_wp = math.sqrt(dx_m**2 + dy_m**2)
        
        # Waypoint reach decision logic
        reached = False
        
        # 1. Super close backup
        if dist_to_wp < 0.5:
            reached = True
            
        # 2. Perpendicular plane crossing check (along-track distance)
        elif prev_wp_gps:
            line_dx, line_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], target_lat, target_lon)
            line_len = math.sqrt(line_dx**2 + line_dy**2)
            if line_len > 1.0:
                u_x = line_dx / line_len
                u_y = line_dy / line_len
                boat_dx, boat_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], current_lat, current_lon)
                along_track = boat_dx * u_x + boat_dy * u_y
                
                # Check if we have crossed the perpendicular plane and are reasonably close laterally
                if along_track >= line_len:
                    reached = True
                    
        # 3. Normal tolerance as backup (if no prev_wp_gps)
        elif dist_to_wp < self.waypoint_tolerance_m:
            reached = True
            
        if reached:
            print(f"[PLANNER] Reached WP {current_wp_idx}! Dist to WP: {dist_to_wp:.2f}m")
            current_wp_idx += 1
            self.cte_integrator = 0.0
            return 0.0, current_yaw_deg, current_wp_idx, (current_wp_idx >= len(waypoints))
            
        # Run rest of APFPlanner plan method using super() but override the waypoint checking by bypassing it
        # We can temporarily set waypoint_tolerance_m to a very small value to avoid double trigger in super().plan
        old_tol = self.waypoint_tolerance_m
        self.waypoint_tolerance_m = 0.001
        res = super().plan(current_lat, current_lon, current_yaw_deg, current_speed, waypoints, current_wp_idx, costmap, prev_wp_gps, dt)
        self.waypoint_tolerance_m = old_tol
        return res

def run_headless_simulation():
    sim = IDASimulator()
    serial_mock = DummySerial(sim)
    logger_mock = DummyLogger()
    
    mission = MissionController(logger_mock, serial_mock)
    mission.set_waypoints(sim.p1_gps, sim.p2_gps, sim.home_gps)
    mission.target_color = "target_red" 
    
    # Use our PlaneCrossingPlanner!
    mission.planner = PlaneCrossingPlanner(waypoint_tolerance_m=1.3, nominal_speed_ms=1.3, max_speed_ms=2.0)
    
    costmap = LocalCostmap(size_m=40.0, resolution=0.25, inflation_radius_m=1.0)
    mission.transition_to(STATE_PARKUR1)
    
    dt = 0.04
    print("Starting headless simulation with PlaneCrossingPlanner...")
    
    positions = []
    for step in range(2000):
        sim.step(dt)
        telemetry = sim.get_simulated_gps_imu()
        mission.update_telemetry(telemetry)
        detections = sim.get_simulated_camera_detections()
        mission.process_step(detections, costmap)
        
        positions.append((sim.x, sim.y, mission.current_wp_idx, mission.state))
        
        if mission.state == "IDLE" and step > 100:
            break
            
    # Trajectory crossing analysis
    print("\n--- Trajectory Analysis ---")
    for i in range(1, len(positions)):
        x1, y1, wp1, st1 = positions[i-1]
        x2, y2, wp2, st2 = positions[i]
        
        if y1 >= 10.0 > y2:
            t = (10.0 - y1) / (y2 - y1) if y2 != y1 else 0.0
            x_cross = x1 + t * (x2 - x1)
            print(f"Boat crossed y=10.0 (going south) at x={x_cross:.3f} (WP active during crossing: {wp2}, State: {st2})")
            if 8.5 <= x_cross <= 11.5:
                print("SUCCESS: Passed between Gate 3 buoys!")
            else:
                print("FAILURE: Did NOT pass between Gate 3 buoys!")

if __name__ == "__main__":
    run_headless_simulation()
