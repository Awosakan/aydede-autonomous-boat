import sys
import os
import math

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from high_level.src.costmap import LocalCostmap
from high_level.src.mission_control import MissionController, STATE_PARKUR1
from high_level.src.planner import gps_to_meters
from scratch.sitl_simulator import IDASimulator, DummySerial, DummyLogger
from scratch.test_gate_navigation import PlaneCrossingPlanner

def run_debug():
    sim = IDASimulator()
    serial_mock = DummySerial(sim)
    logger_mock = DummyLogger()
    
    # Overwrite SimplePID.update to log details
    original_update = serial_mock.pid.update
    def debug_update(current_yaw, target_heading, target_speed, dt):
        left, right = original_update(current_yaw, target_heading, target_speed, dt)
        return left, right
    serial_mock.pid.update = debug_update
    
    mission = MissionController(logger_mock, serial_mock)
    mission.set_waypoints(sim.p1_gps, sim.p2_gps, sim.home_gps)
    mission.target_color = "target_red" 
    mission.planner = PlaneCrossingPlanner(waypoint_tolerance_m=1.3, nominal_speed_ms=1.3, max_speed_ms=2.0)
    
    # Override APFPlanner.plan to inspect forces
    original_plan = mission.planner.plan
    def debug_plan(current_lat, current_lon, current_yaw_deg, current_speed,
                   waypoints, current_wp_idx, costmap, prev_wp_gps=None, dt=0.04):
        
        # Call original plan
        speed, heading, wp_idx, finished = original_plan(
            current_lat, current_lon, current_yaw_deg, current_speed,
            waypoints, current_wp_idx, costmap, prev_wp_gps, dt
        )
        
        # Calculate attractive force manually for logging
        if not finished and current_wp_idx < len(waypoints):
            target_lat, target_lon = waypoints[current_wp_idx]
            dx_m, dy_m = gps_to_meters(current_lat, current_lon, target_lat, target_lon)
            dist_to_wp = math.sqrt(dx_m**2 + dy_m**2)
            
            yaw_rad = math.radians(current_yaw_deg)
            # We ignore CTE correction for simple log
            x_body = dx_m * math.sin(yaw_rad) + dy_m * math.cos(yaw_rad)
            y_body = dx_m * math.cos(yaw_rad) - dy_m * math.sin(yaw_rad)
            
            att_x, att_y = 0.0, 0.0
            if dist_to_wp > 0.1:
                att_x = mission.planner.K_attractive * (x_body / dist_to_wp)
                att_y = mission.planner.K_attractive * (y_body / dist_to_wp)
                
            rep_x, rep_y = costmap.get_obstacle_forces()
            
            nonlocal step_counter
            if 460 <= step_counter <= 500:
                print(f"[APF step {step_counter}] Boat: ({sim.x:.2f}, {sim.y:.2f}), WP: {current_wp_idx}, Dist: {dist_to_wp:.2f}m, "
                      f"Body Att: ({att_x:.3f}, {att_y:.3f}), Body Rep: ({rep_x:.3f}, {rep_y:.3f}), "
                      f"Planned Heading (after EMA): {heading:.1f}")
                      
        return speed, heading, wp_idx, finished
        
    mission.planner.plan = debug_plan
    
    costmap = LocalCostmap(size_m=40.0, resolution=0.25, inflation_radius_m=1.0)
    mission.transition_to(STATE_PARKUR1)
    
    dt = 0.04
    print(f"Step, Time, X, Y, Yaw, Speed, L_Motor, R_Motor, Active_WP, State")
    step_counter = 0
    positions = []
    for step in range(2000):
        step_counter = step
        sim.step(dt)
        telemetry = sim.get_simulated_gps_imu()
        mission.update_telemetry(telemetry)
        detections = sim.get_simulated_camera_detections()
        mission.process_step(detections, costmap)
        
        positions.append((sim.x, sim.y, mission.current_wp_idx, mission.state))
        
        if step % 50 == 0:
            print(f"{step:4d}, {step*dt:5.2f}, {sim.x:7.3f}, {sim.y:7.3f}, {sim.yaw:6.1f}, {sim.speed:5.2f}, {sim.left_motor:6.3f}, {sim.right_motor:6.3f}, {mission.current_wp_idx}, {mission.state}")
            
        if mission.state == "IDLE" and step > 100:
            break

    print("\n--- Trajectory Crossings of y=10.0 ---")
    for i in range(1, len(positions)):
        x1, y1, wp1, st1 = positions[i-1]
        x2, y2, wp2, st2 = positions[i]
        
        if y1 >= 10.0 > y2:
            t = (10.0 - y1) / (y2 - y1) if y2 != y1 else 0.0
            x_cross = x1 + t * (x2 - x1)
            print(f"Boat crossed y=10.0 (going south) at x={x_cross:.3f} (WP active: {wp2}, State: {st2}, Step: {i})")

if __name__ == "__main__":
    run_debug()
