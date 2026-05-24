import sys
import os
import math

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from high_level.src.costmap import LocalCostmap
from high_level.src.mission_control import MissionController, STATE_PARKUR1
from high_level.src.planner import gps_to_meters
from scratch.test_gate_navigation import PlaneCrossingPlanner
from scratch.sitl_simulator import IDASimulator, DummySerial, DummyLogger

def run_debug():
    sim = IDASimulator()
    serial_mock = DummySerial(sim)
    logger_mock = DummyLogger()
    
    mission = MissionController(logger_mock, serial_mock)
    mission.set_waypoints(sim.p1_gps, sim.p2_gps, sim.home_gps)
    mission.target_color = "target_red" 
    
    mission.planner = PlaneCrossingPlanner(waypoint_tolerance_m=1.3, nominal_speed_ms=1.3, max_speed_ms=2.0)
    costmap = LocalCostmap(size_m=40.0, resolution=0.25, inflation_radius_m=1.0)
    mission.transition_to(STATE_PARKUR1)
    
    dt = 0.04
    print("Step,X,Y,Yaw,Speed,ActiveWP,State,RepX,RepY")
    for step in range(800):
        sim.step(dt)
        telemetry = sim.get_simulated_gps_imu()
        mission.update_telemetry(telemetry)
        detections = sim.get_simulated_camera_detections()
        mission.process_step(detections, costmap)
        
        rep_x, rep_y = costmap.get_obstacle_forces()
        print(f"{step},{sim.x:.3f},{sim.y:.3f},{sim.yaw:.1f},{sim.speed:.3f},{mission.current_wp_idx},{mission.state},{rep_x:.3f},{rep_y:.3f}")
        
        if mission.state != STATE_PARKUR1:
            print(f"Exited PARKUR1 at step {step} with state {mission.state}")
            break

if __name__ == "__main__":
    run_debug()
