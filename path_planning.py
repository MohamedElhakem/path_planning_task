from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D
import math

class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """

        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.
        path: Path2D = []
        max_dist = 10 #m
        step = 0.1
        cx = self.car_pose.x
        cy = self.car_pose.y
        yaw_dir = []
        yellow_cones = [cone for cone in self.cones if cone.color == 0]
        blue_cones = [cone for cone in self.cones if cone.color == 1]

        #Get nearest cones on each side to the car
        yellow_cones.sort(key=lambda cone: math.hypot(cone.x - cx, cone.y - cy))
        blue_cones.sort(key=lambda cone: math.hypot(cone.x - cx, cone.y - cy))

        #if no cones, go straight
        if not yellow_cones and not blue_cones:
            yaw = self.car_pose.yaw
            steps = int(max_dist / step)
            for i in range(1, steps + 1):
                dx = math.cos(yaw) * step * i
                dy = math.sin(yaw) * step * i
                path.append((cx + dx, cy + dy))
            return path
        
        #if only one side cones, move left to the blue cones or right to yellow cones

        #Only adjust yaw if we have cones on both sides
        #list of mid points
        main_points = []
        main_points.append((cx, cy))
        if yellow_cones and blue_cones:
            mid_yo = (yellow_cones[0].y + blue_cones[0].y) / 2
            mid_xo = (yellow_cones[0].x + blue_cones[0].x) / 2
            main_points.append((mid_xo, mid_yo))
            yaw_dir.append(math.atan2(mid_yo - cy, mid_xo - cx))
            for i in range(1, max(len(yellow_cones), len(blue_cones))):
                r = 0
                l = 0
                if i >=  min(len(yellow_cones), len(blue_cones)):
                    if i >=  len(blue_cones):
                        r = i
                        l = len(blue_cones) - 1
                    else:
                        r = len(yellow_cones) - 1
                        l = i
                else:
                    r = i
                    l = i
                mid_y = (yellow_cones[r].y + blue_cones[l].y) / 2
                mid_x = (yellow_cones[r].x + blue_cones[l].x) / 2
                main_points.append((mid_x, mid_y))
                yaw_dir.append(math.atan2(main_points[i + 1][1] - main_points[i][1], main_points[i + 1][0] - main_points[i][0])) 
            #get the slop of the normal on the line connecting the last two cones
            final_yaw = (math.atan2(yellow_cones[-1].y - blue_cones[-1].y, yellow_cones[-1].x - blue_cones[-1].x) + math.pi/2)
            yaw_dir.append(final_yaw)
        #number of steps to take between each two midpoints
        num_steps = []
        for j in range(1, len(main_points)):
            dist = math.hypot(main_points[j][0] - main_points[j - 1][0], main_points[j][1] - main_points[j - 1][1])
            num_steps.append(int(dist / step))
            print(f"num_steps debug: dist={dist} steps={int(dist / step)}")
        num_steps.append(int((max_dist - sum(num_steps) * step) / step))
                # debug print - run your scenario and read these numbers
        print(f"DEBUG: yellow={len(yellow_cones)} blue={len(blue_cones)} main_points={len(main_points)} yaw_dir={len(yaw_dir)} num_steps={len(num_steps)} path={len(path)}")
        
        for j in range(len(main_points)):
            for i in range(num_steps[j] + 1):
                dx = math.cos(yaw_dir[j]) * step * i
                dy = math.sin(yaw_dir[j]) * step * i
                path.append((main_points[j][0] + dx,main_points[j][1]  + dy))
    
        return path



