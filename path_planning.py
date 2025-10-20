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

        def sort_cones_by_chain_front(cones, cx, cy, heading):
            if not cones:
                return []

            def is_in_front(cone):
                # vector from car to cone
                dx = cone.x - cx
                dy = cone.y - cy
                # angle between car heading and cone direction
                angle = math.atan2(dy, dx) - heading
                # normalize to -pi..pi
                angle = math.atan2(math.sin(angle), math.cos(angle))
                # in front means within ±90 degrees
                return abs(angle) < math.pi / 2

            # filter cones to those in front of car
            front_cones = [c for c in cones if is_in_front(c)]

            # start from nearest cone in front
            if front_cones:
                current_x, current_y = cx, cy
                sorted_cones = []
                remaining = front_cones[:]
            else:
                # if none in front, fall back to all cones
                current_x, current_y = cx, cy
                sorted_cones = []
                remaining = cones[:]

            # chain sort (nearest to current each step)
            while remaining:
                next_cone = min(remaining, key=lambda c: math.hypot(c.x - current_x, c.y - current_y))
                sorted_cones.append(next_cone)
                remaining.remove(next_cone)
                current_x, current_y = next_cone.x, next_cone.y

            return sorted_cones

        def distance(p1, p2):
            return math.hypot(p1.x - p2.x, p1.y - p2.y)

        def match_cones(blue, yellow):
            pairs = []

            # find nearest yellow for each blue
            for b in blue:
                nearest_y = min(yellow, key=lambda y: distance(b, y))
                d = distance(b, nearest_y)
                pairs.append((b, nearest_y, d))

            # keep smallest distance for each yellow
            best_pairs = {}
            for b, y, d in pairs:
                if y not in best_pairs or d < best_pairs[y][2]:
                    best_pairs[y] = (b, y, d)

            # extract unique matched cones
            matched_blue = [b for b, _, _ in best_pairs.values()]
            matched_yellow = [y for _, y, _ in best_pairs.values()]

            # find remainder cones (unmatched)
            remaining_blue = [b for b in blue if b not in matched_blue]
            remaining_yellow = [y for y in yellow if y not in matched_yellow]

            return matched_blue, matched_yellow, remaining_blue, remaining_yellow
        
        heading = self.car_pose.yaw
        cx = self.car_pose.x
        cy = self.car_pose.y

        yellow_cones = sort_cones_by_chain_front(
            [c for c in self.cones if c.color == 0],
            cx, cy, heading
        )
        blue_cones = sort_cones_by_chain_front(
            [c for c in self.cones if c.color == 1],
            cx, cy, heading
        )   

        # Debug print - run your scenario and read these numbers
        print(f"yellow_cones={len(yellow_cones)} blue_cones={len(blue_cones)}")

        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.
        path: Path2D = []
        max_dist = 10 #meters
        step = 0.1
        yaw_dir = []

        #if no cones, go straight
        if not yellow_cones and not blue_cones:
            last_point = (cx, cy)
            yaw = self.car_pose.yaw
            steps = int(max_dist / step)
            total_path_length = 0.0
            for i in range(steps + 1):
                dx = math.cos(yaw) * step * i
                dy = math.sin(yaw) * step * i
                new_point = (last_point[0] + dx, last_point[1] + dy)
                next_dist = math.hypot(new_point[0] - last_point[0], new_point[1] - last_point[1]) 
                if total_path_length + next_dist > max_dist:
                    break
                total_path_length += next_dist
                last_point = new_point
                path.append(new_point)
        
            # debug print - run your scenario and read these numbers
            print(f"total_path_length={total_path_length}")
            return path
        
        main_points = []
        main_points.append((cx, cy))
        
        #flitering cones to the same number on each side
        filtered_blue, filtered_yellow, remaining_blue, remaining_yellow = match_cones(blue_cones, yellow_cones)
        yellow_cones = filtered_yellow
        blue_cones = filtered_blue
        
        #list of mid points
        if yellow_cones and blue_cones: 
            #calculate mid points and yaw directions
            mid_yo = (yellow_cones[0].y + blue_cones[0].y) / 2
            mid_xo = (yellow_cones[0].x + blue_cones[0].x) / 2
            main_points.append((mid_xo, mid_yo))
            yaw_dir.append(math.atan2(mid_yo - cy, mid_xo - cx))
            for i in range(1, max(len(yellow_cones), len(blue_cones))):
                mid_y = (yellow_cones[i].y + blue_cones[i].y) / 2
                mid_x = (yellow_cones[i].x + blue_cones[i].x) / 2
                main_points.append((mid_x, mid_y))
                yaw_dir.append(math.atan2(main_points[i + 1][1] - main_points[i][1], main_points[i + 1][0] - main_points[i][0])) 
            
            #get the slop of the normal on the line connecting the last two cones
            final_yaw = (math.atan2(yellow_cones[-1].y - blue_cones[-1].y, yellow_cones[-1].x - blue_cones[-1].x) + math.pi/2)
            yaw_dir.append(final_yaw)
        
        #if only one side cones, move right the blue cones or left the yellow cones

        if remaining_yellow and not remaining_blue:
            yellow_cones = remaining_yellow
            for i in range(len(yellow_cones)):
                offset = 1.0  # meters to the left of yellow cones
                angle = math.atan2(yellow_cones[i].y - main_points[i][1], yellow_cones[i].x - main_points[i][0]) + math.pi / 2
                mid_x = yellow_cones[i].x + offset * math.cos(angle)
                mid_y = yellow_cones[i].y + offset * math.sin(angle)
                main_points.append((mid_x, mid_y))
                yaw_dir.append(math.atan2(main_points[i + 1][1] - main_points[i][1], main_points[i + 1][0] - main_points[i][0]))
            #final direction same as last cone
            final_yaw = math.atan2(yellow_cones[-1].y - cy, yellow_cones[-1].x - cx)
            yaw_dir.append(final_yaw)
        
        elif remaining_blue and not remaining_yellow:
            blue_cones = remaining_blue
            for i in range(len(blue_cones)):
                offset = 1.0  # meters to the right of blue cones
                angle = math.atan2(blue_cones[i].y - main_points[i][1], blue_cones[i].x - main_points[i][0]) - math.pi / 2
                mid_x = blue_cones[i].x + offset * math.cos(angle)
                mid_y = blue_cones[i].y + offset * math.sin(angle)
                main_points.append((mid_x, mid_y))
                yaw_dir.append(math.atan2(main_points[i + 1][1] - main_points[i][1], main_points[i + 1][0] - main_points[i][0]))
            #final direction same as last cone
            final_yaw = math.atan2(blue_cones[-1].y - cy, blue_cones[-1].x - cx)
            yaw_dir.append(final_yaw)  
        
        #number of steps to take between each two midpoints
        num_steps = []
        for j in range(1, len(main_points)):
            dist = math.hypot(main_points[j][0] - main_points[j - 1][0], main_points[j][1] - main_points[j - 1][1])
            num_steps.append(int(dist / step))
            print(f"num_steps debug: dist={dist} steps={int(dist / step)}")
        num_steps.append(int((max_dist - sum(num_steps) * step) / step))
            
            # debug print - run your scenario and read these numbers
        print(f"main_points={len(main_points)} yaw_dir={len(yaw_dir)} num_steps={len(num_steps)} ") 
        
        #generate path points
        total_path_length = 0.0
        for j in range(len(main_points)):
            last_point = main_points[j]
            for i in range(num_steps[j] + 1):
                dx = math.cos(yaw_dir[j]) * step * i
                dy = math.sin(yaw_dir[j]) * step * i
                new_point = (main_points[j][0] + dx, main_points[j][1] + dy)
                next_dist = math.hypot(new_point[0] - last_point[0], new_point[1] - last_point[1]) 
                if total_path_length + next_dist > max_dist:
                    break
                total_path_length += next_dist
                last_point = new_point
                path.append(new_point)
        
        # debug print - run your scenario and read these numbers
        print(f"total_path_length={total_path_length}")

        return path



