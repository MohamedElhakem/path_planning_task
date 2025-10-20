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
        #enter 1 for real cones and 0 for virtual cones
        def sort_by_chain_front(cones, cx, cy, heading, type):
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
            front_cones = []
            if type == 1: 
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
            if not blue or not yellow:  
                return blue, yellow, blue, yellow

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

        syellow_cones = sort_by_chain_front(
            [c for c in self.cones if c.color == 0],
            cx, cy, heading, 1
        )
        sblue_cones = sort_by_chain_front(
            [c for c in self.cones if c.color == 1],
            cx, cy, heading, 1
        )   

        # Debug print - run your scenario and read these numbers
        print(f"yellow_cones={len(syellow_cones)} blue_cones={len(sblue_cones)}")

        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.
        path: Path2D = []
        max_dist = 10 #meters
        step = 0.1 #meters

        #if no cones, go straight
        if not syellow_cones and not sblue_cones:
            last_point = (cx, cy)
            yaw = self.car_pose.yaw
            steps = int(max_dist / step)
            total_path_length = 0.0
            for i in range(steps + 1):
                dx = math.cos(yaw) * step
                dy = math.sin(yaw) * step
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
        
        main_points: list[CarPose] = []
        main_points.append(CarPose(cx, cy, None))
        
        #flitering cones to the same number on each side
        filtered_blue, filtered_yellow, remaining_blue, remaining_yellow = match_cones(sblue_cones, syellow_cones)
        yellow_cones = filtered_yellow
        blue_cones = filtered_blue
        
        #list of mid points
        if yellow_cones and blue_cones: 
            #calculate mid points and yaw directions
            mid_yo = (yellow_cones[0].y + blue_cones[0].y) / 2
            mid_xo = (yellow_cones[0].x + blue_cones[0].x) / 2
            yaw = (math.atan2(mid_yo - main_points[0].y, mid_xo - main_points[0].x))
            main_points[-1].yaw = yaw
            main_points.append(CarPose(mid_xo, mid_yo, yaw))
            for i in range(1, max(len(yellow_cones), len(blue_cones))):
                mid_y = (yellow_cones[i].y + blue_cones[i].y) / 2
                mid_x = (yellow_cones[i].x + blue_cones[i].x) / 2
                yaw = (math.atan2(mid_y - main_points[i].y, mid_x - main_points[i].x)) 
                main_points.append(CarPose(mid_x, mid_y, yaw))
                main_points[i].yaw = yaw
            
            if not remaining_yellow and not remaining_blue:
                #get the slop of the normal on the line connecting the last two cones
                main_points[-1].yaw = (math.atan2(yellow_cones[-1].y - blue_cones[-1].y, yellow_cones[-1].x - blue_cones[-1].x) + math.pi/2)
        
        #if only one side cones, move right the blue cones or left the yellow cones

        if remaining_yellow and not remaining_blue:
            yellow_cones = remaining_yellow
            for i in range(len(yellow_cones)):
                offset = 0.75 # meters to the left of yellow cones
                angle = math.atan2(yellow_cones[i].y - main_points[i].y, yellow_cones[i].x - main_points[i].x) + math.pi / 2
                mid_x = yellow_cones[i].x + offset * math.cos(angle)
                mid_y = yellow_cones[i].y + offset * math.sin(angle)
                yaw = (math.atan2(mid_y- main_points[i].y, mid_x- main_points[i].x))
                main_points[-1].yaw = yaw
                main_points.append(CarPose(mid_x, mid_y, yaw))

        elif remaining_blue and not remaining_yellow:
            blue_cones = remaining_blue
            for i in range(len(blue_cones)):
                offset = 0.75  # meters to the right of blue cones
                angle = math.atan2(blue_cones[i].y - main_points[i].y, blue_cones[i].x - main_points[i].x) - math.pi / 2
                mid_x = blue_cones[i].x + offset * math.cos(angle)
                mid_y = blue_cones[i].y + offset * math.sin(angle)
                yaw = (math.atan2(mid_y - main_points[i].y, mid_x - main_points[i].x))
                main_points[-1].yaw = yaw
                main_points.append(CarPose(mid_x, mid_y, yaw))

        final_yaw = main_points[-1].yaw
        #sort main points
        main_points = sort_by_chain_front(
            main_points,
            cx, cy, heading, 0
        )
        
        #new yaw directions
        yaw_dir = []
        for j in range(1, len(main_points)):
            yaw = math.atan2(main_points[j].y - main_points[j - 1].y, main_points[j].x - main_points[j - 1].x)
            yaw_dir.append(yaw)
        yaw_dir.append(final_yaw)  # last point yaw
        for j in range(len(main_points)):
            main_points[j].yaw = yaw_dir[j]
        #Edit the path so blue cones and yellow cones are on the correct side
        for i in range(len(main_points) - 1):
            p1 = main_points[i]
            p2 = main_points[i + 1] 
            offset =0.75  # meters
            #vector from p1 to p2
            vx = (p2.x - p1.x)
            vy = (p2.y - p1.y)
            #blue cones on the left side
            if sblue_cones:
                for j in range(len(sblue_cones)):
                    #in the segement between p1 and p2
                    if min(p1.x, p2.x) <= sblue_cones[j].x <= max(p1.x, p2.x) and min(p1.y, p2.y) <= sblue_cones[j].y <= max(p1.y, p2.y):
                        #vector from p1 to blue cone
                        bx = (sblue_cones[j].x - p1.x)
                        by = (sblue_cones[j].y - p1.y)
                        #cross product to determine side
                        cross = vx * by - vy * bx
                        if cross <= 0:
                            #new main point at the vertex of the rectangle
                            angle = math.atan2(by, bx) - math.pi / 2
                            new_x = sblue_cones[j].x + offset * math.cos(angle)
                            new_y = sblue_cones[j].y + offset * math.sin(angle)
                            yaw_i = math.atan2(new_y - p1.y, new_x - p1.x)
                            yaw_new = math.atan2(p2.y - new_y, p2.x - new_x)
                            main_points[i].yaw = yaw_i
                            main_points.insert(i + 1, CarPose(new_x, new_y, yaw_new))
            #yellow cones on the right side
            if syellow_cones:
                for j in range(len(syellow_cones)):
                    #in the segement between p1 and p2
                    if min(p1.x, p2.x) <= syellow_cones[j].x <= max(p1.x, p2.x) and min(p1.y, p2.y) <= syellow_cones[j].y <= max(p1.y, p2.y):
                        #vector from p1 to yellow cone
                        yx = (syellow_cones[j].x - p1.x)
                        yy = (syellow_cones[j].y - p1.y)
                        #cross product to determine side
                        cross = vx * yy - vy * yx
                        if cross >= 0:
                            #new main point at the vertex of the rectangle
                            angle = math.atan2(yy, yx) + math.pi / 2
                            new_x = syellow_cones[j].x + offset * math.cos(angle)
                            new_y = syellow_cones[j].y + offset * math.sin(angle)
                            yaw_i = math.atan2(new_y - p1.y, new_x - p1.x)
                            yaw_new = math.atan2(p2.y - new_y, p2.x - new_x)
                            main_points[i].yaw = yaw_i
                            main_points.insert(i + 1, CarPose(new_x, new_y, yaw_new))
    
        #number of steps to take between each two midpoints
        num_steps = []
        for j in range(1, len(main_points)):
            dist = math.hypot(main_points[j].y - main_points[j - 1].y, main_points[j].x - main_points[j - 1].x)
            num_steps.append(int(dist / step))
            print(f"num_steps debug: dist={dist} steps={int(dist / step)} , between point {j - 1} and point {j }")
        num_steps.append(int((max_dist - sum(num_steps) * step) / step))
            
            # debug print - run your scenario and read these numbers
        print(f"main_points={len(main_points)} edges={len(num_steps)} num_steps={sum(num_steps)} ") 
        
        #generate path points
        total_path_length = 0.0
        for j in range(len(main_points)):
            last_point = (main_points[j].x, main_points[j].y)
            for i in range(num_steps[j]):
                dx = math.cos(main_points[j].yaw) * step
                dy = math.sin(main_points[j].yaw) * step 
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



