import math
import random
import numpy as np
import time
import pandas as pd
from PIL import Image, ImageOps
import matplotlib.pyplot as plt

class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None
        self.cost = 0.0

class Tree():
    def __init__(self, start_node):
        self.nodes = [start_node]

    def add_node(self, new_node, parent_node):
        self.nodes.append(new_node)
        parent_node.children.append(new_node)
        new_node.parent = parent_node

    def nearest(self,new_node):
        best = None
        min_dist = float('inf')
        for node in self.nodes:
            dist = math.dist((node.locationX, node.locationY),(new_node.locationX, new_node.locationY))
            if dist < min_dist:
                min_dist = dist
                best = node
        return best

    def path_recovery(self,final_node):
        path = []
        while final_node.parent is not None:
            path.append(final_node)
            final_node = final_node.parent
        return path

    def distance(self,node1,node2):
        return math.dist((node1.locationX,node1.locationY), (node2.locationX,node2.locationY))
    
    def near(self, node_mid, radius):
        nodes_list= []
        for node in self.nodes:
            if self.distance(node_mid, node) <= radius:
                nodes_list.append(node)
        return nodes_list
    
    def rewire(self, child, new_parent):
        if child.parent:
            child.parent.children.remove(child)
        child.parent = new_parent
        new_parent.children.append(child)
        
    def update_descendant_costs(self, node):
        if node.parent:
            new_cost = node.parent.cost + self.distance(node.parent, node)
            if abs(node.cost - new_cost) > 1e-6:
                node.cost = new_cost
                for child in node.children:
                    self.update_descendant_costs(child)

def collision_free(p1,p2,mapa):
    x1, y1 = p1.locationX, p1.locationY
    x2, y2 = p2.locationX, p2.locationY

    h, w = mapa.shape
    
    if not (0 <= x1 < w and 0 <= y1 < h and 0 <= x2 < w and 0 <= y2 < h):
        return False

    num_points = int(max(abs(x1-x2), abs(y1-y2)))

    if num_points == 0:
        return True

    for i in range(num_points + 1):
        t = i / max(num_points, 1)
        x = round(x1 + (x2 - x1) * t)
        y = round(y1 + (y2 - y1) * t)
        
        if not (0 <= x < w and 0 <= y < h):
            return False 

        if mapa[y, x] == 1:
            return False
    return True

def get_ellipse_params(start, goal, par_1=1.5, par_2=0.4):
    d = math.dist(start, goal)
    a = par_1 * d
    b = par_2 * d
    sr_x = (start[0] + goal[0]) / 2
    sr_y = (start[1] + goal[1]) / 2
    theta = math.atan2(goal[1] - start[1], goal[0] - start[0])
    return sr_x, sr_y, a, b, theta

def samp_point_elipse(start, goal, mapa, par_1 = 1.5, par_2 = 0.4):
    while True:
        d = math.dist(start,goal)
        a = par_1 * d
        b = par_2 * d
        sr_x = (start[0] + goal[0])  / 2
        sr_y = (start[1] + goal[1])  / 2
        theta = math.atan2(goal[1] - start[1], goal[0] - start[0])

        phi = random.uniform(0, 2 * math.pi)
        r = math.sqrt(random.uniform(0, 1))
        x_norm= a * r * math.cos(phi)
        y_norm = b * r * math.sin(phi)

        X = x_norm * math.cos(theta) - y_norm * math.sin(theta) + sr_x
        Y = x_norm * math.sin(theta) + y_norm * math.cos(theta) + sr_y

        if 0 <= X < mapa.shape[1] and 0 <= Y < mapa.shape[0]:
            if mapa[int(Y), int(X)] == 0:
                return treeNode(X, Y)


def tests_rrt_solver(start, goal, mapa, step_len=25, max_iter=1000, tolerance=30, bias_mode='FIXED', is_rrt_star=False, fixed_bias=0.9, pmax=0.4, pmin=0.05):
    
    start_time_total = time.time()
    
    start_node = treeNode(start[0], start[1])
    tree = Tree(start_node)

    RRT_STAR_RADIUS = 5 * step_len
    success = 0
    pa = pmin

    for i in range(1, max_iter + 1):
        if bias_mode != 'FIXED':
            ps = success / i
            if ps < 1e-6:
                ps = 1e-6

            if bias_mode == 'AIW':
                pa = pmin + (pmax - pmin) * ps
            elif bias_mode == 'HYBRID':
                time_factor = (max_iter - i) / max_iter 
                term_1 = (pmax - pmin) / ps
                term_2 = pmin / ps
                pa = 1 - (term_1 + time_factor * term_2)
            pa = np.clip(pa, pmin, pmax)
        else: 
            pa = fixed_bias
            ps = success / i

        if random.random() < pa:
            sample_node = treeNode(goal[0], goal[1])
        else:
            sample_node = samp_point_elipse(start, goal, mapa)

        pot_parent = tree.nearest(sample_node)
        distance = tree.distance(pot_parent, sample_node)

        if distance <= step_len:
            new_node = sample_node
        else:
            stepX = ((sample_node.locationX - pot_parent.locationX) / distance) * step_len
            stepY = ((sample_node.locationY - pot_parent.locationY) / distance) * step_len
            new_node = treeNode(pot_parent.locationX + stepX, pot_parent.locationY + stepY)

        final_parent = pot_parent
        new_cost = 0.0
        if pot_parent:
            new_cost = pot_parent.cost + tree.distance(pot_parent, new_node)

        if is_rrt_star:
            near_nodes = tree.near(new_node, RRT_STAR_RADIUS)
            min_cost = new_cost
            best_parent = final_parent

            for near_node in near_nodes:
                potential_cost = near_node.cost + tree.distance(near_node, new_node)
                if potential_cost < min_cost and collision_free(near_node, new_node, mapa):
                    min_cost = potential_cost
                    best_parent = near_node
            
            final_parent = best_parent
            new_cost = min_cost
        
        new_node.cost = new_cost

        if final_parent and collision_free(final_parent, new_node, mapa):
            tree.add_node(new_node, final_parent)
            success += 1

            if is_rrt_star:
                for near_node in near_nodes:
                    rewire_cost = new_node.cost + tree.distance(new_node, near_node)
                    if rewire_cost < near_node.cost and collision_free(new_node, near_node, mapa):
                        tree.rewire(near_node, new_node)
                        tree.update_descendant_costs(near_node)

            if tree.distance(new_node, treeNode(goal[0], goal[1])) < tolerance:
                goal_node = treeNode(goal[0], goal[1])
                goal_node.cost = new_node.cost + tree.distance(new_node, goal_node)
                tree.add_node(goal_node, new_node)
                
                end_time_total = time.time()
                return {
                        "iterations": i,
                        "cost": goal_node.cost,
                        "success": True,
                        "time_sec": end_time_total - start_time_total,
                        "pa": pa
                        }
    
    end_time_total = time.time()
    return {
        "iterations": max_iter,
        "cost": float('inf'),
        "success": False,
        "time_sec": end_time_total - start_time_total,
        "pa": pa
    }

def load_map_and_preprocess(filepath, start_coords, goal_coords):
    """Wczytuje i przetwarza obraz mapy."""
    try:
        img = Image.open(filepath)
        img = ImageOps.grayscale(img)
        img = ImageOps.invert(img)
        np_img = np.array(img)
        np_img = (np_img > 128).astype(np.uint8)
        return np_img, start_coords, goal_coords
    except FileNotFoundError:
        print(f"BŁĄD: Nie znaleziono pliku {filepath}")
        return None, None, None


def main_benchmark(num_trials=30, max_iter=1500, output_file='rrt_benchmark_results.csv'):
    
    maps_data = {
        'mapa2': {'path': './images/mapa2.png', 'start': (1190, 80), 'goal': (100, 800)},
        'mapa1': {'path': './images/mapa1.png', 'start': (1190, 80), 'goal': (100, 800)}, 
        'mapa3': {'path': './images/mapa3.png', 'start': (1190, 80), 'goal': (100, 800)},
    }
    
    configs = [
        {'bias': 'FIXED', 'star': False, 'name': 'RRT (Fixed)'},
        {'bias': 'AIW', 'star': False, 'name': 'RRT (AIW)'},
        {'bias': 'HYBRID', 'star': False, 'name': 'RRT (Hybrid)'},
        {'bias': 'FIXED', 'star': True, 'name': 'RRT* (Fixed)'},
        {'bias': 'AIW', 'star': True, 'name': 'RRT* (AIW)'},
        {'bias': 'HYBRID', 'star': True, 'name': 'RRT* (Hybrid)'},
    ]

    all_results = []
    
    print(f"--- ROZPOCZYNAM TESTY ---")
    print(f"Konfiguracja: {len(maps_data)} map x {len(configs)} wariantów x {num_trials} prób")

    for map_name, data in maps_data.items():
        mapa_array, start, goal = load_map_and_preprocess(data['path'], data['start'], data['goal'])
        
        if mapa_array is None:
            continue
            
        print(f"\nMapa: {map_name}")

        for config in configs:
            print(f"  > Test: {config['name']}...", end="")
            
            for trial in range(num_trials):
                result = tests_rrt_solver(
                    start=start, 
                    goal=goal, 
                    mapa=mapa_array, 
                    max_iter=max_iter,
                    bias_mode=config['bias'], 
                    is_rrt_star=config['star']
                )
                
                all_results.append({
                    'Map': map_name,
                    'Algorithm': config['name'],
                    'Is_RRT_Star': config['star'],
                    'Bias_Mode': config['bias'],
                    'Trial': trial,
                    'Success': result['success'],
                    'Iterations': result['iterations'],
                    'Path_Cost': result['cost'],
                    'Time_sec': result['time_sec']
                })
            
            print("OK")
                
    results_df = pd.DataFrame(all_results)
    
    results_df.to_csv(output_file, index=False)
    
    results_df.to_pickle(output_file.replace('.csv', '.pkl')) 
    
    print("\n--- TESTY ZAKOŃCZONE ---")
    print(f"Dane zapisano w pliku: {output_file} (oraz w pliku .pkl)")
    
    return results_df

if __name__ == '__main__':
    df = main_benchmark(num_trials=30)
    
    summary = df.groupby(['Map', 'Algorithm']).agg(
        Mean_Iters=('Iterations', 'mean'),
        Mean_Cost=('Path_Cost', lambda x: x[x != float('inf')].mean()),
        Success_Rate=('Success', 'sum')
    )
    print("\n--- PODSUMOWANIE WYNIKÓW ---")
    print(summary)