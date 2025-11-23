import requests
import json
import math
import random
import time
import sys

# --- Constants ---
BBOX_LARGE = [19.0600, 72.8500, 19.1000, 72.9000]  # Mumbai area
BBOX_SMALL = [19.0700, 72.8700, 19.0800, 72.8800]

ALLOWED_ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]
ALLOWED_POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]

QUERY_COUNT_PHASE_1_2_3 = 500
QUERY_COUNT_PHASE_3_SMALL = 200

OSM_MAPPING = {
    "motorway": "expressway", "trunk": "expressway",
    "primary": "primary", "secondary": "secondary", "tertiary": "tertiary",
    "unclassified": "local", "residential": "local", "service": "local",
    "footway": "local", "path": "local", "cycleway": "local" 
}
# Speeds in meters per second (m/s)
SPEED_OPTS_MS = {"expressway": 27.78, "primary": 16.67, "secondary": 13.89, "tertiary": 11.11, "local": 8.33}


# --- Core Functions ---

def haversine_meters(lat1, lon1, lat2, lon2):
    """Calculates the distance between two points on the Earth in meters."""
    R = 6371000.0  # Earth's radius in meters
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
    
    # Clamp 'a' to [0, 1] due to floating point inaccuracies near poles or antipodes
    if a > 1.0: a = 1.0
    if a < 0.0: a = 0.0
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def generate_speed_profile(avg_speed_ms):
    """Generates a 96-step (15-min intervals for a day) speed profile with +/- 20% variation."""
    return [round(avg_speed_ms * random.uniform(0.8, 1.2), 4) for _ in range(96)]


def generate_synthetic_graph(bbox, max_nodes=10000):
    """Generates a random graph when Overpass API fails."""
    print("Generating synthetic graph as API fallback...")
    
    num_nodes = min(max_nodes, 1000) 
    nodes = []
    edges = []
    
    min_lat, min_lon, max_lat, max_lon = bbox
    for i in range(num_nodes):
        lat = random.uniform(min_lat, max_lat)
        lon = random.uniform(min_lon, max_lon)
        
        pois = []
        global ALLOWED_POIS 
        if random.random() < 0.15: 
            pois = random.sample(ALLOWED_POIS, k=random.randint(1, min(3, len(ALLOWED_POIS))))

        nodes.append({
            "id": i,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "pois": pois
        })

    # --- FIX 1: Start edge_counter at 0 for continuous 0-indexing ---
    edge_counter = 0 
    # -------------------------------------------------------------
    global SPEED_OPTS_MS
    global ALLOWED_ROAD_TYPES
    seen_pairs = set()

    for i in range(num_nodes):
        # Prevent self-loops
        all_other_nodes = [j for j in range(num_nodes) if i != j]
        if not all_other_nodes: continue

        num_neighbors = random.randint(1, 3)
        neighbors = random.sample(all_other_nodes, min(num_neighbors, len(all_other_nodes)))
        
        for j in neighbors:
            n1, n2 = nodes[i], nodes[j]
            dist = haversine_meters(n1['lat'], n1['lon'], n2['lat'], n2['lon'])
            
            if dist < 5: continue
            
            road_type = random.choice(ALLOWED_ROAD_TYPES)
            speed_ms = SPEED_OPTS_MS[road_type]
            time_avg = round(dist / speed_ms, 3)
            
            # Ensure unique edges
            if (i, j) not in seen_pairs:
                edges.append({
                    "id": edge_counter, # Now starts at 0
                    "u": i, "v": j,
                    "length": round(dist, 3),
                    "average_time": time_avg,
                    "road_type": road_type,
                    "oneway": random.choice([True, False]),
                    "speed_profile": generate_speed_profile(speed_ms) 
                })
                seen_pairs.add((i, j))
                edge_counter += 1

                if not edges[-1]['oneway'] and (j, i) not in seen_pairs:
                    edges.append({
                        "id": edge_counter, # Now increments from 0
                        "u": j, "v": i,
                        "length": round(dist, 3),
                        "average_time": time_avg,
                        "road_type": road_type,
                        "oneway": False,
                        "speed_profile": generate_speed_profile(speed_ms)
                    })
                    seen_pairs.add((j, i))
                    edge_counter += 1
    
    print(f"Synthetic graph created: {len(nodes)} nodes, {len(edges)} edges.")
    return {"meta": {"id": "synthetic_fallback", "description": "Synthetic graph due to API failure"}, "nodes": nodes, "edges": edges}


def fetch_and_process_graph(bbox, filename):
    """Fetches map data from Overpass API and processes it into a simulation graph format."""
    min_lat, min_lon, max_lat, max_lon = bbox
    query = f"""
    [out:json];
    (
        way["highway"]~"^(motorway|trunk|primary|secondary|tertiary|residential|unclassified|service|footway|path|cycleway)"({min_lat},{min_lon},{max_lat},{max_lon});
        node(w);
    );
    out body; >; out skel qt;
    """
    print(f"Fetching map data for {filename} from Overpass...")
    
    try:
        max_retries = 3
        for attempt in range(max_retries):
            # Increase timeout for potentially slow Overpass queries
            resp = requests.post("http://overpass-api.de/api/interpreter", data={'data': query}, timeout=60) 
            if resp.status_code == 200:
                data = resp.json()
                break
            else:
                print(f"API Request failed (Status: {resp.status_code}). Retrying in {2**attempt}s...")
                time.sleep(2**attempt)
        else:
            raise requests.exceptions.RequestException("API failed after maximum retries.")

    except requests.exceptions.RequestException as e:
        print(f"API Request failed: {e}. Falling back to synthetic data.")
        return generate_synthetic_graph(bbox, max_nodes=5000 if "2" in filename else 10000)
    
    # --- 1. Process Nodes ---
    osm_nodes = {}
    for el in data.get('elements', []):
        if el['type'] == 'node':
            osm_nodes[el['id']] = {"lat": el['lat'], "lon": el['lon']}

    node_map = {}
    final_nodes = []
    current_id = 0
    
    # Find all nodes that are part of a road way
    used_ids = set()
    ways = [el for el in data.get('elements', []) if el['type'] == 'way']
    for w in ways:
        for n in w.get('nodes', []): used_ids.add(n)

    global OSM_MAPPING, ALLOWED_POIS 
    for oid in used_ids:
        if oid in osm_nodes:
            # Assign new, sequential ID (Node IDs are already 0-indexed and continuous)
            node_map[oid] = current_id 
            
            # Add POIs randomly
            pois = []
            if random.random() < 0.1:
                pois = random.sample(ALLOWED_POIS, k=random.randint(1, min(2, len(ALLOWED_POIS))))
            
            final_nodes.append({
                "id": current_id,
                "lat": round(osm_nodes[oid]['lat'], 6),
                "lon": round(osm_nodes[oid]['lon'], 6),
                "pois": pois
            })
            current_id += 1

    # --- 2. Process Edges ---
    final_edges = []
    # --- FIX 2: Start edge_counter at 0 for continuous 0-indexing ---
    edge_counter = 0
    # -------------------------------------------------------------
    seen_pairs = set()

    global SPEED_OPTS_MS, ALLOWED_ROAD_TYPES
    for w in ways:
        tags = w.get('tags', {})
        raw_type = tags.get('highway', 'local')
        rtype = OSM_MAPPING.get(raw_type, "local")
        
        if rtype not in ALLOWED_ROAD_TYPES: continue

        oneway = tags.get('oneway', 'no') in ('yes', 'true', '1')
        speed_ms = SPEED_OPTS_MS[rtype]

        nodes = w.get('nodes', [])
        for i in range(len(nodes)-1):
            u_osm, v_osm = nodes[i], nodes[i+1]
            # Ensure both OSM nodes exist and are in the final graph
            if u_osm not in node_map or v_osm not in node_map: continue
            
            u, v = node_map[u_osm], node_map[v_osm]
            
            n1, n2 = osm_nodes[u_osm], osm_nodes[v_osm]
            dist = haversine_meters(n1['lat'], n1['lon'], n2['lat'], n2['lon'])
            if dist < 0.1: continue # Skip very short segments
            
            time_avg = round(dist / speed_ms, 3)
            
            def add_e(src, dst, is_oneway):
                nonlocal edge_counter
                if (src, dst) in seen_pairs: return
                final_edges.append({
                    "id": edge_counter, # Now starts at 0
                    "u": src, "v": dst,
                    "length": round(dist, 3), "average_time": time_avg,
                    "road_type": rtype, "oneway": is_oneway,
                    "speed_profile": generate_speed_profile(speed_ms) 
                })
                seen_pairs.add((src, dst))
                edge_counter += 1

            # Add forward edge
            add_e(u, v, oneway)
            # Add reverse edge if not oneway
            if not oneway: add_e(v, u, False)

    print(f"Graph processed: {len(final_nodes)} nodes, {len(final_edges)} edges.")
    return {"meta": {"id": filename.replace(".json", "")}, "nodes": final_nodes, "edges": final_edges}


# --- Query Generation Functions ---

def get_random_nodes(nodes, count=2):
    """Helper to get random node IDs."""
    if len(nodes) < count:
        return [n['id'] for n in nodes]
    return random.sample([n['id'] for n in nodes], count)

def generate_shortest_path_query(nodes, q_id):
    """Generates a shortest path query with optional constraints."""
    if len(nodes) < 2: return None
    
    src, tgt = random.sample([n['id'] for n in nodes], 2)
    q = {
        "id": q_id,
        "type": "shortest_path",
        "source": src,
        "target": tgt,
        "mode": random.choice(["time", "distance"])
    }
    
    global ALLOWED_ROAD_TYPES
    if random.random() < 0.3:
        q["constraints"] = {}
        if random.random() < 0.5:
            # Forbidden nodes constraint
            available_nodes = [n['id'] for n in nodes if n['id'] not in (src, tgt)]
            if available_nodes:
                q["constraints"]["forbidden_nodes"] = random.sample(available_nodes, k=random.randint(1, min(3, len(available_nodes))))
        else:
            # Forbidden road types constraint
            q["constraints"]["forbidden_road_types"] = random.sample(ALLOWED_ROAD_TYPES, k=random.randint(1, 2))

    return q

def generate_knn_query(nodes, bbox, q_id):
    """Generates a k-nearest-neighbors (k-NN) query."""
    min_lat, min_lon, max_lat, max_lon = bbox
    global ALLOWED_POIS
    return {
        "id": q_id,
        "type": "knn",
        "poi": random.choice(ALLOWED_POIS),
        "query_point": {
            "lat": round(random.uniform(min_lat, max_lat), 6),
            "lon": round(random.uniform(min_lon, max_lon), 6)
        },
        "k": random.randint(1, 5),
        "metric": random.choice(["shortest_path", "euclidean"])
    }

def generate_dynamic_update(edges, q_id):
    """Generates an edge modification or removal event."""
    if not edges: return None
    edge = random.choice(edges)
    update_type = random.choice(["modify_edge", "remove_edge"])
    
    if update_type == "remove_edge":
        return {
            "id": q_id,
            "type": "remove_edge",
            "edge_id": edge['id']
        }
    else: 
        patch_field = random.choice(["length", "road_type"])
        patch = {}
        
        global ALLOWED_ROAD_TYPES
        if patch_field == "length":
            patch["length"] = round(edge['length'] * random.uniform(0.5, 2.0), 3)
        elif patch_field == "road_type":
            # Select a different road type
            available_types = [r for r in ALLOWED_ROAD_TYPES if r != edge['road_type']]
            if available_types:
                patch["road_type"] = random.choice(available_types)
        
        return {
            "id": q_id,
            "type": "modify_edge",
            "edge_id": edge['id'],
            "patch": patch
        }

def generate_phase1_queries(graph, bbox, count):
    """Generates a mix of simple queries and dynamic updates."""
    nodes = graph['nodes']
    edges = graph['edges']
    if not nodes or not edges: return {"events": []}

    queries = []
    for i in range(1, count + 1):
        q_type = random.choices(["shortest_path", "knn", "dynamic_update"], weights=[0.6, 0.2, 0.2], k=1)[0]
        
        q = None
        if q_type == "shortest_path":
            q = generate_shortest_path_query(nodes, i)
        elif q_type == "knn":
            q = generate_knn_query(nodes, bbox, i)
        elif q_type == "dynamic_update":
            q = generate_dynamic_update(edges, i)

        if q: queries.append(q)
    
    return {"meta": {"id": "queries-phase1"}, "events": queries}

def generate_phase2_queries(graph, count):
    """Generates advanced pathfinding queries (k-shortest, approximate)."""
    nodes = graph['nodes']
    if len(nodes) < 2: return {"events": []}

    queries = []
    i = 1
    
    while i <= count:
        q_type = random.choices(["k_shortest_paths_exact", "k_shortest_paths_heuristic", "approx_shortest_path"], weights=[0.4, 0.3, 0.3], k=1)[0]
        
        q = None
        if len(nodes) < 2: break 

        src, tgt = random.sample([n['id'] for n in nodes], 2)

        if q_type == "k_shortest_paths_exact":
            q = {
                "id": i,
                "type": "k_shortest_paths",
                "source": src,
                "target": tgt,
                "k": random.randint(2, 20),
                "mode": "distance"
            }
            i += 1
        
        elif q_type == "k_shortest_paths_heuristic":
            q = {
                "id": i,
                "type": "k_shortest_paths_heuristic",
                "source": src,
                "target": tgt,
                "k": random.randint(2, 7),
                "overlap_threshold": random.randint(20, 80)
            }
            i += 1

        elif q_type == "approx_shortest_path":
            num_sub_queries = random.randint(5, 10)
            sub_queries = []
            
            for _ in range(num_sub_queries):
                if len(nodes) < 2: break
                sub_src, sub_tgt = random.sample([n['id'] for n in nodes], 2)
                sub_queries.append({"source": sub_src, "target": sub_tgt})
            
            if sub_queries:
                q = {
                    "id": i,
                    "type": "approx_shortest_path",
                    "queries": sub_queries,
                    "time_budget_ms": random.choice([5, 10, 15, 20]),
                    "acceptable_error_pct": random.choice([5.0, 10.0, 15.0])
                }
                i += 1
        
        if q: queries.append(q)
    
    return {"meta": {"id": "queries-phase2"}, "events": queries}


def generate_phase3_queries(graph, count, filename):
    """Generates multi-driver, multi-order Vehicle Routing Problem (VRP) queries."""
    node_ids = [n['id'] for n in graph['nodes']]
    
    # Needs at least one depot node + 2 nodes for the smallest order (pickup/dropoff)
    if len(node_ids) < 3: return {"events": []} 

    queries = []
    
    for i in range(1, count + 1):
        # Determine the maximum possible number of unique orders
        max_possible_orders = (len(node_ids) - 1) // 2
        if max_possible_orders < 1: continue

        num_orders = random.randint(1, min(5, max_possible_orders))
        num_drivers = random.randint(1, 3)
        unique_points_needed = 2 * num_orders
        
        depot = random.choice(node_ids)
        available_nodes = [n for n in node_ids if n != depot]
        
        # Select unique pickup/dropoff nodes from non-depot nodes
        try:
            # This is guaranteed to succeed because num_orders is capped by max_possible_orders
            order_nodes = random.sample(available_nodes, unique_points_needed)
        except ValueError:
            # Should not happen, but serves as a safety break
            continue

        orders = []
        for j in range(num_orders):
            pickup = order_nodes[j]
            dropoff = order_nodes[j + num_orders] # Takes the node from the second half of the list
            
            orders.append({
                "order_id": j + 1,
                "pickup": pickup,
                "dropoff": dropoff
            })

        q = {
            "id": i,
            "type": "delivery_scheduling",
            "orders": orders,
            "goal": random.choice(["minimize_total_time", "minimize_max_time"]),
            "fleet": {
                "num_delievery_guys": num_drivers,
                "depot_node": depot
            }
        }
        queries.append(q)
    
    return {"meta": {"id": filename.replace(".json", "")}, "events": queries}


def main():
    """Main function to fetch graph data and generate all query files."""
    if not all(hasattr(requests, func) for func in ['get', 'post']):
        print("Error: The 'requests' library is not available. Please ensure it is installed.")
        sys.exit(1)

    # --- Phase 1: Large Graph and Queries ---
    graph_data = fetch_and_process_graph(BBOX_LARGE, "graph.json")
    
    if graph_data['nodes'] and graph_data['edges']:
        queries_phase1_data = generate_phase1_queries(graph_data, BBOX_LARGE, QUERY_COUNT_PHASE_1_2_3)
        queries_phase2_data = generate_phase2_queries(graph_data, QUERY_COUNT_PHASE_1_2_3)
        queries_phase3_data = generate_phase3_queries(graph_data, QUERY_COUNT_PHASE_1_2_3, "queries-phase3.json")

        with open("graph.json", "w") as f:
            json.dump(graph_data, f, indent=4)
        print(f"\n✅ graph.json created ({len(graph_data['nodes'])} nodes, {len(graph_data['edges'])} edges)")

        with open("queries-phase1.json", "w") as f:
            json.dump(queries_phase1_data, f, indent=4)
        print(f"✅ queries-phase1.json created ({len(queries_phase1_data['events'])} queries)")

        with open("queries-phase2.json", "w") as f:
            json.dump(queries_phase2_data, f, indent=4)
        print(f"✅ queries-phase2.json created ({len(queries_phase2_data['events'])} queries)")

        with open("queries-phase3.json", "w") as f:
            json.dump(queries_phase3_data, f, indent=4)
        print(f"✅ queries-phase3.json created ({len(queries_phase3_data['events'])} queries)")
    else:
        print("\n⚠️ Skipping large query generation due to empty graph data.")

    # --- Phase 2: Small Graph and Queries (VRP Focus) ---
    graph_small_data = fetch_and_process_graph(BBOX_SMALL, "graph-2.json")

    if graph_small_data['nodes'] and graph_small_data['edges']:
        queries_phase3_small_data = generate_phase3_queries(graph_small_data, QUERY_COUNT_PHASE_3_SMALL, "queries-phase3-small.json")
        
        with open("graph-2.json", "w") as f:
            json.dump(graph_small_data, f, indent=4)
        print(f"\n✅ graph-2.json created ({len(graph_small_data['nodes'])} nodes, {len(graph_small_data['edges'])} edges)")

        with open("queries-phase3-small.json", "w") as f:
            json.dump(queries_phase3_small_data, f, indent=4)
        print(f"✅ queries-phase3-small.json created ({len(queries_phase3_small_data['events'])} queries)")
    else:
        print("\n⚠️ Skipping small graph generation due to empty graph data.")


if __name__ == "__main__":
    main()