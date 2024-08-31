import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import cv2
import copy

def load_depth_from_bag(bag_file, align_to_color=True):
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_device_from_file(bag_file, repeat_playback=False)

    pipeline.start(config)

    align = rs.align(rs.stream.color)

    try:
        frames = pipeline.wait_for_frames()
        if align_to_color:
            frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            raise RuntimeError("Failed to retrieve depth frame from .bag file")

        depth_image = np.asanyarray(depth_frame.get_data())

        # Get the depth scale from the depth sensor
        depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()

        return depth_image, depth_scale

    finally:
        pipeline.stop()

def generate_pointcloud_from_depth_mask(mask_file, depth_image, depth_scale, camera_intrinsics):
    mask = cv2.imread(mask_file, cv2.IMREAD_GRAYSCALE)

    if mask is None:
        raise ValueError(f"Could not load mask image from {mask_file}")

    # Masking non-brick areas
    depth_image = np.where(mask > 0, depth_image, 0)

    height, width = depth_image.shape

    fx = camera_intrinsics.fx
    fy = camera_intrinsics.fy
    cx = camera_intrinsics.ppx
    cy = camera_intrinsics.ppy

    points = []
    for v in range(height):
        for u in range(width):
            z = depth_image[v, u] * depth_scale * 100  # Convert to centimeters
            if z > 0:
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(np.array(points))

    return pc

def refine_pointcloud(pointcloud):
    voxel_size = 2  # Adjust voxel size as needed
    downsampled_pc = pointcloud.voxel_down_sample(voxel_size)
    
    # Remove statistical outliers
    cl, ind = downsampled_pc.remove_statistical_outlier(nb_neighbors=10000, std_ratio=1)
    refined_pc = downsampled_pc.select_by_index(ind)

    return refined_pc

def generate_pointcloud_from_ply(ply_file):
    return o3d.io.read_point_cloud(ply_file)

def apply_transform_to_pointcloud(pointcloud, transformation):
    transformed_pc = copy.deepcopy(pointcloud)
    transformed_pc.transform(transformation)
    return transformed_pc

def display_pointcloud(vis, pointcloud, hex_color):
    rgb_color = tuple(int(hex_color[i:i+2], 16)/255.0 for i in (0, 2, 4))
    colors = np.tile(rgb_color, (np.asarray(pointcloud.points).shape[0], 1))
    pointcloud.colors = o3d.utility.Vector3dVector(colors)
    vis.add_geometry(pointcloud)

def display_reference_system(vis, size=5.0):
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    vis.add_geometry(axis_frame)

def print_transformation_matrix(transformation):
    if not isinstance(transformation, np.ndarray) or transformation.shape != (4, 4):
        raise ValueError("Transformation must be a 4x4 numpy array.")
    
    rotation = transformation[:3, :3]
    translation = transformation[:3, 3]
    
    print("Transformation Matrix:")
    print("-" * 50)
    print("Rotation Matrix:")
    for row in rotation:
        print(f"[{row[0]: .6f}, {row[1]: .6f}, {row[2]: .6f}]")
    
    print("\nTranslation Vector:")
    print(f"[{translation[0]: .6f}, {translation[1]: .6f}, {translation[2]: .6f}]")
    print("-" * 50)
    
    euler_angles = rotation_matrix_to_euler_angles(rotation)
    print(f"Roll (X):  {np.degrees(euler_angles[0]): .2f}")
    print(f"Pitch (Y): {np.degrees(euler_angles[1]): .2f}")
    print(f"Yaw (Z):   {np.degrees(euler_angles[2]): .2f}")
    print("-" * 50)

def rotation_matrix_to_euler_angles(R):
    assert(is_rotation_matrix(R))
    
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def is_rotation_matrix(R):
    Rt = np.transpose(R)
    should_be_identity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - should_be_identity)
    return n < 1e-6

def fit_plane_to_pointcloud(pointcloud, plane_distance_threshold=1.0, min_inliers=20):
    planes = []
    remaining_pc = pointcloud

    for _ in range(3):
        plane_model, inliers = remaining_pc.segment_plane(distance_threshold=plane_distance_threshold, ransac_n=3, num_iterations=1000)
        if len(inliers) < min_inliers:
            continue
        inlier_cloud = remaining_pc.select_by_index(inliers)
        planes.append((plane_model, len(inliers), inlier_cloud))
        remaining_pc = remaining_pc.select_by_index(inliers, invert=True)

    return planes

def calculate_line_of_intersection(plane1, plane2):
    direction = np.cross(plane1[:3], plane2[:3])
    
    A = np.array([plane1[:3], plane2[:3]])
    B = -np.array([plane1[3], plane2[3]])
    
    if direction[0] != 0:
        A = A[:, 1:]
        point = np.linalg.solve(A, B)
        point = np.array([0, point[0], point[1]])
    elif direction[1] != 0:
        A = A[:, [0, 2]]
        point = np.linalg.solve(A, B)
        point = np.array([point[0], 0, point[1]])
    else:
        A = A[:, :2]
        point = np.linalg.solve(A, B)
        point = np.array([point[0], point[1], 0])

    return direction, point

def detect_edge_from_largest_planes(planes):
    known_lengths = [7.6, 15.0, 23.0]

    planes = sorted(planes, key=lambda x: x[1], reverse=True)

    if len(planes) < 2:
        return None, 0

    plane1, plane2 = planes[0][0], planes[1][0]

    line_dir, point_on_line = calculate_line_of_intersection(plane1, plane2)

    combined_inliers = np.vstack((np.asarray(planes[0][2].points), np.asarray(planes[1][2].points)))
    projected_points = project_points_on_line(combined_inliers, line_dir, point_on_line)
    
    min_point = projected_points.min(axis=0)
    max_point = projected_points.max(axis=0)
    detected_length = np.linalg.norm(max_point - min_point)

    closest_length = min(known_lengths, key=lambda x: abs(x - detected_length))

    edge_center = (min_point + max_point) / 2

    half_length = closest_length / 2

    line_eq = lambda t: point_on_line + t * line_dir

    new_min_point = edge_center - half_length * line_dir
    new_max_point = edge_center + half_length * line_dir

    new_extreme_points = np.array([new_min_point, new_max_point])

    return new_extreme_points, closest_length

def project_points_on_line(points, line_dir, point_on_line):
    line_dir = line_dir / np.linalg.norm(line_dir)
    projected_points = []
    
    for point in points:
        vector = np.array(point) - np.array(point_on_line)
        distance_along_line = np.dot(vector, line_dir)
        projected_point = point_on_line + distance_along_line * line_dir
        projected_points.append(projected_point)
    
    return np.array(projected_points)

def detect_matching_edge_on_model(model_pc, target_length, tolerance=0.5):
    points = np.asarray(model_pc.points)
    num_points = points.shape[0]

    for i in range(num_points):
        for j in range(i + 1, num_points):
            length = np.linalg.norm(points[i] - points[j])
            if abs(length - target_length) <= tolerance:
                return points[i], points[j]
    
    return None, None

def calculate_alignment_transform(scene_edge, model_edge):
    translation = scene_edge[0] - model_edge[0]

    translated_model_edge = model_edge + translation

    model_direction = translated_model_edge[1] - translated_model_edge[0]
    scene_direction = scene_edge[1] - scene_edge[0]

    model_direction /= np.linalg.norm(model_direction)
    scene_direction /= np.linalg.norm(scene_direction)

    rotation_axis = np.cross(model_direction, scene_direction)
    rotation_angle = np.arccos(np.clip(np.dot(model_direction, scene_direction), -1.0, 1.0))

    if np.linalg.norm(rotation_axis) < 1e-6:
        rotation_matrix = np.eye(3)
    else:
        rotation_axis /= np.linalg.norm(rotation_axis)
        
        K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                      [rotation_axis[2], 0, -rotation_axis[0]],
                      [-rotation_axis[1], rotation_axis[0], 0]])

        rotation_matrix = np.eye(3) + np.sin(rotation_angle) * K + (1 - np.cos(rotation_angle)) * np.dot(K, K)

    transformation_matrix = np.eye(4)

    transformation_matrix[:3, 3] = translation

    T1 = np.eye(4)
    T1[:3, 3] = -scene_edge[0]

    R = np.eye(4)
    R[:3, :3] = rotation_matrix

    T2 = np.eye(4)
    T2[:3, 3] = scene_edge[0]

    full_transform = T2 @ R @ T1

    transformation_matrix = full_transform @ transformation_matrix

    aligned_model_edge = (transformation_matrix[:3, :3] @ model_edge.T).T + transformation_matrix[:3, 3]
    alignment_error = np.linalg.norm(aligned_model_edge[1] - scene_edge[1])
    print(f"Alignment error after transformation: {alignment_error}")

    return transformation_matrix

def score_alignment(transformed_model_pc, scene_pc, max_score=10, radius=1.0):
    score = 0.0
    scene_kdtree = o3d.geometry.KDTreeFlann(scene_pc)

    for vertex in transformed_model_pc.points:
        [_, idx, distances] = scene_kdtree.search_knn_vector_3d(vertex, 1)
        if len(idx) > 0:
            nearest_distance = distances[0] ** 0.5
            vertex_score = min(max_score, 1.0 / nearest_distance)
            score += vertex_score
    
    return score

def optimize_alignment(transformed_model_pc, scene_pc, transformed_model_edge):
    best_score = -float('inf')
    best_transformed_pc = None
    best_transformation = None
    best_angle = None

    edge_vector = transformed_model_edge[1] - transformed_model_edge[0]
    edge_vector /= np.linalg.norm(edge_vector)

    rotation_origin = transformed_model_edge[0]

    for degree in range(0, 360):
        angle = np.deg2rad(degree)
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(edge_vector * angle)
        
        T1 = np.eye(4)
        T1[:3, 3] = -rotation_origin

        T2 = np.eye(4)
        T2[:3, 3] = rotation_origin

        rotation_transform = np.eye(4)
        rotation_transform[:3, :3] = R

        full_rotation_transform = T2 @ rotation_transform @ T1

        candidate_transformed_pc = copy.deepcopy(transformed_model_pc).transform(full_rotation_transform)

        score = score_alignment(candidate_transformed_pc, scene_pc)

        if score > best_score:
            best_score = score
            best_transformed_pc = copy.deepcopy(candidate_transformed_pc)
            best_transformation = full_rotation_transform
            best_angle = degree

    print(f"Best rotation at {best_angle}° with score = {best_score:.2f}")

    return best_transformed_pc, best_transformation

if __name__ == "__main__":
    mask_file = "brick_registration_test_set/mask0.png"
    bag_file = "brick_registration_test_set/depth_map0.bag"
    model_ply = "brick_registration_test_set/model.ply"

    depth_image, depth_scale = load_depth_from_bag(bag_file)

    camera_intrinsics = rs.intrinsics()
    camera_intrinsics.fx = 640
    camera_intrinsics.fy = 480
    camera_intrinsics.ppx = 320
    camera_intrinsics.ppy = 240

    scene_pc = generate_pointcloud_from_depth_mask(mask_file, depth_image, depth_scale, camera_intrinsics)
    scene_pc = refine_pointcloud(scene_pc)

    model_pc = generate_pointcloud_from_ply(model_ply)

    planes = fit_plane_to_pointcloud(scene_pc)

    new_extreme_points, most_certain_length = detect_edge_from_largest_planes(planes)

    if new_extreme_points is not None:
        print(f"Detected edge length: {most_certain_length:.2f} cm")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud Visualization", width=800, height=600, left=100, top=100)
    display_reference_system(vis)
    display_pointcloud(vis, scene_pc, 'FF0000')        
    display_pointcloud(vis, model_pc, '00FF00')        
    vis.poll_events()
    vis.update_renderer()

    if new_extreme_points is not None:
        extreme_edge_pc = o3d.geometry.PointCloud()
        extreme_edge_pc.points = o3d.utility.Vector3dVector(new_extreme_points)
        extreme_edge_pc.paint_uniform_color([1, 0, 1])  
        vis.add_geometry(extreme_edge_pc)

    if most_certain_length > 0:
        model_edge_start, model_edge_end = detect_matching_edge_on_model(model_pc, most_certain_length)

        if model_edge_start is not None and model_edge_end is not None:
            model_edge = np.array([model_edge_start, model_edge_end])
            print(f"Matching edge found on the model with length: {most_certain_length:.2f} cm")

            transformation_matrix = calculate_alignment_transform(new_extreme_points, model_edge)

            print_transformation_matrix(transformation_matrix)

            transformed_model_pc = apply_transform_to_pointcloud(model_pc, transformation_matrix)

            transformed_model_edge = np.array([
                transformation_matrix[:3, :3] @ model_edge[0] + transformation_matrix[:3, 3],
                transformation_matrix[:3, :3] @ model_edge[1] + transformation_matrix[:3, 3]
            ])

            best_transformed_pc, best_transformation = optimize_alignment(transformed_model_pc, scene_pc, transformed_model_edge)

            display_pointcloud(vis, best_transformed_pc, '0000FF')

            matching_edge_pc = o3d.geometry.PointCloud()
            matching_edge_pc.points = o3d.utility.Vector3dVector(model_edge)
            translation_matrix = np.eye(4)
            translation_matrix[2, 3] = 1.0
            matching_edge_pc.transform(translation_matrix)
            matching_edge_pc.paint_uniform_color([1, 1, 0])  
            vis.add_geometry(matching_edge_pc)
        else:
            print("No matching edge found in the model.")

    vis.poll_events()
    vis.update_renderer()
    vis.run()
