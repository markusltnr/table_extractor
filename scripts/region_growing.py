#!/usr/bin/python
import numpy as np
import open3d as o3d


class RegionGrowing:
    def __init__(self, scene):
        scene.remove_none_finite_points()
        scene.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        self.scene = scene
        self.scene_tree = o3d.geometry.KDTreeFlann(scene)
        self.curvature = self.compute_curvature(self.scene, self.scene_tree)
        print('RegionGrowing initzialization done.')
    
    def set_plane(self, plane):
        plane.remove_none_finite_points()
        self.plane = plane

    def compute_curvature(self, pcd, pcd_tree, radius=0.1):
        # from https://github.com/intel-isl/Open3D/issues/2471#issuecomment-722582817
        pcd.remove_none_finite_points()
        points = np.asarray(pcd.points)
        tree = pcd_tree

        curvature = [ 0 ] * points.shape[0]

        for index, point in enumerate(points):
            [k, indices, distances] = pcd_tree.search_hybrid_vector_3d(point, radius, 30)
            if k<=1:
                curvature[index] = np.inf
                continue
            # local covariance
            M = np.array([ points[i] for i in indices ]).T
            M = np.cov(M)
            # eigen decomposition
            V, E = np.linalg.eig(M)
            # h3 < h2 < h1
            h1, h2, h3 = V

            curvature[index] = h3 / (h1 + h2 + h3)

        return curvature

    def grow_region(self):
        scene = self.scene
        plane = self.plane

        #scene = o3d.io.read_point_cloud('/home/v4r/Markus_L/src/table_extractor/clouds/scene_cloud_0.pcd')
        #plane = o3d.io.read_point_cloud('/home/v4r/Markus_L/src/table_extractor/clouds/lowest_cupboard_plane.pcd')
        plane_box = plane.get_axis_aligned_bounding_box()
        plane_box.scale(10)
        scene = scene.crop(plane_box)
        #plane.paint_uniform_color([1,0,0])
        #plane.remove_none_finite_points()
        #scene.remove_none_finite_points()
        #scene.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        #o3d.visualization.draw_geometries([scene, plane, plane_box])

        scene_tree = o3d.geometry.KDTreeFlann(scene)
        processed_scene = np.zeros(np.asarray(scene.points).shape[0], dtype=bool)

        eps_angle_threshold_rad = 0.2
        max_neighbour_distance = 0.03
        curvature_threshold = 0.1


        scene_curvature = self.curvature #self.compute_curvature(scene, scene_tree)

        plane_size = np.asarray(plane.points).shape[0]
        orig_object_ind = []

        for i in range(plane_size):
            seed_queue = []
            sq_idx = 0

            [k, nn_indices, nn_sqrt_distances] = scene_tree.search_knn_vector_3d(plane.points[i], 2)
            orig_object_ind.append(nn_indices[0])

            
            if (nn_sqrt_distances[1] > max_neighbour_distance*max_neighbour_distance or processed_scene[nn_indices[1]]):
                continue

            seed_queue.append(nn_indices[1])
            processed_scene[nn_indices[1]] = True
            orig_object_ind.append(nn_indices[1])

            while (sq_idx < len(seed_queue)):
                sidx = seed_queue[sq_idx]
                query_pt = scene.points[sidx]
                query_n = scene.normals[sidx]

                [k, nn_indices, nn_sqrt_distances] = scene_tree.search_radius_vector_3d(query_pt, np.sqrt(2) * max_neighbour_distance)
                if k <=1:
                    sq_idx = sq_idx + 1
                    continue

                for j in range(k-1):
                    if processed_scene[nn_indices[j+1]]:
                        continue
                    if scene_curvature[nn_indices[j+1]] > curvature_threshold:
                        continue

                    nn = scene.normals[nn_indices[j+1]]
                    dot_p = np.dot(query_n, nn)
                    if(abs(dot_p) > np.cos(eps_angle_threshold_rad)):
                        processed_scene[nn_indices[j+1]] = True
                        seed_queue.append(nn_indices[j+1])
                        orig_object_ind.append(nn_indices[j+1])

                sq_idx = sq_idx + 1
                
        plane_expanded = scene.select_down_sample(orig_object_ind)
        #plane_expanded.paint_uniform_color([1,0,0])
        #o3d.visualization.draw_geometries([scene, plane_expanded])
        return plane_expanded
