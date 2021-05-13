import numpy as np
import open3d as o3d

if __name__ == "__main__":

    #mesh1 = o3d.io.read_triangle_mesh("panda_epi_data_ver.1(xzy-coord, no hierarchy)/epi0000_frame0.obj")
    mesh1 = o3d.io.read_triangle_mesh("panda_epi_data/epi0000/epi0000_frame0.obj")
    vertices1 = np.asarray(mesh1.vertices)
    triangles1 = np.asarray(mesh1.triangles)

    # ========================================================================
    # visualization
    # ========================================================================
    mesh1.compute_vertex_normals()

    #coord_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    #o3d.visualization.draw_geometries([mesh, coord_mesh])
    o3d.visualization.draw_geometries([mesh1])
    # break point
    # __import__('pdb').set_trace()