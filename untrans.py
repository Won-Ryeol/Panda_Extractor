import numpy as np
import open3d as o3d

init_matrixes_str = ["1 0 0 -0.0409997 0 1 0 -0.0750001 0 0 1 0.0700002 0 0 0 1",
"1 0 0 0.000294633 0 1 0 -0.112231 0 0 1 0.263855 0 0 0 1",
"1 -8.67219e-09 -2.0647e-09 0.000267856 8.67219e-09 1 -3.91858e-16 -0.0378044 2.0647e-09 3.73952e-16 1 0.402076 0 0 0 1",
"1 -1.35332e-07 1.6606e-09 0.0414289 1.35332e-07 1 1.49011e-08 -0.0468997 -1.6606e-09 -1.49011e-08 1 0.614657 0 0 0 1",
"-1.19209e-07 -7.27863e-07 1 0.1163 -2.58972e-07 1 7.27863e-07 -0.102473 -1 -2.58972e-07 -1.19209e-07 0.690551 0 0 0 1",
"-1.19209e-07 -6.60808e-07 1 0.361643 -2.9995e-07 1 6.60808e-07 -0.0375155 -1 -2.9995e-07 -1.19209e-07 0.731525 0 0 0 1",
"1 3.20213e-08 3.05153e-07 0.508857 -3.20214e-08 1 1.78814e-07 -0.0803597 -3.05153e-07 -1.78814e-07 1 0.744918 0 0 0 1",
"1 -5.2315e-09 9.65371e-08 0.554692 5.23152e-09 1 -1.83794e-07 -0.0741114 -9.65371e-08 1.83794e-07 1 0.650992 0 0 0 1"]
init_matrixes = [mat.split(" ") for mat in init_matrixes_str]
init_matrixes = [[(float)(item) for item in mat] for mat in init_matrixes]

def get_init_rot_mat(link_num):
    return np.array([init_matrixes[link_num][:3],init_matrixes[link_num][4:7],init_matrixes[link_num][8:11]])

def get_init_trans(link_num):
    return np.array([init_matrixes[link_num][3],init_matrixes[link_num][7],init_matrixes[link_num][11]])

if __name__ == "__main__":

    max_link = 8
    meshes = []
    vertices = []
    triangles = []

    for i in range(max_link):
        mesh = o3d.io.read_triangle_mesh(f"panda_obj/panda{i+1}.obj")
        meshes.append(mesh)
        vertices.append(np.asarray(mesh.vertices))
        triangles.append(np.asarray(mesh.triangles))

    # ========================================================================
    # translation
    # ========================================================================
    for i in range(max_link):
        if i >= 3:
            #vertices[i] = np.matmul(get_init_rot_mat(i).T, vertices[i].T).T
            vertices[i] = vertices[i] - get_init_trans(i)
        meshes[i].vertices = o3d.utility.Vector3dVector(vertices[i])

    # ========================================================================
    # rotation
    # ========================================================================


    # ========================================================================
    # save
    # ========================================================================
    for i in range(max_link):
        o3d.io.write_triangle_mesh(f'panda_obj/panda{i+1}.obj', meshes[i])

    # ========================================================================
    # visualization
    # ========================================================================
    for i in range(max_link):
        meshes[i].compute_vertex_normals()

    coord_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    #o3d.visualization.draw_geometries([mesh, coord_mesh])

    o3d.visualization.draw_geometries(meshes+[coord_mesh])
    # break point
    # __import__('pdb').set_trace()
