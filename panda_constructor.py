import numpy as np
import open3d as o3d
import math
import pandas as pd
import os

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

trans = [[mat[3],mat[7],mat[11]] for mat in init_matrixes]
rel_trans = np.array(trans)
for i in range(len(rel_trans)-1,-1,-1):
    rel_trans[i] -= rel_trans[i-1]

joint_pos_str = ["+2.7148e-04 -7.4989e-02 +3.5763e-07",
                 "+2.7147e-04 -7.4989e-02 +3.3300e-01",
                 "+2.7127e-04 -7.4989e-02 +3.3300e-01",
                 "+8.2771e-02 -7.4989e-02 +6.4900e-01",
                 "+4.6677e-01 -7.4989e-02 +7.3150e-01",
                 "+4.6677e-01 -7.4989e-02 +7.3150e-01",
                 "+5.5477e-01 -7.4989e-02 +7.3150e-01"]
joint_ori = "z y z y x y z"

joint_pos = [pos.split(" ") for pos in joint_pos_str]
joint_pos = [[(float)(item) for item in mat] for mat in joint_pos]
joint_ori = joint_ori.split(" ")

pi = math.pi

def rot_by_axis(vertices, angle, link_num, joint_num):
    new_vertices = np.array(vertices)
    new_vertices -= np.array(joint_pos[joint_num]) - np.array(trans[joint_num+1])

    if joint_ori[joint_num] == "x":
        new_vertices = np.matmul(np.array([[1,0,0],[0,math.cos(angle),-math.sin(angle)],[0,math.sin(angle),math.cos(angle)]]) , new_vertices.T).T
    elif joint_ori[joint_num] == "y":
        new_vertices = np.matmul(np.array([[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]]) , new_vertices.T).T
    elif joint_ori[joint_num] == "z":
        new_vertices = np.matmul(np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]]) , new_vertices.T).T

    new_vertices += np.array(joint_pos[joint_num]) - np.array(trans[joint_num+1])
    return new_vertices

if __name__ == "__main__":

    for epi in range(2000):
        epi_df = pd.read_csv('~/PyRep/examples/joint_data/joint_data_epi{:0>4}.csv'.format(epi), index_col = False).to_numpy()
        epi_df = np.delete(epi_df, 0, axis = 1)
        os.mkdir('./panda_epi_data/epi{:0>4}'.format(epi))
        for idx, frame in enumerate(epi_df):
            max_link = 8
            meshes = []
            vertices = []
            triangles = []

            joint_angles = frame
            joint_angles[3] = -pi/2 - joint_angles[3]
            joint_angles[5] = pi/2 - joint_angles[5]
            
            for i in range(max_link):
                mesh = o3d.io.read_triangle_mesh(f"panda_obj/panda{i+1}.obj")
                meshes.append(mesh)
                vertices.append(np.asarray(mesh.vertices))
                triangles.append(np.asarray(mesh.triangles))

            # ========================================================================
            # rotation / translation
            # ========================================================================
            for j in range(max_link-2, -1, -1):
                for l in range(j+1, max_link):
                    vertices[l] = rot_by_axis(vertices[l], joint_angles[j], l,j)
                    vertices[l] += rel_trans[j+1]

            # ========================================================================
            # merge into single mesh
            # ========================================================================
            for i in range(max_link):
                vertices[i][:,[1,2]] = vertices[i][:,[2,1]]
                meshes[i].vertices = o3d.utility.Vector3dVector(vertices[i])

            mesh = meshes[0]
            for i in range(1, max_link):
                mesh = mesh + meshes[i]

            # ========================================================================
            # save
            # ========================================================================
            o3d.io.write_triangle_mesh('./panda_epi_data/epi{:0>4}/epi{:0>4}_frame{}.obj'.format(epi, epi, idx), mesh)
            print('epi{:0>4}_frame{}.obj Saved!'.format(epi, idx))

            # ========================================================================
            # visualization
            # ========================================================================
            # mesh.compute_vertex_normals()
            # coord_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
            # o3d.visualization.draw_geometries([mesh, coord_mesh])

            # ========================================================================
            # break point
            # ========================================================================
            # __import__('pdb').set_trace()