# Path-Tracer-with-kd-Tree
Simple path tracer, used k-d tree to do fast path tracing. 
# Functions
- Load .obj model 
- Mesh rendering based on monte carlo method 
- Change material type: diffuse, specular, emission
- k-d tree for meshes
# Compile
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```
# Demo
![alt-text](https://github.com/Junyingw/renderppl-dataset/blob/master/examples/aaron_image.png)
# Code
Testing for intersection between a ray and a triangle is easy, but if the geometry has too many triangles, that would be time consuming. In this project, we use axis-aligned bounding box to build k-d tree, trying to optimize this problem. We can benifit k-d tree by only testing if the ray hits the bounding box. 
1) Using Obj-reader to read and detect triangle-based geometries. 
2) Building axis-align bounding box to generate k-d tree, and level down the tree.
3) For each level of the tree, find the midpoint of all triangles in the node; finding the longest axis of the bounding box for that node。
4) Checking each triangle in the node, and then push them to the certain child node. 
