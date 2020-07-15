#include <iostream>
#include <stdexcept>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "../lib/lodepng/lodepng.h"
#include "utils.h"

using namespace std;

Camera::Camera(Vec pos, Vec tar, int width, int height) 
{
    cam_width = width;
    cam_height = height;
    cam_ratio = (double)cam_width/cam_height;

    cam_pos = pos;
    cam_dir = (tar - cam_pos).norm();
    cam_x_dir = Vec(0, 0, 1).cross(cam_dir * -1).norm();
    cam_y_dir = cam_x_dir.cross(cam_dir).norm();

    cam_x_spacing = (2.0 * cam_ratio)/(double)cam_width;
    cam_y_spacing = (double)2.0/(double)cam_height;
    cam_x_spacing_half = cam_x_spacing * 0.5;
    cam_y_spacing_half = cam_y_spacing * 0.5;
}

int Camera::get_width() { return cam_width; }
int Camera::get_height() { return cam_height; }

Ray Camera::get_ray(int x, int y, bool jitter, unsigned short *Xi) 
{

    double x_jitter;
    double y_jitter;

    // Jitter point for anti-aliasing
    if (jitter) 
    {
        x_jitter = (erand48(Xi) * cam_x_spacing) - cam_x_spacing_half;
        y_jitter = (erand48(Xi) * cam_y_spacing) - cam_y_spacing_half;

    }
    else 
    {
        x_jitter = 0;
        y_jitter = 0;
    }

    Vec pixel = cam_pos + cam_dir*2;
    pixel = pixel - cam_x_dir*cam_ratio + cam_x_dir*((x * 2 * cam_ratio)/cam_width) + x_jitter;
    pixel = pixel + cam_y_dir - cam_y_dir*((y * 2.0)/cam_height + y_jitter);

    return Ray(cam_pos, (pixel-cam_pos).norm());
}

// Build Kd-Tree
KdNode* KdNode::build(std::vector<Triangle*> &tris, int depth)
{
    KdNode* node = new KdNode();
    node->leaf = false;
    node->triangles = std::vector<Triangle*>();
    node->left = NULL;
    node->right = NULL;
    node->box = AABBOX();

    if (tris.size() == 0) return node;

    if (depth > 25 || tris.size() <= 6) 
    {
        node->triangles = tris;
        node->leaf = true;
        node->box = tris[0]->get_bounding_box();

        for (long i=1; i<tris.size(); i++) 
        {
            node->box.box_check(tris[i]->get_bounding_box());
        }

        node->left = new KdNode();
        node->right = new KdNode();
        node->left->triangles = std::vector<Triangle*>();
        node->right->triangles = std::vector<Triangle*>();

        return node;
    }

    node->box = tris[0]->get_bounding_box();
    Vec mid_point = Vec();

    for (int i=1; i<tris.size(); i++) 
    {
        node->box.box_check(tris[i]->get_bounding_box());
        mid_point = mid_point + (tris[i]->get_midpoint()/tris.size());
    }

    std::vector<Triangle*> left_tris;
    std::vector<Triangle*> right_tris;
    int axis = node->box.get_longest_axis();

    for (int i=0; i<tris.size(); i++) 
    {
        switch (axis) 
        {
            case 0:
                if(mid_point.x >= tris[i]->get_midpoint().x)
                    right_tris.push_back(tris[i]);
                else
                    left_tris.push_back(tris[i]);
                break;
            case 1:
                if(mid_point.y >= tris[i]->get_midpoint().y)
                    right_tris.push_back(tris[i]);
                else
                    left_tris.push_back(tris[i]);
                break;
            case 2:
                if(mid_point.z >= tris[i]->get_midpoint().z)
                    right_tris.push_back(tris[i]);
                else
                    left_tris.push_back(tris[i]);
                break;
        }
    }

    if (tris.size() == left_tris.size() || tris.size() == right_tris.size()) 
    {
        node->triangles = tris;
        node->leaf = true;
        node->box = tris[0]->get_bounding_box();

        for (int i=1; i<tris.size(); i++) 
        {
            node->box.box_check(tris[i]->get_bounding_box());
        }

        node->left = new KdNode();
        node->right = new KdNode();
        node->left->triangles = std::vector<Triangle*>();
        node->right->triangles = std::vector<Triangle*>();

        return node;
    }

    node->left = build(left_tris, depth+1);
    node->right = build(right_tris, depth+1);

    return node;
}

// If the ray intersect triangles in kd tree, find the nearest one
bool KdNode::hit(KdNode *node, const Ray &ray, double &t, double &tmin, Vec &normal, Vec &c) 
{
    double dis;
    if (node->box.intersection(ray, dis))
    {
        if (dis > tmin) return false;

        bool hit_tri = false;
        bool hit_left = false;
        bool hit_right = false;
        int tri_idx;

        if (!node->leaf) 
        {
            hit_left = hit(node->left, ray, t, tmin, normal, c);
            hit_right = hit(node->right, ray, t, tmin, normal, c);

            return hit_left || hit_right;
        }
        else 
        {
            int triangles_size = node->triangles.size();
            for (int i=0; i<triangles_size; i++) 
            {
                if (node->triangles[i]->tris_intersection(ray, t, tmin, normal))
                {
                    hit_tri = true;
                    tmin = t;
                    tri_idx = i;
                }
            }
            if (hit_tri) 
            {
                Vec p = ray.orig + ray.dir * tmin;
                c = node->triangles[tri_idx]->get_colour_at(p);
                return true;
            }
        }
    }
    return false;
}

Material::Material(MaterialType t, Vec c, Vec e, Texture tex) 
{
    m_type=t;
    m_colour=c;
    m_emission=e;
    m_texture = tex;
}

MaterialType Material::get_type() { return m_type; }
Vec Material::get_colour() { return m_colour; }

// Get colour at u,v coords
Vec Material::get_colour_at(double u, double v)
{
    return m_colour;
}

Vec Material::get_emission() { return m_emission; }

Ray Material::get_reflected_ray(const Ray &r, Vec &p, const Vec &n, unsigned short *Xi)
{
    // Specular
    if (m_type == SPEC) 
    {
        double rough = 0.8;
        // The reflection vector R is calculated with the following formula: R = 2 * (N · L) * N ­ L
        Vec refl = r.dir - n * 2 * n.dot(r.dir);
        refl = Vec
        (
            refl.x + (erand48(Xi)-0.5)*rough,
            refl.y + (erand48(Xi)-0.5)*rough,
            refl.z + (erand48(Xi)-0.5)*rough
        ).norm();

        return Ray(p, refl);
    }

    // Diffuse
    if (m_type == DIFF) 
    {
        Vec nl, u, v, w, d;
        double r1, r2, r2s;
        nl=n.dot(r.dir)<0?n:n*-1;
        r1=2*PI*erand48(Xi);
        r2=erand48(Xi);
        r2s=sqrt(r2);
        w=nl;
        u=((fabs(w.x)>.1?Vec(0,1):Vec(1))%w).norm();
        v=w%u;
        d = (u*cos(r1)*r2s + v*sin(r1)*r2s + w*sqrt(1-r2)).norm();
        return Ray(p, d);
    }
}

ObjIntersection::ObjIntersection(bool Hit, double U, Vec N, Material M)
{
    hit=Hit, u=U, n=N, m=M;
}

Sphere::Sphere( Vec P, double R, Material M ) 
{
    m_p=P, m_r=R, m_m=M;
}

double Sphere::get_radius() { return m_r; }
Material Sphere::get_material() { return m_m; }

// Check if ray intersects with sphere
ObjIntersection Sphere::get_intersection(const Ray &ray) 
{
    bool hit = false;
    double dis = 0;
    Vec n = Vec();
    Vec op = m_p-ray.orig;
    double t, eps, b, det;
    eps=1e-4;
    b=op.dot(ray.dir);
    det=b*b-op.dot(op)+m_r*m_r;
    if (det<0) return ObjIntersection(hit, dis, n, m_m); 
    else det=sqrt(det);
    dis = (t=b-det)>eps ? t : ((t=b+det)>eps ? t : 0);
    if (dis != 0) hit = true;
    n = ((ray.orig + ray.dir * dis) - m_p).norm();

    return ObjIntersection(hit, dis, n, m_m);
}


Mesh::Mesh(Vec p, const char* path, Material m) 
{
    m_p=p, m_m=m;
    std::string mtlbasepath;
    std::string inputfile = path;
    unsigned long pos = inputfile.find_last_of("/");
    mtlbasepath = inputfile.substr(0, pos+1);
    // Load mesh
    std::string err = tinyobj::LoadObj(m_shapes, m_materials, inputfile.c_str(), mtlbasepath.c_str());
    int shapes_size, idx_size, materials_size;
    shapes_size = m_shapes.size();
    materials_size = m_materials.size();

    // Load materials
    for (int i=0; i<materials_size; i++) 
    {
        std::string texture_path = "";

        if (!m_materials[i].diffuse_texname.empty())
        {
            if (m_materials[i].diffuse_texname[0] == '/') texture_path = m_materials[i].diffuse_texname;
            texture_path = mtlbasepath + m_materials[i].diffuse_texname;
            materials.push_back( Material(DIFF, Vec(1,1,1), Vec()) );
        }
        else 
        {
            materials.push_back( Material(DIFF, Vec(1,1,1), Vec()) );
        }

    }

    // Load triangles
    for (int i = 0; i < shapes_size; i++) 
    {
        idx_size = m_shapes[i].mesh.indices.size() / 3;
        for (size_t f = 0; f < idx_size; f++) 
        {
            Vec V0 = Vec
            (
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f] * 3     ],
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f] * 3 + 1 ],
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f] * 3 + 2 ]
            ) + m_p;

            Vec V1 = Vec
            (
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f + 1] * 3     ],
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f + 1] * 3 + 1 ],
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f + 1] * 3 + 2 ]
            ) + m_p;

            Vec V2 = Vec
            (
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f + 2] * 3     ],
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f + 2] * 3 + 1 ],
                m_shapes[i].mesh.positions[ m_shapes[i].mesh.indices[3*f + 2] * 3 + 2 ]
            ) + m_p;

            Vec T0, T1, T2;
            if (m_shapes[i].mesh.indices[3 * f + 2] * 2 + 1 < m_shapes[i].mesh.texcoords.size()) 
            {
                T0 = Vec
                (
                    m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f] * 2],
                    m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f] * 2 + 1],
                    0
                );

                T1 = Vec
                (
                    m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 1] * 2],
                    m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 1] * 2 + 1],
                    0
                );

                T2 = Vec
                (
                    m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 2] * 2],
                    m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 2] * 2 + 1],
                    0
                );
            }
            else 
            {
                T0=Vec();
                T1=Vec();
                T2=Vec();
            }

            if (m_shapes[i].mesh.material_ids[ f ] < materials.size())
                tris.push_back(new Triangle(V0, V1, V2, T0, T1, T2, &materials[m_shapes[i].mesh.material_ids[f]]));
            else
                tris.push_back(new Triangle(V0, V1, V2, T0, T1, T2, &m_m));
        }
    }

    m_shapes.clear();
    m_materials.clear();
    node = KdNode().build(tris, 0);
}

// Check if ray intersects with mesh
ObjIntersection Mesh::get_intersection(const Ray &ray) 
{
    double t=0, tmin=INFINITY;
    Vec normal = Vec();
    Vec colour = Vec();
    bool hit = node->hit(node, ray, t, tmin, normal, colour);
    return ObjIntersection(hit, tmin, normal, Material(DIFF, colour, Vec()));
}

inline double clam_p(double x){ return x<0 ? 0 : x>1 ? 1 : x; }
inline int toInt(double x){ return int(clam_p(x)*255+.5); }

Renderer::Renderer(Scene *scene, Camera *camera) 
{
    m_scene = scene;
    m_camera = camera;
    m_pixel_buffer = new Vec[m_camera->get_width()*m_camera->get_height()];
}

void Renderer::render(int samples) 
{
    int width = m_camera->get_width();
    int height = m_camera->get_height();

    for (int y=0; y<height; y++)
    {
        unsigned short Xi[3]={0,0,y*y*y};              
        for (int x=0; x<width; x++)
        {
            Vec col = Vec();
            for (int a=0; a<samples; a++)
            {
                Ray ray = m_camera->get_ray(x, y, a>0, Xi);
                col = col + m_scene->trace_ray(ray,0,Xi);
            }
            m_pixel_buffer[(y)*width + x] = col/samples;
        }
    }
}

void Renderer::save_image(string path) 
{
    int width = m_camera->get_width();
    int height = m_camera->get_height();

    std::vector<unsigned char> pixel_buffer;

    int pixel_count = width*height;

    for (int i=0; i<pixel_count; i++) 
    {
        pixel_buffer.push_back(toInt(m_pixel_buffer[i].x));
        pixel_buffer.push_back(toInt(m_pixel_buffer[i].y));
        pixel_buffer.push_back(toInt(m_pixel_buffer[i].z));
        pixel_buffer.push_back(255);
    }

    unsigned error = lodepng::encode(path, pixel_buffer, width, height);
    if(error) {printf("error!!\n"); return;}

    pixel_buffer.clear();
}

void Scene::add(Object *object) 
{
    m_objects.push_back( object );
}

ObjIntersection Scene::tris_intersection(const Ray &ray) 
{
    ObjIntersection inter = ObjIntersection();
    ObjIntersection tempt_p;
    long size = m_objects.size();

    for (int i=0; i<size; i++)
    {
        tempt_p = m_objects.at((unsigned)i)->get_intersection(ray);

        if (tempt_p.hit) 
        {
            if (inter.u == 0 || tempt_p.u < inter.u) inter = tempt_p;
        }
    }
    return inter;
}

Vec Scene::trace_ray(const Ray &ray, int depth, unsigned short*Xi) 
{
    ObjIntersection inter = tris_intersection(ray);

    if (!inter.hit) return Vec();

    if (inter.m.get_type() == EMIT) return inter.m.get_emission();

    Vec colour = inter.m.get_colour();

    double p = colour.x>colour.y && colour.x>colour.z ? colour.x : colour.y>colour.z ? colour.y : colour.z;
    double rnd = erand48(Xi);

    if (++depth>5)
    {
        if (rnd<p*0.9) 
        { 
            colour=colour*(0.9/p);
        }
        else 
        {
            return inter.m.get_emission();
        }
    }

    Vec x = ray.orig + ray.dir * inter.u;
    Ray reflected = inter.m.get_reflected_ray(ray, x, inter.n, Xi);

    return colour.mult( trace_ray(reflected, depth, Xi) );
}

Texture::Texture(const char *filename) 
{
    image = std::vector<unsigned char>();
    unsigned error = lodepng::decode(image, width, height, filename);
    if(error) { printf("error!!!"); return; }
    loaded = true;
}

Vec Texture::get_pixel(unsigned x, unsigned y)
{
    if (!loaded)
        return (Vec(1,0,1));

    double r, g, b;
    r = (double)image.at(y*width*4 + x    )/255.;
    g = (double)image.at(y*width*4 + x + 1)/255.;
    b = (double)image.at(y*width*4 + x + 2)/255.;
    return Vec(r, g, b);
}

bool Texture::is_loaded() 
{
    return loaded;
}


/************************** Main ********************************/

int main(int argc, char *argv[]) 
{

    time_t start, stop;
    time(&start);              
    string out_name;
    int sam_ples = 3;

    if (argc < 3) 
    {
        printf("Please enter: sam_ple number + output file name (i.e./pathtracer 3 render.jpg)\n ");
        return 0;
    }
    if (argc == 3) sam_ples = atoi(argv[1]);
    if (argc == 3) out_name = argv[2];

    // For Aram test
    Camera camera = Camera(Vec(0, -10, 2.5), Vec(0,0,1), 720, 450);     
    Scene scene = Scene();                                             

    scene.add( dynamic_cast<Object*>(new Sphere(Vec(0,0,-1000), 1000, Material())) );
    scene.add( dynamic_cast<Object*>(new Sphere(Vec(-1004,0,0), 1000, Material(DIFF, Vec(1.0, 0.3,0.3)))) );
    scene.add( dynamic_cast<Object*>(new Sphere(Vec(1004,0,0), 1000, Material(DIFF, Vec(0.2, 0.6, 0.2)))) );
    scene.add( dynamic_cast<Object*>(new Sphere(Vec(0,1006,0), 1000, Material())) );
    scene.add( dynamic_cast<Object*>(new Sphere(Vec(0,0,110), 100, Material(EMIT, Vec(1,1,1), Vec(2.2,2.2,2.2)))) );
    scene.add( dynamic_cast<Object*>(new Mesh(Vec(), "../obj/Aram.obj", Material(DIFF, Vec(0.9, 0.9, 0.9)))) );

    Renderer renderer = Renderer(&scene, &camera); 

    printf("===> Sample number: %i\n", sam_ples); 
    printf("===> Start rendering...\n");

    renderer.render(sam_ples);                      
    renderer.save_image(out_name);             
   
    printf("===> Completed!!!\n");
    time(&stop);
    double diff = difftime(stop, start);
    int dif_time = int(diff);
    printf("===> Total time: %i seconds\n", dif_time);

    return 0;
}