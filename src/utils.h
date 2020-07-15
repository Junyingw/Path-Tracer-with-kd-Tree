#include <cfloat>
#include <algorithm>
#include <stdint.h>
#include "../lib/rand48/erand48.h"
#include "../lib/tiny_obj_loader/tiny_obj_loader.h"

using namespace std;
#define PI 3.1415926

// Vector3d 
struct Vec 
{
    double x, y, z;

    Vec(double X=0, double Y=0, double Z=0) : x(X), y(Y), z(Z) {}

	//Vector3d operator overloading 
    Vec operator+(const Vec &b) const { return Vec(x + b.x, y + b.y, z + b.z); }       
    Vec operator-(const Vec &b) const { return Vec(x - b.x, y - b.y, z - b.z); }        
    Vec operator*(double b) const { return Vec(x*b, y*b, z*b); }                  
    Vec operator/(float b) const { return Vec(x/b, y/b, z/b); }                   
    Vec mult(const Vec &b) const { return Vec(x*b.x, y*b.y, z*b.z); }                           
    Vec operator%(Vec &b){return Vec(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x);}      
    Vec cross(const Vec &b){return Vec(y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x);}    
    double dot(const Vec &b) const { return x*b.x + y*b.y + z*b.z; }   
    Vec& norm(){ return *this = *this * (1/sqrt(x*x + y*y + z*z)); }    
};

// Ray = orig + t * dir
struct Ray 
{
    Vec orig, dir, dir_inv;
    Ray(Vec o, Vec d) : orig(o), dir(d) 
    {
        dir_inv = Vec(1./dir.x, 1./dir.y, 1./dir.z);
    }
};

// Axis-aligned bounding box
struct AABBOX 
{
    Vec box_min;     
    Vec box_max ;   

    AABBOX (Vec minleft=Vec(), Vec topright=Vec())
    {
        box_min=minleft, box_max =topright;
    }

    // Check box axis to get the right bbox
    void box_check(const AABBOX &box) 
    {
        if (box.box_min.x < box_min.x) box_min.x = box.box_min.x;
        if (box.box_min.y < box_min.y) box_min.y = box.box_min.y;
        if (box.box_min.z < box_min.z) box_min.z = box.box_min.z;

        if (box.box_max.x > box_max .x) box_max .x = box.box_max.x;
        if (box.box_max.y > box_max .y) box_max .y = box.box_max.y;
        if (box.box_max.z > box_max .z) box_max .z = box.box_max.z ;
    }
    
    // Get longest axis
    int get_longest_axis() 
    {   
        // For axis -> x: 0, y: 1, z: 2
        Vec diff = box_max - box_min;
        if (diff.x > diff.y && diff.x > diff.z) 
            return 0;
        else if (diff.y > diff.x && diff.y > diff.z) 
            return 1;
        else
            return 2;
    }

    // Check if the ray intersects with the box
    bool intersection(const Ray &r, double &t) 
    {
        // If the ray is parallel to an axis, it wont't interest with the bbox
        double tx1 = (box_min.x - r.orig.x)/r.dir.x;
        double tx2 = (box_max.x - r.orig.x)/r.dir.x;        
        double ty1 = (box_min.y - r.orig.y)/r.dir.y;
        double ty2 = (box_max.y - r.orig.y)/r.dir.y;       
        double tz1 = (box_min.z - r.orig.z)/r.dir.z;
        double tz2 = (box_max.z - r.orig.z)/r.dir.z;

        double tmin = std::min(tx1, tx2);
        double tmax = std::max(tx1, tx2);
        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));
        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));

        t = tmin;
        
        if (tmax >= tmin)
            return true;
        else
            return false;
    }
};

// Define Camera
class Camera 
{
    private:
        int cam_width;
        int cam_height;
        double cam_ratio;
        double cam_x_spacing;        
        double cam_y_spacing;
        double cam_x_spacing_half;
        double cam_y_spacing_half;

        Vec cam_pos;
        Vec cam_dir;
        Vec cam_x_dir;
        Vec cam_y_dir;

    public:
        Camera(Vec pos, Vec tar, int width, int height);
        int get_width();
        int get_height();
        Ray get_ray(int x, int y, bool jitter, unsigned short *Xi);
};

class Texture 
{
    private:
        unsigned width;
        unsigned height;
        bool loaded = false;
        std::vector<unsigned char> image;
    public:
        Texture(const char* file);
        Texture(){};
        Vec get_pixel(unsigned x, unsigned y);
        bool is_loaded();
};

enum MaterialType { DIFF, SPEC, EMIT };
class Material 
{
    private:
        MaterialType m_type;
        Vec m_colour;
        Vec m_emission;
        Texture m_texture;

    public:
        Material( MaterialType m=DIFF, Vec c=Vec(1,1,1), Vec r=Vec(0,0,0), Texture tex=Texture() );
        MaterialType get_type();
        Vec get_colour();
        Vec get_colour_at(double u, double v);
        Vec get_emission();
        Ray get_reflected_ray( const Ray &r, Vec &p, const Vec &n, unsigned short *Xi );
};

struct Triangle 
{
    Vec v0, v1, v2;     
    Vec e1, e2;         
    Vec n;
    Vec t0, t1, t2; 
    Material *m;       

    Triangle(Vec V0, Vec V1, Vec V2, Vec T0=Vec(), Vec T1=Vec(), Vec T2=Vec(), Material *M=NULL)
    {
        v0=V0, v1=V1, v2=V2;
        e1=v1-v0, e2=v2-v0;
        t0=T0, t1=T1, t2=T2;
        n=e1.cross(e2).norm();
        m=M;
    }    
    
    // Midpoint of the triangle
    Vec get_midpoint()
    {
        Vec mid_point;
        mid_point = (v0 + v1 + v2)/3;
        return mid_point;
    }

    // Get the AABBOX that contains the triangle
    AABBOX get_bounding_box()
    {
        Vec box_min = Vec
        (
            std::min (std::min(v0.x, v1.x), v2.x),
            std::min (std::min(v0.y, v1.y), v2.y),
            std::min (std::min(v0.z, v1.z), v2.z)
        );
        Vec box_max  = Vec
        (
            std::max (std::max(v0.x, v1.x), v2.x),
            std::max (std::max(v0.y, v1.y), v2.y),
            std::max (std::max(v0.z, v1.z), v2.z)
        );

        return AABBOX(box_min, box_max);
    }

    // Checks if the ray intersects with the triangle
    bool tris_intersection(Ray ray, double &t, double tmin, Vec &norm)
    {
        double u, v, t_temp=0;
        Vec pc = ray.dir.cross(e2);
        double det = e1.dot(pc);
        if (det == 0) 
            return false;
        Vec tc = ray.orig - v0;
        u = tc.dot(pc)/det;
        if (u < 0 || u > 1) 
            return false;
        Vec qc = tc.cross(e1);
        v = ray.dir.dot(qc)/det;
        if (v < 0 || u + v > 1) 
            return false;
        t_temp = e2.dot(qc)/det; 
        if (t_temp < tmin) 
        {
            if (t_temp > 1e-9)
            {   
                t = t_temp;        
                norm = n;
                return true;
            }
        }
        return false;
    }

    // Get the barycentric poit of the triangle
    Vec barycentric(Vec p)
    {
        Vec pv0 = p - v0;
        double t00 = e1.dot(e1);
        double t01 = e1.dot(e2);
        double t11 = e2.dot(e2);
        double t20 = pv0.dot(e1);
        double t21 = pv0.dot(e2);
        double t = t00*t11 - t01*t01;
        double v = (t11*t20 - t01*t21) / t;
        double w = (t00*t21 - t01*t20) / t;
        double u = 1 - v - w;
        Vec bary = Vec(u, v, w);
        return bary;
    }

    // Get the colour of point p
    Vec get_colour_at(Vec p)
    {
        if(m==NULL) return Vec(1,1,1);

        Vec b = barycentric(p);
        Vec c = Vec();
        c = c + (t0 * b.x);
        c = c + (t1 * b.y);
        c = c + (t2 * b.z);

        Vec color = m->get_colour_at(c.x, c.y);

        return color;
    }
};

class KdNode 
{
    public:
        AABBOX box;
        KdNode* left;
        KdNode* right;
        std::vector<Triangle*> triangles;
        bool leaf;

        KdNode(){};
        KdNode* build(std::vector<Triangle*> &tris, int depth);
        bool hit (KdNode* node, const Ray &ray, double &t, double &tmin, Vec &normal, Vec &c);
};

struct ObjIntersection 
{
	bool hit;	
	double u;	
	Vec n;		
	Material m;

	ObjIntersection(bool hit_=false, double u_=0, Vec n_=Vec(), Material m_=Material());
};


class Object 
{
    public:
        Vec m_p;
        virtual ObjIntersection get_intersection(const Ray &r) = 0;
};


class Sphere : public Object 
{
    private:
        double m_r;	
        Material m_m;

    public:
        Sphere(Vec p, double r, Material m);	
        virtual double get_radius();
        virtual Material get_material();

        virtual ObjIntersection get_intersection(const Ray &r);
};


class Mesh : public Object 
{
    private:
        std::vector<tinyobj::shape_t> m_shapes;
        std::vector<tinyobj::material_t> m_materials;
        std::vector<Material> materials;
        std::vector<Triangle*> tris;
        Material m_m;
        KdNode *node;

    public:
        Mesh(Vec p_, const char* file_path, Material m_);
        virtual ObjIntersection get_intersection(const Ray &r);

};

class Scene 
{
    private:
        std::vector<Object*> m_objects;

    public:
        Scene(){};
        void add(Object *object);
        ObjIntersection tris_intersection(const Ray &ray);
        Vec trace_ray(const Ray &ray, int depth, unsigned short*Xi);
};

class Renderer 
{
    private:
        Scene *m_scene;
        Camera *m_camera;
        Vec *m_pixel_buffer;

    public:
        Renderer(Scene *scene, Camera *camera);
        void render(int samples=3);
        void save_image(string file_path);
};
