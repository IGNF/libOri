#include "../src/Ori.hpp"
#include "../src/IntrinsicModel.hpp"
#include "PlyHeaderParser.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "accelerators/kdtreeaccel.h"
#include "core/geometry.h"
#include "core/primitive.h"
#include "core/intersection.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

#define epsilon 1.e-3 // avoid numerical precision issues

// default logging macro (no ; at end to force user using common function syntax)
#define ERR(txt) {cout << "[" << __FUNCTION__ << "] ERROR: " << txt << endl; return false;}
#define LOG(txt) {cout << txt;}

struct ColoredPoint : public Point
{
    unsigned char r,g,b;
    ColoredPoint(Point P, unsigned char r_=0, unsigned char g_=0, unsigned char b_=0):
        Point(P), r(r_),g(g_),b(b_){}
    ColoredPoint(float x=0.f, float y=0.f, float z=0.f, unsigned char r_=0, unsigned char g_=0, unsigned char b_=0):
        Point(x,y,z), r(r_),g(g_),b(b_){}
};

// Ray/triangle3d intersaction. Warning: Unormalized normal
bool DoIntersect(const Ray &r, Point A, Point B, Point C,
                 Point & I, Vector & n, double & beta, double & gamma)
{
    /// (surfacic) Triangle3/Line3 intersection
    Vector AB = B - A;
    Vector AC = C - A;
    n = Cross(AB,AC);
    double vn = Dot(r.d,n);
    if(vn == 0.f)
    {
        //cout << "ray is parallel to triangle" << endl;
        return false;
    }
    double xI = Dot(A - r.o,n)/vn;
    I = r.o + xI * r.d;
    Vector AI = I - A;
    double inv_norm = 1.f/n.Length();
    n = inv_norm*n;
    Vector ABorth = Cross(n,AB);
    Vector ACorth = Cross(n,AC);
    // barycentric coords
    gamma = inv_norm*Dot(AI,ABorth);
    beta = -inv_norm*Dot(AI,ACorth);
    return (beta>-epsilon && gamma>=-epsilon && (beta+gamma)<1.f+epsilon);
    //    Vector nA = Cross(n,AB);
    //    Vector nB = Cross(n,BC);
    //    Vector nC = Cross(n,CA);
    //    // if ray intersection with plane is inside the triangle
    //    return (Dot(I - A,nA)>0.f && Dot(I - B,nB)>0.f && Dot(I - C,nC)>0.f);
}

class  KdTreeTriangle : public Primitive
{
public:
    // indices of the 3 vertices
    unsigned int a,b,c;
    // vector of points supporting the triangles
    vector<ColoredPoint> * pv_pt;

    KdTreeTriangle(unsigned int a_, unsigned int b_, unsigned int c_, vector<ColoredPoint> * pv_pt_):
        a(a_),b(b_),c(c_), pv_pt(pv_pt_) {}
    inline ColoredPoint A() const {return pv_pt->at(a);}
    inline ColoredPoint B() const {return pv_pt->at(b);}
    inline ColoredPoint C() const {return pv_pt->at(c);}

    ~ KdTreeTriangle(){;}

    BBox WorldBound() const
    {
        return Union(BBox(A(), B()), C());
    }

    bool Intersect(const Ray &r, Intersection *p_isect) const
    {
        if(p_isect->primitive != NULL)
        {
            //cout << '.';
            return false; // already intersected, we want first intersection only
        }
        Vector n;
        Point I;
        double  beta, gamma;
        if(!DoIntersect(r, A(), B(), C(), I, n, beta, gamma)) return false;
        double alpha = (1-beta-gamma);
        // stupid trick to avoid derivating Intersection, store rgb in dudx, dvdx and dudy that we are not using
        p_isect->dg.dudx = alpha*A().r + beta*B().r + gamma*C().r;
        p_isect->dg.dvdx = alpha*A().g + beta*B().g + gamma*C().g;
        p_isect->dg.dudy = alpha*A().b + beta*B().b + gamma*C().b;

        p_isect->dg.p = I;
        Normal nn(n.x,n.y,n.z);
        p_isect->dg.nn = nn*(1./nn.Length());
        p_isect->primitive = this;
        return true;
    }

    bool IntersectP(const Ray &r) const /*!< Identique à la fonction Intersect (ne retourne que true ou false).*/
    {
        Vector n;
        Point I;
        double beta, gamma;
        return DoIntersect(r, A(), B(), C(), I, n, beta, gamma);
    }

    inline bool CanIntersect() const { return true;}

    /*!< Not implemented*/
    const AreaLight *GetAreaLight() const {return (AreaLight*)NULL;}
    /*!< Not implemented*/
    BSDF *GetBSDF(const DifferentialGeometry &dg, const Transform &ObjectToWorld, MemoryArena &arena) const {return (BSDF*)NULL;}
    /*!< Not implemented*/
    BSSRDF *GetBSSRDF(const DifferentialGeometry &dg, const Transform &ObjectToWorld, MemoryArena &arena) const {return (BSSRDF*)NULL;}
};

inline void RobustGetLine(ifstream & ifs, string & line)
{
    getline(ifs, line);
    if(line[line.size()-1] == '\r')
        line.erase(line.size()-1);
}

template <typename T> void Read(ifstream ifs, T data)
{
    ifs.get((char*)&data, sizeof(T));
}

bool loadPly(const string &filepath,
             vector<ColoredPoint> & v_pt,
             vector<Reference<Primitive> > & prims,
             Point & pivot)
{
    LOG("Parsing " << filepath);
    // read header
    PlyHeader header(filepath);
    // get Pivot
    pivot.x = header.m_georef.m_E;
    pivot.y = header.m_georef.m_N;
    pivot.z = header.m_georef.m_H;
    // checks
    if(header.mv_element.size() != 2) ERR("There should be 2 elements (vertex and face), not " << header.mv_element.size());
    if(header.mv_element[0].m_name != "vertex") LOG("First element should be named vertex, not " << header.mv_element[0].m_name);
    if(header.mv_element[1].m_name != "face") LOG("Second element should be named face, not " << header.mv_element[1].m_name);
    int n_vertex_prop = header.mv_element[0].mv_property.size();
    int n_face_prop = header.mv_element[1].mv_property.size();
    if(n_vertex_prop != 6) ERR(n_vertex_prop << "!=6 vertex properties");
    if(n_face_prop != 1) ERR(n_face_prop << "!=1 face properties");
    vector<string> vertex_property(6);
    vertex_property[0] = "x";
    vertex_property[1] = "y";
    vertex_property[2] = "z";
    vertex_property[3] = "red";
    vertex_property[4] = "green";
    vertex_property[5] = "blue";
    for(int i=0; i<6; i++) if(header.mv_element[0].mv_property[i].m_name != vertex_property[i])
        LOG("vertex property " << i << " should be named " << vertex_property[i] << " not " << header.mv_element[0].mv_property[i].m_name);

    PlyProperty face_prop = header.mv_element[1].mv_property[0];
    if(face_prop.m_name != "vertex_indices")
        LOG("First face property should be named vertex_indices and not " << face_prop.m_name);
    if(face_prop.m_type != "list" || face_prop.m_list_size_type != "uchar" || face_prop.m_list_elem_type != "int")
        ERR("First face property should be of type 'list uchar int' and not " <<
            face_prop.m_type<<" "<<face_prop.m_list_size_type <<" "<<face_prop.m_list_elem_type);

    // all sizes
    unsigned int n_vertex = header.mv_element[0].m_num, n_tri = header.mv_element[1].m_num;
    int vertex_size=3*sizeof(float)+3*sizeof(unsigned char), tri_idx_size = sizeof(unsigned char)+3*sizeof(int);
    const int ptsSize = n_vertex*vertex_size, triSize=n_tri*tri_idx_size, dataSize = ptsSize+triSize;

    int bad_size=0;
    if(header.m_format.m_name == "ascii")
    {
        LOG("ascii");
        string line="";
        ifstream ifs(filepath.c_str());
        while(line != "end_header" && getline (ifs,line)) ;
        for(int i=0; i<n_vertex; i++)
        {
            ColoredPoint cp;
            int r,g,b;
            //getline (ifs,line);
            //sscanf(line.c_str(),"%f %f %f %u %u %u", &cp.x, &cp.y, & cp.z, &r, &g, &b);
            ifs >> cp.x >>cp.y >> cp.z >> r >> g >> b;
            cp.r=r; cp.b=b; cp.g=g;
            v_pt.push_back(cp);
        }
        // triangle indices
        for(int i=0; i<n_vertex; i++)
        {
            unsigned int n,a,b,c;
            //getline (ifs,line);
            //sscanf(line.c_str(),"%u %u %u %u", &n, a, c, b);
            if(n != 3) bad_size++;
            prims.push_back(Reference<Primitive>(new KdTreeTriangle(a,b,c, &v_pt)));
        }
        ifs.close();
    }
    else if(header.m_format.m_name == "binary_little_endian")
    {
        LOG("binary_litte_endian");
        ifstream bin_ifs(filepath.c_str(), ios::binary);
        if(!bin_ifs.good()) {cout << "Failed to open " << filepath << endl; return false;}
        bin_ifs.seekg(0, ios::end);
        LOG("Binary file size: " << bin_ifs.tellg() << " dataSize: " << ptsSize << "+" << triSize << "=" << dataSize << endl);
        bin_ifs.seekg(-dataSize, ios::end);
        cout << "Header ends at " << bin_ifs.tellg() << endl;
        for(int i_vertex=0; i_vertex<n_vertex; i_vertex++)
        {
            ColoredPoint cp;
            bin_ifs.read((char*)&cp.x, sizeof(float));
            bin_ifs.read((char*)&cp.y, sizeof(float));
            bin_ifs.read((char*)&cp.z, sizeof(float));
            bin_ifs.read((char*)&cp.r, sizeof(unsigned char));
            bin_ifs.read((char*)&cp.g, sizeof(unsigned char));
            bin_ifs.read((char*)&cp.b, sizeof(unsigned char));
            v_pt.push_back(cp);
        }

        for(int i_tri=0; i_tri<n_tri; i_tri++)
        {
            unsigned char n;
            unsigned int a,b,c;
            bin_ifs.read((char*)&n, sizeof(unsigned char));
            bin_ifs.read((char*)&a, sizeof(unsigned int));
            bin_ifs.read((char*)&b, sizeof(unsigned int));
            bin_ifs.read((char*)&c, sizeof(unsigned int));
            if(n != 3) bad_size++;
            prims.push_back(Reference<Primitive>(new KdTreeTriangle(a,b,c, &v_pt)));
        }
    }
    else ERR("Unsupported format " << header.m_format.m_name);
    if(bad_size>0) LOG("Unsupported face size " << bad_size);
    return true;
}

int main(int argc, char **argv)
{
    if(argc<4)
    {
        cout << "Usage: " << argv[0] << " input.ori.xml input.ply depth_img rgb_img" << endl;
        return 1;
    }
    Ori ori;
    string in_ori(argv[1]);
    string in_ply(argv[2]);
    string depth_img(argv[3]);
    string rgb_img(argv[4]);

    ori.Read (in_ori );
    int nl = ori.intrinsic()->height();
    int nc = ori.intrinsic()->width();

    vector<ColoredPoint> v_pt;
    vector<Reference<Primitive> > prims;
    Point pivot;
    if(!loadPly(in_ply, v_pt, prims, pivot)) return 1;

    // Remove pivot from ori center
    ori.extrinsic().sommet()[0] -= pivot.x;
    ori.extrinsic().sommet()[1] -= pivot.y;
    ori.extrinsic().sommet()[2] -= pivot.z;
    Point sommet(ori.extrinsic().sommet()[0], ori.extrinsic().sommet()[1], ori.extrinsic().sommet()[2]);

    cout << "Building kdtree..." << endl;
    KdTreeAccel kdtree(prims); // TODO: play with the params
    int n_inter=0;
    cv::Mat depth(nl, nc, CV_8UC1);
    cv::Mat rgb(nl, nc, CV_8UC3);
    float d_norm = 255.f/20.f;
    for(int l=0; l<nl; l++) for(int c=0; c<nc; c++)
    {
        double x0,y0,z0,x1,y1,z1;
        ori.ImageAndDepthToGround(c,l,0.,x0,y0,z0);
        ori.ImageAndDepthToGround(c,l,1.,x1,y1,z1);
        Point P0(x0,y0,z0), P1(x1,y1,z1);
        Ray ray(P0, P1-P0, 0.f);
        Intersection isect;
        unsigned char dc = 0, rc=0, gc=0, bc=0;
        if(kdtree.Intersect(ray,&isect))
        {
            n_inter++;
            Vector v = isect.dg.p - sommet;
            float d = d_norm*v.Length();
            if(d<0.f) d=0.f;
            if(d>255.f) d=255.f;
            dc = (unsigned char)(255.-d);
            rc = (unsigned char)isect.dg.dudx;
            gc = (unsigned char)isect.dg.dvdx;
            bc = (unsigned char)isect.dg.dudy;
        }
        depth.at<unsigned char>(l,c)=dc;
        rgb.at<cv::Vec3b>(l,c)[0]=bc;
        rgb.at<cv::Vec3b>(l,c)[1]=gc;
        rgb.at<cv::Vec3b>(l,c)[2]=rc;
        if(l%100==0 && c==0) cout << l << " " << n_inter << endl;
    }
    cout << n_inter << "/" << nl*nc << " intersections" << endl;
    cv::imwrite(depth_img, depth);
    cv::imwrite(rgb_img, rgb);
    return 0;
}
