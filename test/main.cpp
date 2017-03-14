#include "../src/Ori.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    if(argc<3)
    {
        std::cout << "Usage: " << argv[0] << " input.ori.xml output.ori.xml" << std::endl;
        return 1;
    }
    Ori ori;
    std::string in (argv[1]);
    std::string out(argv[2]);

    ori.Read (in );
    ori.Write(out);

    double x = 0;
    double y = 0;
    double z = 0;
    double c = 10;
    double l = 100;
    double d = 10;

    std::cout.precision(4);

    bool ok = true;
    std::cout << ok << " : xyz=(" << x << "," << y << "," << z << "), cld=(" << c << "," << l << "," << d << ")" << std::endl;

    ok = ori.ImageAndDepthToGround(c,l,d,x,y,z);
    std::cout << ok << " : xyz=(" << x << "," << y << "," << z << "), cld=(" << c << "," << l << "," << d << ")" << std::endl;

    c = l = d = 0;
    ok = ori.GroundToImage(x,y,z,c,l);
    std::cout << ok << " : xyz=(" << x << "," << y << "," << z << "), cl=(" << c << "," << l << ")" << std::endl;

    c = l = d = 0;
    ok = ori.GroundToImageAndDepth(x,y,z,c,l,d);
    std::cout << ok << " : xyz=(" << x << "," << y << "," << z << "), cld=(" << c << "," << l << "," << d << ")" << std::endl;

    x = y = z = 0;
    ok = ori.ImageAndDepthToGround(c,l,d,x,y,z);
    std::cout << ok << " : xyz=(" << x << "," << y << "," << z << "), cld=(" << c << "," << l << "," << d << ")" << std::endl;

    return 0;
}
