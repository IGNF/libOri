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

    std::cout.precision(21);

    bool ok = true;
    std::cout << ok << " : " << x << "," << y << "," << z << " : " << c << "," << l << std::endl;

    ok = ori.ImageAndDepthToGround(c,l,1.,x,y,z);
    std::cout << ok << " : " << x << "," << y << "," << z << " : " << c << "," << l << std::endl;

    ok = ori.GroundToImage(x,y,z,c,l);
    std::cout << ok << " : " << x << "," << y << "," << z << " : " << c << "," << l << std::endl;

    ok = ori.ImageAndDepthToGround(c,l,1.,x,y,z);
    std::cout << ok << " : " << x << "," << y << "," << z << " : " << c << "," << l << std::endl;
    return 0;
}
