#include "tgaimage.h"
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Vec3f barycentric(Vec2i *pts, Vec2i P) {
	// AB × AC
	Vec3f u = Vec3f(pts[2].x - pts[0].x, pts[1].x - pts[0].x, pts[0].x- P.x)^
		Vec3f(pts[2].y - pts[0].y, pts[1].y - pts[0].y, P.x - pts[0].y);
	if (std::abs(u.z) <1) {
		// 三角形退化 a+b=c
		return Vec3f(-1, 1, 1);
	}
	// barycentric coordinate: P = (1-u.x-u.y)A + u.x B+ u.y C
	return  Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);;

}
int main(int argc, char** argv) {
        TGAImage image(100, 100, TGAImage::RGB);
        image.set(52, 41, red);
        image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        image.write_tga_file("output.tga");
        return 0;
}