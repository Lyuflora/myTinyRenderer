#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(20,  255, 20,  255);
const int width = 800;
const int height = 800;
Model *model = NULL;
int *zbuffer = NULL;
TGAColor getDiffuse(TGAImage diffuse, int u, int v) {
	TGAColor color;
	color = diffuse.get(u, v);
	return color;
}
const Vec3f light_dir = Vec3f(0, 0, -1);

Vec3f cross(Vec3f v1, Vec3f v2) {
	Vec3f v = Vec3f(v1.y*v2.z-v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.z*v2.y - v1.y*v2.z);
	return v;
}

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
	bool isSteep = false;
	if (std::abs(x1 - x0) < std::abs(y1 - y0)) {
		// transpose
		std::swap(x0, y0);
		std::swap(x1, y1);
		isSteep = true;
	}

	if (x1 < x0) {
		std::swap(x1, x0);
		std::swap(y1, y0);
	}

	/*float derror = std::abs((y1 - y0) / float(x1 - x0));
	float error = 0;*/
	// 优化：乘 (x1 - x0)*2
	int derror2 = std::abs(y1 - y0)*2;
	int error2 = 0;
	int y = y0;
	for (int x = x0; x < x1; x++) {
		float t = (x - x0) / float(x1 - x0);

		if (isSteep) {
			image.set(y, x, color);
		}
		else {
			image.set(x, y, color);
		}
		
		// 如果渲染的像素直线和真正理想的直线距离过大(正负0.5)，需要在y上进行1像素的位移
		// 每个循环向右一个像素，x+1, y+derror，误差是error+derror，绝对值非负
		// 如果它超过0.5，就进行向上(斜率为正)、向下的平移(斜率为负)
		//error += derror;		
		//if (error > 0.5) {
		//	y += (y1 > y0 ? 1:-1);
		//	 error -= 1.;
		//}

		error2 += derror2;
		if (error2 > std::abs(float(x1 - x0))) {
			y += (y1 > y0 ? 1 : -1);
			error2 -= std::abs(float(x1 - x0)) * 2;
		}
	}
}

// input: vertices of the triangle, a pixel P
// output: barycentric coordinate of P
Vec3f barycentric(Vec3f p0, Vec3f p1, Vec3f p2, Vec2i P) {
	// AB × AC
	Vec3f u = Vec3f(p2.x - p0.x, p1.x - p0.x, p0.x- P.x)^
		Vec3f(p2.y - p0.y, p1.y - p0.y, p0.y - P.y);
	if (std::abs(u.z) <1) {
		// 三角形退化 a+b=c
		return Vec3f(-1, 1, 1);
	}
	// barycentric coordinate: P = (1-u-v)A + u B+ v C
	return  Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
}
Vec3f barycentric2i(Vec2i p0, Vec2i p1, Vec2i p2, Vec2i P) {
	// AB × AC
	Vec3f u = Vec3f(p2.x - p0.x, p1.x - p0.x, p0.x - P.x) ^
		Vec3f(p2.y - p0.y, p1.y - p0.y, p0.y - P.y);
	if (std::abs(u.z) < 1) {
		// 三角形退化 a+b=c
		return Vec3f(-1, 1, 1);
	}
	// barycentric coordinate: P = (1-u-v)A + u B+ v C
	return  Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
}
void triangle(Vec3f t0, Vec3f t1, Vec3f t2, TGAImage &image, float *zbuffer, TGAColor color) {
	Vec2f bbox_min = Vec2f(image.get_width() - 1, image.get_height() - 1);
	Vec2f bbox_max = Vec2f(0, 0);
	Vec2f clamp = Vec2f(image.get_width() - 1, image.get_height() - 1);

	Vec3f *t = new Vec3f[3];
	t[0] = t0; t[1] = t1; t[2] = t2;

	for (int i = 0; i < 3; i++) {
		// 套一层是为了用边界值限制
		bbox_min.x = std::max(0.f, std::min(bbox_min.x, t[i].x));
		bbox_min.y = std::max(0.f, std::min(bbox_min.y, t[i].y));

		bbox_max.x = std::min(clamp.x, std::max(bbox_max.x, t[i].x));
		bbox_max.y = std::min(clamp.y, std::max(bbox_max.y, t[i].y));
	}

	for (int i = bbox_min.x; i <= bbox_max.x; i++) {
		for (int j = bbox_min.y; j <= bbox_max.y; j++) {
			Vec3f bc_coord = barycentric(t0, t1, t2, Vec2i(i, j));
			if (bc_coord.x < 0 || bc_coord.y < 0 || bc_coord.z < 0)	continue;
			// 计算该点的深度
			// 用重心坐标插值的方式计算三角形内一点的坐标，只需深度z坐标，所以只在z上插值
			float z = 0.;
			z += bc_coord.x * t[0].z + bc_coord.y * t[1].z + bc_coord.z * t[2].z;

			// 注意zbuffer的符号
			if (zbuffer[i + width * j] < bc_coord.z) {
				image.set(i, j, color);
				zbuffer[i + width * j] = bc_coord.z;
			}
		}
	}

	delete t;
}
void triangle_tex(Vec3f t0, Vec3f t1, Vec3f t2, Vec2i uv0, Vec2i uv1, Vec2i uv2, TGAImage &image, float intensity, int*zbuffer, TGAImage texture) {
	if (t0.y == t1.y && t0.y == t2.y) return; 
	if (t0.y > t1.y) { std::swap(t0, t1); std::swap(uv0, uv1); }
	if (t0.y > t2.y) { std::swap(t0, t2); std::swap(uv0, uv2); }
	if (t1.y > t2.y) { std::swap(t1, t2); std::swap(uv1, uv2); }

	int total_height = t2.y - t0.y;
	for (int i = 0; i < total_height; i++) {
		bool second_half = i > t1.y - t0.y || t1.y == t0.y;
		int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
		float alpha = (float)i / total_height;
		float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height; // be careful: with above conditions no division by zero here
		Vec3f A = t0 + Vec3f(t2 - t0)*alpha;
		Vec3f B = second_half ? t1 + Vec3f(t2 - t1)*beta : t0 + Vec3f(t1 - t0)*beta;
		Vec2i uvA = uv0 + (uv2 - uv0)*alpha;
		Vec2i uvB = second_half ? uv1 + (uv2 - uv1)*beta : uv0 + (uv1 - uv0)*beta;
		if (A.x > B.x) { std::swap(A, B); std::swap(uvA, uvB); }
		for (int j = A.x; j <= B.x; j++) {
			float phi = B.x == A.x ? 1. : (float)(j - A.x) / (float)(B.x - A.x);
			Vec3f   P = Vec3f(A) + Vec3f(B - A)*phi;
			Vec2i uvP = uvA + (uvB - uvA)*phi;
			int idx = P.x + P.y*width;
			//std::cout <<"P.z:"<< P.z <<"; zbuffer:"<< zbuffer[idx] << std::endl;
			if (zbuffer[idx] < P.z) {
				zbuffer[idx] = P.z;
				//std::cout << "new zbuffer:" << zbuffer[idx] << std::endl;
				TGAColor color = model->diffuse(uvP);
				image.set(P.x, P.y, TGAColor(color.r*intensity, color.g*intensity, color.b*intensity, 255));
			}
		}
	}
}

Vec3f world_to_screen(Vec3f v_world) {
	return Vec3f((v_world.x + 1.)*width / 2., (v_world.y + 1.)*height / 2., v_world.z);
}


int main(int argc, char** argv) {

	model = new Model("obj/african_head/african_head.obj");
	
	TGAImage diffuse_tex;
	diffuse_tex.read_tga_file("african_head_diffuse.tga");
    
	TGAImage image(width, height, TGAImage::RGB);

	// initialize zbuffer
	
	zbuffer = new int[width*height+1000];
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			zbuffer[i*width + j] = std::numeric_limits<int>::min();
		}
	}
	std::cout << "zbuffer_created" << std::endl;
	
	for (int i = 0; i < model->nfaces(); i++) {
		std::vector<int> face = model->face(i);
		Vec3f *v_world_coord = new Vec3f[3];
		Vec3f *v_screen_coord = new Vec3f[3];
		for (int j = 0; j < 3; j++) {
			v_world_coord[j] = model->vert(face[j]);
			v_screen_coord[j] = world_to_screen(v_world_coord[j]);
		}

		// 面的法向量
		Vec3f n = (v_world_coord[0] - v_world_coord[1]) ^ (v_world_coord[0] - v_world_coord[2]);	// 为什么n是错误的--应该是-n,方向反了
		Vec3f n_1 = (v_world_coord[2] - v_world_coord[0]) ^ (v_world_coord[1] - v_world_coord[0]);
		n.normalize();
		n_1.normalize();
		
		// 计算光照
		float intensity = n_1 * light_dir;
		if (intensity > 0) {
			// 纹理坐标
			Vec2i uv[3];
			for (int k = 0; k < 3; k++)
				uv[k] = model->uv(i, k);
			triangle_tex(v_screen_coord[0], v_screen_coord[1], v_screen_coord[2], uv[0], uv[1], uv[2], image, intensity, zbuffer, diffuse_tex);
		}
		delete v_world_coord;
		delete v_screen_coord;
	}

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
	delete model;
	TGAImage zbimage(width, height, TGAImage::GRAYSCALE);
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			zbimage.set(i, j, TGAColor(zbuffer[i + j * width], 1));
		}
	}
	zbimage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	zbimage.write_tga_file("zbuffer.tga");
	delete[] zbuffer;
	std::cout << "done" << std::endl;
    return 0;
}

