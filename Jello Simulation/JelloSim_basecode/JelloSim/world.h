#ifndef WORLD_H
#define WORLD_H

#include <string>
#include <vector>
#include "vec.h"


class World
{
public:
    World(const std::string& filename);
	World(const World& filename);
    virtual ~World();

    void LoadFromFile(const std::string& filename);
    virtual void Draw();

public:
    enum ShapeType {SPHERE, GROUND, CUBE, CYLINDER};
    class Shape
    {
    public:
        vec3 pos; 
		double angle;
        ShapeType GetType() const { return type; }
		void move(double hor, double vert);
		void rotate(double ang);
		vec3 start, end;
        double r;
		double hx, hy, hz;
		Shape(ShapeType t) : type(t), pos(0,0,0), angle(0.0) {
			r =  0;
			hx = 0;
			hy = 0;
			hz = 0;
			start = vec3();
			end = vec3();
		}
        ShapeType type;
		Shape(Shape& s) : type(s.type), pos(s.pos), angle(s.angle) {

			r =  s.r;
			hx = s.hx;
			hy = s.hy;
			hz = s.hz;
			start = s.start;
			end = s.end;
		 }
		Shape(const Shape* s) {
			r =  s->r;
			hx = s->hx;
			hy = s->hy;
			hz = s->hz;
			start = s->start;
			end = s->end;
			type =s->type;
			pos = s->pos;
			angle = s->angle;
		}


		Shape& Shape::operator=(const Shape& s) {
			if (&s == this) return *this;
			r =  s.r;
			hx = s.hx;
			hy = s.hy;
			hz = s.hz;
			start = s.start;
			end = s.end;
			type =s.type;
			pos = s.pos;
			angle = s.angle;

			return *this;
		}



    protected:
        
    };

    class Ground : public Shape
    {
    public:
        Ground() : Shape(GROUND) {}
    };

    class Sphere : public Shape
    {
    public:
        Sphere() : Shape(SPHERE) {
		  r = 1.0;
		}
		Sphere& Sphere::operator=(const Sphere& s) {
			if (&s == this) return *this;
			r =  s.r;
			hx = s.hx;
			hy = s.hy;
			hz = s.hz;
			start = s.start;
			end = s.end;
			type =s.type;
			pos = s.pos;
			angle = s.angle;

			return *this;
		}
       
    };

    class Cube : public Shape
    {
    public:
        Cube() : Shape(CUBE) {
			hx = (0.5); 
			hy = (0.5);
			hz = (0.5);
		}
       // double hx, hy, hz;
    };

    class Cylinder : public Shape
    {
    public:
        Cylinder() : Shape(CYLINDER) {
			r = 1.0;
		}
       // vec3 start, end;
       // double r;
    };

    std::vector<Shape*> m_shapes;
};

#endif