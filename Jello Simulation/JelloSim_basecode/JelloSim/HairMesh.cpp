#include "HairMesh.h"
#include <GL/glut.h>
#include <algorithm>
#include "matrix.h"

double MATH_PI = M_PI;

using namespace math;


double HairMesh::g_structuralKs = 5000.000; //3k 
double HairMesh::g_structuralKd = 5.000; //10
double HairMesh::g_shearKs = 4000.000; //4000
double HairMesh::g_shearKd = 6.00; //10
double HairMesh::g_penaltyKs = 100.000; //5000
double HairMesh::g_penaltyKd = 20.000; //10

double HairMesh::friction_Mk = 0.5;
double HairMesh::friction_Amp = 2;
double HairMesh::COLLISION_THRESHOLD = 0.01; 
double HairMesh::jelloStartY = 1.3; //0.0

//Da Hair Vars
double HairMesh::g_bendKs = 2000.0000; //3000
double HairMesh::g_bendKd = 0.70; // 7

double HairMesh::g_torsionKs = 00.0;
double HairMesh::g_torsionKd = 0.00;

double HairMesh::g_edgeKs = 7000.0;
double HairMesh::g_edgeKd = 0.50;

double HairMesh::g_stictionKs = 100.0000;
double HairMesh::g_stictionKd = 50.0;



double HairMesh::g_attachmentKs = 0.000;
double HairMesh::g_attachmentKd = 0.000;

void HairMesh::moveHairStrandRotate(int strandNum, double angle) {
	for (unsigned int sNum =0; sNum < StrandList.size(); sNum++) {
		HairStrand& strand = StrandList.getStrand(sNum);
		ParticleList& strandParticles = strand.strandParticles;
		for (unsigned int i = 0; i < strandParticles.size(); i++) {
			//Only move the root particle
			Particle& pt = strandParticles[i];
			double rads = angle*(MATH_PI/180);
			RotationMatrix<double> a = RotationMatrix<double>(1, rads);
			pt.position = a*pt.position;
			break;
		}
	}
}

void HairMesh::moveHairStrandTranslate(int strandNum, double hor, double vert) {
	for (unsigned int sNum =0; sNum < StrandList.size(); sNum++) {
		HairStrand& strand = StrandList.getStrand(sNum);
		ParticleList& strandParticles = strand.strandParticles;
		for (unsigned int i = 0; i < strandParticles.size(); i++) {
			Particle& pt = strandParticles[i];
			//double rads = angle*(MATH_PI/180);
			//RotationMatrix<double> a = RotationMatrix<double>(1, rads);
			pt.position[0] += hor;
			pt.position[1] += vert;
			break;
		}
	}
}


HairMesh::HairMesh() :     
    m_integrationType(HairMesh::RK4), m_drawflags(MESH | SHOULD_DRAW_HAIR),
    m_cols(0), m_rows(0), m_stacks(0), m_width(0.0), m_height(0.0), m_depth(0.0)
{
	SHOULD_DRAW_HAIR = true;
	SHOULD_DRAW_HAIR_PARTICLES = false;
	SHOULD_DRAW_GHOST_PARTICLES = false;
	SHOULD_DRAW_STICTION_PARTICLES = true;
	
    SetSize(1.0, 1.0, 1.0);
    SetGridSize(6, 6, 6);
}

HairMesh::~HairMesh()
{
}

void HairMesh::Reset()
{
    InitHairMesh();
}

HairMesh::Particle& HairMesh::GetParticle(HairMesh::ParticleGrid& grid, int i, int j, int k)
{
    return grid[i][j][k];
}

HairMesh::Particle& HairMesh::GetParticle(HairMesh::ParticleGrid& grid, int idx)
{
    int i,j,k;
    GetCell(idx, i, j, k);
    return GetParticle(grid, i,j,k);
}

const HairMesh::Particle& HairMesh::GetParticle(const HairMesh::ParticleGrid& grid, int i, int j, int k) const
{
    return grid[i][j][k];
}

const HairMesh::Particle& HairMesh::GetParticle(const HairMesh::ParticleGrid& grid, int idx) const
{
    int i,j,k;
    GetCell(idx, i, j, k);
    return GetParticle(grid, i,j,k);
}

//bool HairMesh::isInterior(const HairMesh::Spring& s) const
//{
//    int i1,j1,k1,i2,j2,k2;
//    GetCell(s.m_p1, i1, j1, k1);
//    GetCell(s.m_p2, i2, j2, k2);
//    return isInterior(i1,j1,k1) || isInterior(i2,j2,k2);
//}


bool HairMesh::isInterior(int idx) const
{
    int i,j,k;
    GetCell(idx, i, j, k);
    return isInterior(i,j,k);
}

bool HairMesh::isInterior(int i, int j, int k) const
{
    return (i*j*k*(m_rows-i)*(m_cols-j)*(m_stacks-k) != 0);
}

void HairMesh::SetGridSize(int cols, int rows, int stacks)
{
    m_cols = cols;
    m_rows = rows;
    m_stacks = stacks;

    if (m_cols > 0 && m_rows > 0 && m_stacks > 0)
    {
        m_vparticles.resize(m_rows+1);
        for (int i = 0; i < m_rows+1; i++)
        {
            m_vparticles[i].resize(m_cols+1);
            for (int j = 0; j < m_cols+1; j++)
            {
                m_vparticles[i][j].resize(m_stacks+1);
            }
        }
    }
    InitHairMesh();
}

int HairMesh::GetGridCols() const
{
    return m_cols;
}

int HairMesh::GetGridRows() const
{
    return m_rows;
}

int HairMesh::GetGridStacks() const
{
    return m_stacks;
}

void HairMesh::SetSize(float width, float height, float depth)
{
    m_width = width;
    m_height = height;
    m_depth = depth;
    InitHairMesh();
}

float HairMesh::GetWidth() const
{
    return m_width;
}

float HairMesh::GetHeight() const
{
    return m_height;
}

float HairMesh::GetDepth() const
{
    return m_depth;
}

int HairMesh::GetIndex(int i, int j, int k) const
{
    int cols = j;
    int rows = i*(m_cols+1);
    int stacks = k*(m_cols+1)*(m_rows+1);
    return cols + rows + stacks;
}

#define ROUND(x) (floor(x + 0.5))
#define FLOOR(x) (floor(x))
#define FRACT(x) (x - FLOOR(x))
void HairMesh::GetCell(int idx, int& i, int &j, int& k) const
{
    float rows = m_rows+1;
    float cols = m_cols+1;
    float stacks = m_stacks+1;

    // derived from idx = cols*(rows*k + i) + j
    float tmp = FLOOR(idx/cols);
    j = (int) ROUND(cols*(FRACT(idx/cols)));
    i = (int) ROUND(rows*(FRACT(tmp/rows)));
    k = (int) FLOOR(tmp/rows);
}

void HairMesh::AddStructuralSpring(Particle& p1, Particle& p2)
{
    //double restLen = (p1.position - p2.position).Length();
    //m_vsprings.push_back(Spring(STRUCTURAL, p1.index, p2.index, g_structuralKs, g_structuralKd, restLen));
}


void HairMesh::AddShearSpring(HairMesh::Particle& p1, HairMesh::Particle& p2)
{
    //double restLen = (p1.position - p2.position).Length();
    //m_vsprings.push_back(Spring(SHEAR, p1.index, p2.index, g_shearKs, g_shearKd, restLen));
}






void HairMesh::SetIntegrationType(HairMesh::IntegrationType type)
{
    m_integrationType = type;
}

HairMesh::IntegrationType HairMesh::GetIntegrationType() const
{
    return m_integrationType;
}

void HairMesh::SetDrawFlags(unsigned int flags)
{
    m_drawflags = flags;
}

unsigned int HairMesh::GetDrawFlags() const
{
    return m_drawflags;
}

void HairMesh::DrawMesh(const vec3& eyePos)
{
    const ParticleGrid& g = m_vparticles;
    float red[4] = {1.0,0.4,0.4,0.8};
    float white[4] = {1.0,1.0,1.0,1.0};
    float pink[4] = {0.5,0.0,0.0,1.0};
    float black[4] = {0.0,0.0,0.0,1.0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, red);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pink);

    for (unsigned int i = 0; i < m_mesh.size(); i++)
    {        
       m_mesh[i].CalcDistToEye(*this, eyePos);
    }
    std::sort(m_mesh.begin(), m_mesh.end(), FaceMesh::compare);
    for (unsigned int i = 0; i < m_mesh.size(); i++)
    {        
       m_mesh[i].Draw(*this);
    }

    //glDisable(GL_LIGHTING);
    //for (unsigned int i = 0; i < m_mesh.size(); i++)
    //{        
    //   m_mesh[i].DrawNormals(*this);
    //}

}

void HairMesh::DrawSprings(double a)
{ //Do nothing
}

void HairMesh::DrawCollisionNormals()
{
    const ParticleGrid& g = m_vparticles;
    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 0.0);
    for(unsigned int i = 0; i < m_vcollisions.size(); i++)
    {
       Intersection intersection = m_vcollisions[i];
       if (isInterior(intersection.m_p)) continue;

       const Particle& pt = GetParticle(g, intersection.m_p);
       vec3 normal = intersection.m_normal;
       vec3 end = pt.position + 0.2 * normal;
       glVertex3f(pt.position[0], pt.position[1], pt.position[2]);
       glVertex3f(end[0], end[1], end[2]);
    }     
    glEnd();
}

void HairMesh::DrawForces()
{
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    for (int i = 0; i < m_rows+1; i++)
    {
        for (int j = 0; j < m_cols+1; j++)
        {
            for (int k = 0; k < m_stacks+1; k++)
            {
                Particle p = m_vparticles[i][j][k];
                if (isInterior(i,j,k)) continue;

                vec3 normal = p.force.Normalize();
                vec3 end = p.position + 0.1 * normal;
                glVertex3f(p.position[0], p.position[1], p.position[2]);
                glVertex3f(end[0], end[1], end[2]);
            }
        }
    }     
    glEnd();
}

void HairMesh::Draw(const vec3& eyePos)
{
    if (m_drawflags & MESH) DrawMesh(eyePos);

    if (m_drawflags & (STRUCTURAL|BEND|SHEAR))
    {
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glLineWidth(1.0);
        DrawSprings(0.2);
        glLineWidth(1.5);
        glEnable(GL_DEPTH_TEST);
        DrawSprings(0.4);
    }

    if (m_drawflags & NORMALS) DrawCollisionNormals();
    if (m_drawflags & FORCES) DrawForces();


	DrawHair(); 
	DrawHairParticles();
	DrawHairSprings();

    glEnable(GL_LIGHTING);

}

void HairMesh::Update(double dt, const World& world, const vec3& externalForces)
{
    m_externalForces = externalForces;
	ComputeHairForces(StrandList, dt);
	applyStrainLimiting(dt);		
	CheckParticleCollisions(world);
	CheckStrandCollisions();
	applyStiction();
	applyImpulse();
	ResolveHairContacts();
	ResolveHairCollisions();
	RK4Integrate(dt);
	applyStrainLimiting(dt);	
	ComputeHairForces(StrandList, dt);
}

void HairMesh::CheckForCollisions(ParticleGrid& grid, const World& world)
{ //Not Used
}

void HairMesh::ComputeForces(ParticleGrid& grid)
{// Not Used
}

//Object particle is below the surface of another object
void HairMesh::ResolveContacts(ParticleGrid& grid)
{
	//Not Used
}


//Object is within a threshold about the surface of another object
void HairMesh::ResolveCollisions(ParticleGrid& grid)
{
   //Not Used
}

bool HairMesh::FloorIntersection(Particle& p, int strandIndex, int particleIndex, Intersection& intersection)
{
		vec3 X = p.position;		//Particle Position
		double pY = X[1];
		vec3 V = p.velocity;		//Particle Velocity
		vec3 P = vec3(0.0, 0.0, 0.0);	//Floor Position 
		vec3 N = vec3(0.0, 1.0, 0.0);	//Floor Normal
		
		//HAve point position and project it onto the normal, if the projection length > COLLISION Threshold, no collision
		double projectedV = (V*N);
		
		//If our Y location of the point is above the threshold or the projected velocity is moving up and the particle is within the threshold range.
		if (pY > COLLISION_THRESHOLD || (pY < COLLISION_THRESHOLD && projectedV >= 0.0)) {
			//Greater than length of collision detection space - no collison
			return false;
		}

		//Check if point is within the collision space - Collision
		if (0 <= pY && pY <= COLLISION_THRESHOLD) {
			//cout << "Floor Collision\n";
			intersection.m_p = particleIndex;
			intersection.m_strand = strandIndex;
			intersection.m_distance = fabs(COLLISION_THRESHOLD-pY);			//Distance from top of the collision threshold
			intersection.m_type = IntersectionType::COLLISION;
			intersection.m_normal = N;
			intersection.m_ground_friction = friction_Mk;
			return true;
		}
		//Check if point is below surface of floor - Contact
		else if (pY < 0) {
			//cout<< "Floor Contact\n";
			intersection.m_p = particleIndex;
			intersection.m_strand = strandIndex;
			intersection.m_distance = fabs(0.0-pY);							//Distance from the surface
			intersection.m_type = IntersectionType::CONTACT;
			intersection.m_normal = N;
			intersection.m_ground_friction = friction_Mk;
			return true;
		}
    return false;
}


bool HairMesh::SphereIntersection(Particle& p, int strandIndex, int particleIndex, World::Sphere* sphere, Intersection& intersection) {
	//cout << "strandIndex within sphere intersection: " << strandIndex << endl;
	vec3 s = sphere->pos;
	double radius = sphere->r; 
	vec3 x = p.position;
	vec3 V = p.velocity;

	//Vector to particle position
	vec3 a = x-s;
	double aLength = a.Length();

	vec3 N = a/a.Length();
	vec3 R = radius*N;
	double d = (a-R).Length();
	//if we are moving away from the sphere

	if (0.0 <= aLength && aLength <= radius) {
		//Contact in sphere
		intersection.m_type = IntersectionType::CONTACT;
		intersection.m_p = particleIndex;
		intersection.m_strand = strandIndex;
		intersection.m_normal = N;
		intersection.m_distance = R.Length()-aLength;
		return true;
	}
	else if (radius < aLength && aLength < radius+COLLISION_THRESHOLD) {
		//In threshold right above surface
		intersection.m_type = IntersectionType::COLLISION;
		intersection.m_p = particleIndex;
		intersection.m_strand = strandIndex;
		intersection.m_normal = N;
		intersection.m_distance = (radius+COLLISION_THRESHOLD)-aLength;
		return true;
	}
	
	return false;

}


bool HairMesh::CylinderIntersection(Particle& pt, World::Cylinder* cylinder, 
                                 HairMesh::Intersection& intersection)
{
	vec3 cylinderStart = cylinder->start;
    vec3 cylinderEnd = cylinder->end;
    vec3 cylinderAxis = cylinderEnd - cylinderStart;
	double cylinderRadius = cylinder->r; 
	double cylinderHeight = cylinderAxis.Length();

	vec3 x = pt.position;
	vec3 b = x-cylinderStart;
	vec3 a = cylinderEnd - cylinderStart;
	vec3 aNorm = a/a.Length();
	double dotProjB = b*(a);
	vec3 projB = (b*aNorm)*(aNorm);
	vec3 B = projB + cylinderStart;
	//double bLength = B.Length();

	vec3 c = x-B;
	double cLength = c.Length();
	vec3 N = c/cLength;					//Surface Normal

	//double h = (N*cylinderRadius);			//Height to surface from center of cylinder
	double d = cylinderRadius-c.Length();						//height to surface - height of point along surface normal

	//Pt is not within a cylinder radius
	if (0.0 > cLength || cLength > cylinderRadius+COLLISION_THRESHOLD ) {
		return false;
	}
	//Point is outside of the cylinder's top end (not contained w/in the cylinder & threshold)
	if (projB.Length() >= cylinderHeight+COLLISION_THRESHOLD) {	
	//	cout<<"Not within top";
		return false;
	}

	//Bottom case - is angleProjB < 0 && |projB| < Collision Threshold
	if (dotProjB < 0.0 ) {
		if (projB.Length() <= COLLISION_THRESHOLD && cLength <= cylinderRadius) {
			//Collision just outside bottom
			intersection.m_type = IntersectionType::COLLISION;
			intersection.m_normal = aNorm;
			intersection.m_distance = COLLISION_THRESHOLD-projB.Length();
			intersection.m_p = pt.index;
			return true;		}
	}

	vec3 cylinderRevAxis = cylinderStart - cylinderEnd;
	vec3 bEnd = x-cylinderEnd;
	vec3 revNorm = cylinderRevAxis/cylinderRevAxis.Length();
	double dotProjBEnd = bEnd*(a);

	//Top case is |proB| >cylinder 
	if (dotProjB > 0.0 && dotProjBEnd < 0.0) {
		if (cylinderHeight < projB.Length() && projB.Length() < cylinderHeight+COLLISION_THRESHOLD) {
			//Collision just outside top
			intersection.m_type = IntersectionType::COLLISION;
			intersection.m_normal = aNorm;
			intersection.m_distance = fabs((a.Length()+COLLISION_THRESHOLD)-projB.Length());
			intersection.m_p = pt.index;
			return true;
		}
		else if (cylinderHeight-0.15 < projB.Length() && projB.Length() < cylinderHeight) {
			//cout << "TOP CASE \n";
			intersection.m_type = IntersectionType::CONTACT;
			intersection.m_normal = aNorm;
			intersection.m_distance = fabs(a.Length()-projB.Length());
			intersection.m_p = pt.index;
			return true;
		}
	}
	 
	//In Cylinder Collision Case
	if ( 0.0 <= projB.Length() && projB.Length() <= cylinderHeight) {
		//cout<<"In CYL \n";
		if (cylinderRadius < cLength && cLength < cylinderRadius+COLLISION_THRESHOLD) {
			//Near Cylinder Case - Collision 
			//cout << "Close Collision";
			intersection.m_type = IntersectionType::COLLISION;
			intersection.m_normal = c;
			intersection.m_p = pt.index;
			intersection.m_distance = fabs(cylinderRadius+COLLISION_THRESHOLD-cLength);
			return true;
		}
		else if (c.Length() <= cylinderRadius) {
			//cout<<"In Cylinder Contact";
			intersection.m_type = IntersectionType::CONTACT;
			intersection.m_normal = c;
			intersection.m_p = pt.index;
			intersection.m_distance = fabs(d);
			return true;
		}
		else {
			intersection.m_type = IntersectionType::CONTACT;
			intersection.m_normal = c;
			intersection.m_p = pt.index;
			intersection.m_distance = fabs(d);
			return true;
		}
	}

	return false;
}

void HairMesh::EulerIntegrate(double dt)
{
	//Not Used
}

void HairMesh::MidPointIntegrate(double dt)
{
   //Not Used
}

void HairMesh::RK4Integrate(double dt)
{
	//TODO: DO RK4 for STICTION PARTICLE
	double halfdt = 0.5 * dt;

	HairStrandList& source = StrandList;	//Ptr
	HairStrandList target = StrandList;		//Copy

	for (unsigned int i = 0; i < StrandList.size(); i++) {
		target = StrandList;
		HairStrand& sStrand = source.getStrand(i);
		HairStrand& tStrand = target.getStrand(i);
		
		//Get Particles for a strand 
		HairMesh::ParticleList& sParticles = sStrand.strandParticles;
		HairMesh::ParticleList& tParticles = tStrand.strandParticles;
		
		//Step 1 - normal
		HairMesh::ParticleList accum1 = sStrand.strandParticles;
		for (unsigned int j = 0; j < sParticles.size(); j++) {
				Particle& s = sParticles[j];
				Particle& k1 = accum1[j];
				k1.force = s.force;
				k1.velocity = s.velocity;

				Particle& t = tParticles[j];
				t.velocity = s.velocity + halfdt * k1.force * 1/k1.mass;
				t.position = s.position + halfdt * k1.velocity;
		}
		//Step 1- stiction
		HairMesh::StictionParticleList& st_sParticles = sStrand.stictionParticles;
		HairMesh::StictionParticleList& st_tParticles = tStrand.stictionParticles;
		HairMesh::StictionParticleList st_accum1 = sStrand.stictionParticles;
		
		for (unsigned int j = 0; j < st_sParticles.size(); j++) {
				StictionParticle& s = st_sParticles[j];
				StictionParticle& k1 = st_accum1[j];
				k1.force = s.force;
				k1.velocity = s.velocity;

				StictionParticle& t = st_tParticles[j];
				t.velocity = s.velocity + halfdt * k1.force * 1/k1.mass;
				t.position = s.position + halfdt * k1.velocity;
		}

		ComputeHairForces(target, dt);

		//Step 2
		HairMesh::ParticleList accum2 = sStrand.strandParticles;
		for (unsigned int j = 0; j < sParticles.size(); j++) {
				Particle& t =  tParticles[j];
				Particle& k2 = accum2[j];

				k2.force = t.force;
				k2.velocity = t.velocity;

				Particle& s = sParticles[j];
				t.velocity = s.velocity + halfdt * k2.force * 1/k2.mass;
				t.position = s.position + halfdt * k2.velocity;
		}
		//Step 2 - stiction
		HairMesh::StictionParticleList st_accum2 = sStrand.stictionParticles;
		for (unsigned int j = 0; j < st_sParticles.size(); j++) {
				StictionParticle& t =  st_tParticles[j];
				StictionParticle& k2 = st_accum2[j];

				k2.force = t.force;
				k2.velocity = t.velocity;

				StictionParticle& s = st_sParticles[j];
				t.velocity = s.velocity + halfdt * k2.force * 1/k2.mass;
				t.position = s.position + halfdt * k2.velocity;
		}

		ComputeHairForces(target, dt);

		//Step 3
		HairMesh::ParticleList accum3 = sStrand.strandParticles;
		for (unsigned int j = 0; j < sParticles.size(); j++) {
				Particle& t = tParticles[j];
				Particle& k3 = accum3[j];

				k3.force = t.force;
				k3.velocity = t.velocity;

				Particle& s = sParticles[j];
				t.velocity = s.velocity + dt * k3.force * 1/k3.mass;
				t.position = s.position + dt * k3.velocity;
		}

		//Step 3 - Stiction
		HairMesh::StictionParticleList st_accum3 = sStrand.stictionParticles;
		for (unsigned int j = 0; j < st_sParticles.size(); j++) {
				StictionParticle& t = st_tParticles[j];
				StictionParticle& k3 = st_accum3[j];

				k3.force = t.force;
				k3.velocity = t.velocity;

				StictionParticle& s = st_sParticles[j];
				t.velocity = s.velocity + dt * k3.force * 1/k3.mass;
				t.position = s.position + dt * k3.velocity;
		}

		ComputeHairForces(target, dt);

		//Step 4
		HairMesh::ParticleList accum4 = sStrand.strandParticles;
		for (unsigned int j = 0; j < sParticles.size(); j++) {
				Particle& t = tParticles[j];
				Particle& k4 = accum4[j];

				k4.force = t.force;
				k4.velocity = t.velocity;
		}

		//Step 4 - Stiction
		HairMesh::StictionParticleList st_accum4 = sStrand.stictionParticles;
		for (unsigned int j = 0; j < st_sParticles.size(); j++) {
				if (j == 0) continue;
				StictionParticle& t = st_tParticles[j];
				StictionParticle& k4 = st_accum4[j];

				k4.force = t.force;
				k4.velocity = t.velocity;
		}

		ComputeHairForces(target, dt);

		//Put it all together
		double asixth = 1/6.0;
		double athird = 1/3.0;
		for (unsigned int j = 0; j < sParticles.size(); j++) {
				Particle& p = sParticles[j];
				Particle& k1 = accum1[j];
				Particle& k2 = accum2[j];
				Particle& k3 = accum3[j];
				Particle& k4 = accum4[j];

			if (j >0) {
				p.velocity = p.velocity + dt*(asixth * k1.force +
					athird * k2.force + athird * k3.force + asixth * k4.force)*1/p.mass;

				p.position = p.position + dt*(asixth * k1.velocity +
				athird * k2.velocity + athird * k3.velocity + asixth * k4.velocity);
		
			}
			else { 
				p.velocity = vec3(0,0,0);
			}
		}

		//Put all the stictin together
		for (unsigned int j = 0; j < st_sParticles.size(); j++) {
				StictionParticle& p = st_sParticles[j];
				StictionParticle& k1 = st_accum1[j];
				StictionParticle& k2 = st_accum2[j];
				StictionParticle& k3 = st_accum3[j];
				StictionParticle& k4 = st_accum4[j];

				if (j >0) {
					p.velocity = p.velocity + dt*(asixth * k1.force +
						athird * k2.force + athird * k3.force + asixth * k4.force)*1/p.mass;
	
					p.position = p.position + dt*(asixth * k1.velocity +
					athird * k2.velocity + athird * k3.velocity + asixth * k4.velocity);
		
				} else { 
					p.velocity = vec3(0,0,0);
				}
			}

		}
	}





//---------------------------------------------------------------------
// Spring
//---------------------------------------------------------------------
HairMesh::Spring::Spring() :
    m_type(HairMesh::STRUCTURAL), 
    m_p1(-1), 
    m_p2(-1), 
	m_s1(-1),
	m_s2(-1),
    m_Ks(1.0), m_Kd(1.0), m_restLen(1.0)
{
}

HairMesh::Spring::Spring(const HairMesh::Spring& p) :
m_type(p.m_type),m_s1(p.m_s1), m_p1(p.m_p1), m_s2(p.m_s2),m_p2(p.m_p2),
    m_Ks(p.m_Ks), m_Kd(p.m_Kd), m_restLen(p.m_restLen)
{
}

HairMesh::Spring& HairMesh::Spring::operator=(const HairMesh::Spring& p)
{
    if (&p == this) return *this;

    m_type = p.m_type;
	m_s1 = p.m_s1;
	m_s2 = p.m_s2;
    m_p1 = p.m_p1;
    m_p2 = p.m_p2;
	
	/*m_pt1 = p.m_pt1;
	m_pt2 = p.m_pt2;*/
	
    m_Ks = p.m_Ks;
    m_Kd = p.m_Kd;
    m_restLen = p.m_restLen;
    return *this;
}

HairMesh::Spring::Spring(HairMesh::SpringType t, 
    int s1, int p1, int s2, int p2, double Ks, double Kd, double restLen) :
    m_type(t), m_Ks(Ks), m_Kd(Kd), m_s1(s1),  m_p1(p1), m_s2(s2), m_p2(p2), m_restLen(restLen)
{
}



//---------------------------------------------------------------------
// Particle
//---------------------------------------------------------------------

HairMesh::Particle HairMesh::Particle::EMPTY;

HairMesh::Particle::Particle(int idx, const vec3& p, const vec3& v, double m, bool isTemp)
{
    index = idx;
    position = p;
    velocity = v;
    force = vec3(0,0,0);
    mass = m;
	isTemporary = isTemp;
}
HairMesh::Particle::Particle() : index(-1), position(0,0,0), velocity(0,0,0), force(0,0,0), mass(1.0)
{
	isTemporary = false;
}

HairMesh::Particle::Particle(const HairMesh::Particle& p) : 
index(p.index), position(p.position), velocity(p.velocity), force(p.force), mass(p.mass), isTemporary(p.isTemporary)
{
}

HairMesh::Particle& HairMesh::Particle::operator=(const HairMesh::Particle& p)
{
    if (&p == this) return *this;

    index = p.index;
    position = p.position;
    velocity = p.velocity;
    force = p.force;
    mass = p.mass;
	isTemporary = p.isTemporary;
    return *this;
}

//---------------------------------------------------------------------
// Intersection
//---------------------------------------------------------------------

HairMesh::Intersection::Intersection() : 
	m_p(-1), m_normal(0,0,0), m_distance(0) , m_type(CONTACT) , m_ground_friction(0.0), m_strand(0)
{
}

HairMesh::Intersection::Intersection(const HairMesh::Intersection& p) :
m_p(p.m_p), m_normal(p.m_normal), m_distance(p.m_distance), m_type(p.m_type), m_ground_friction(p.m_ground_friction), m_strand(p.m_strand)
{
}

HairMesh::Intersection& HairMesh::Intersection::operator=(const HairMesh::Intersection& p)
{
    if (&p == this) return *this;
    m_p = p.m_p;
    m_normal = p.m_normal;
    m_distance = p.m_distance;
    m_type = p.m_type;
	m_ground_friction = p.m_ground_friction;
	m_strand = p.m_strand;
    return *this;
}

HairMesh::Intersection::Intersection(IntersectionType type, int p, const vec3& normal, double d) :
	m_p(p), m_normal(normal), m_distance(d), m_type(type), m_ground_friction(0.0)
{
}


//---------------------------------------------------------------------
// Drawing
//---------------------------------------------------------------------

void HairMesh::FaceMesh::Draw(const HairMesh& m)
{
	//Not Used
}

void HairMesh::FaceMesh::DrawNormals(const HairMesh& m)
{
	//Not Used
}

#define R(i) max(0, min(i, m.m_rows)) // CLAMP row index
#define C(j) max(0, min(j, m.m_cols)) // CLAMP col index
#define D(j) max(0, min(j, m.m_stacks)) // CLAMP stack index
HairMesh::FaceMesh::FaceMesh(const HairMesh& m, HairMesh::Face f)
{
	//Not Used
}

void HairMesh::FaceMesh::CalcDistToEye(const HairMesh& m, const vec3& eyePos)
{
    std::vector<int> points = m_strips[(int) (m_strips.size()*0.5)];
    int idx = points[(int) (points.size()*0.5)];
    vec3 pos = m.GetParticle(m.m_vparticles, idx).position;
    distToEye = (pos - eyePos).Length();
}

bool HairMesh::FaceMesh::compare(const FaceMesh& one, const FaceMesh& other)
{
    return one.distToEye > other.distToEye;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////// ALL DAT HURRRRRR ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

//###############################################################
//######################### GhostParticle #######################
//###############################################################
HairMesh::GhostParticle::GhostParticle(int idx, const vec3& p, const vec3& v, double m)
{
    index = idx;
    position = p;
    velocity = v;
    force = vec3(0,0,0);
    mass = m;
}

HairMesh::GhostParticle::GhostParticle()
{
	Particle::Particle();
}

HairMesh::GhostParticle::GhostParticle(const HairMesh::GhostParticle& p) 
{
	GhostParticle::GhostParticle(p.index, p.position, p.velocity, p.mass);
	force = p.force;
}

HairMesh::GhostParticle& HairMesh::GhostParticle::operator=(const HairMesh::GhostParticle& p)
{
    if (&p == this) return *this;

    index = p.index;
    position = p.position;
    velocity = p.velocity;
    force = p.force;
    mass = p.mass;
    return *this;
}


//##################################################################
//######################### StictionParticle #######################
//##################################################################
HairMesh::StictionParticle::StictionParticle(int s1, int p1, const vec3& p, int s2, int p2, const vec3& vel, double m)
{
	Particle::Particle();
    m_s1 = s1;
	m_p1 = p1;
	m_s2 = s2;
	m_p2 = p2;
    
	position = p;
    velocity = vel;
    force = vec3(0,0,0);
    mass = m;
	isTemporary = true;
}

HairMesh::StictionParticle::StictionParticle()
{
	Particle::Particle();
	m_s1 = -1;
	m_p1 = -1;
	m_s2 = -1;
	m_p2 = -1;
	 isTemporary = true;
}

HairMesh::StictionParticle::StictionParticle(const HairMesh::StictionParticle& p) {
	 isTemporary = p.isTemporary;
	 velocity = p.velocity;
	 force = p.force;
	 mass = p.mass;
	 position = p.position;
	 m_s1 = p.m_s1;
	m_p1 = p.m_p1;
	m_s2 = p.m_s2;
	m_p2 = p.m_p2;
	 //Nothing else
}

HairMesh::StictionParticle& HairMesh::StictionParticle::operator=(const HairMesh::StictionParticle& p)
{
    if (&p == this) return *this;
	isTemporary = p.isTemporary;
    index = p.index;
    position = p.position;
    velocity = p.velocity;
    force = p.force;
    mass = p.mass;
	m_s1 = p.m_s1;
	m_p1 = p.m_p1;
	m_s2 = p.m_s2;
	m_p2 = p.m_p2;

    return *this;
}

//###################################################
//################ InitHairMesh #####################
//###################################################
void HairMesh::InitHairMesh()
{

	HAIR_SPRINGS.clear();
	HAIR_CONTACTS.clear();
	HAIR_COLLISIONS.clear();
	HAIR_STICTIONS.clear();
	HAIR_IMPULSES.clear();

	//##########################################################
	//#################### HAIR INITIALIZATION #################
	//##########################################################
	//Init Hairs
	this->StrandList = HairStrandList();
	
	//Make a strand
	//HairStrand h = HairStrand(vec3(0, 1, 0));
	//Add to our strandList
	//StrandList.addStrand(h);

	int numStrands = 8;
	double angleOffset = 360.0 / numStrands;
	double hOffset = -0.15;
	double vOffset = 0.00;
	// Create a strand for each angle and add to StrandList
	for (int i = 0; i < numStrands; i++) {
		vec3 rootPosition(vOffset, 1.4, hOffset);
		HairStrand h = HairStrand(rootPosition, 0);
		StrandList.addStrand(h);
		hOffset+= 0.05;
		vOffset += 0.02;
	}

	//For each hair strand make the springs
	for (unsigned int sNum = 0; sNum < StrandList.size(); sNum++) {
		HairStrand strand = StrandList.getStrand(sNum);
		
		//Get Particles for a strand 
		HairMesh::ParticleList hairParticles = strand.strandParticles;
		
		///////////////////////
		// Add Edge Springs ///
		///////////////////////
		for (unsigned int pNum = 0; pNum < hairParticles.size()-2; pNum+=2 ) {
			
			//Current Particle index in a strand (Normal)
			int p0 = pNum;
			
			//Next  particle index in the strand (Ghost)
			int pG = pNum+1;
			
			//Second Next index Particle in the strand (Normal)
			int p1 = pNum+2;

			//Add the Edge Springs
			AddEdgeSpring(sNum, p0, sNum, pG);   //first to ghost
			AddEdgeSpring(sNum, pG, sNum, p1);   //ghost to second
			AddEdgeSpring(sNum, p0, sNum, p1);   //first to second
		}
		//////////////////////////
		// Add Torsion Springs ///
		//////////////////////////
		for (unsigned int pNum = 0; pNum < hairParticles.size()-3; pNum++ ) {
			
			//Current Particle index in a strand (Normal)
			int p0 = pNum;
			
			//Next  particle index in the strand (Ghost)
			int p1 = pNum+3;
			
			//Add the Torsion Springs
			AddTorsionSpring(sNum, p0, sNum, p1); //first to ghost
		}
		///////////////////////
		// Add Bend Springs ///
		///////////////////////
		for (unsigned int pNum = 0; pNum < hairParticles.size()-3; pNum++ ) {
			//Normal Particles
			if (pNum % 2 == 0) {
				//Current Particle index in a strand (Normal)
				int p0 = pNum;
				//Next  particle index in the strand (Ghost)
				int p1 = pNum+4;
				//Add the Bend Springs
				AddBendSpring(sNum, p0, sNum, p1);   //first to second
			}
			else if (pNum % 2 == 1) {
				//Current Particle index in a strand (Normal)
				int p0 = pNum;
				//Next  particle index in the strand (Ghost)
				int p1 = pNum+2;
				//Add the Bend Springs
				AddBendSpring(sNum, p0, sNum, p1);   //first to second
			}
		}
	}

}

/////////////////////////////
//// ADD HAIR SPRING ////////
/////////////////////////////

HairMesh::Particle& HairMesh::GetParticleInStrand(int sNum, int pNum) {
	return (StrandList.getStrand(sNum)).strandParticles[pNum];
}

HairMesh::StictionParticle& HairMesh::GetStictionParticleInStrand(int sNum, int pNum) {
	return (StrandList.getStrand(sNum)).stictionParticles[pNum];
}



void HairMesh::AddTorsionSpring(int s1, int p1, int s2, int p2)
{	//Get particles for strand for both sets of numbers
	Particle& pt1 = GetParticleInStrand(s1, p1);
	Particle& pt2 = GetParticleInStrand(s2, p2);
	double restLen = (pt1.position - pt2.position).Length();
    HAIR_SPRINGS.push_back(Spring(TORSION, s1, p1, s2, p2, g_torsionKs, g_torsionKd, restLen));
}

void HairMesh::AddEdgeSpring(int s1, int p1, int s2, int p2)
{
   Particle& pt1 = GetParticleInStrand(s1, p1);
   Particle& pt2 = GetParticleInStrand(s2, p2);
   double restLen = (pt1.position - pt2.position).Length();
   HAIR_SPRINGS.push_back(Spring(EDGE, s1, p1, s2, p2, g_edgeKs, g_edgeKd, restLen));
}

void HairMesh::AddBendSpring(int s1, int p1, int s2, int p2)
{
	Particle& pt1 = GetParticleInStrand(s1, p1);
	Particle& pt2 = GetParticleInStrand(s2, p2);
	double restLen = (pt1.position - pt2.position).Length();
    HAIR_SPRINGS.push_back(Spring(BEND, s1, p1, s2, p2, g_bendKs, g_bendKd, restLen));
}

void HairMesh::AddStictionSpring(int s1, int p1Index, int s2, int p2Index)
{
	//GET STI
	StictionParticle& pt1 = GetStictionParticleInStrand(s1, p1Index);
	StictionParticle& pt2 = GetStictionParticleInStrand(s2, p2Index);
	double restLen = (pt1.position - pt2.position).Length();
    HAIR_SPRINGS.push_back(Spring(STICTION, s1, p1Index, s2, p2Index, g_stictionKs, g_stictionKd, restLen));
}
    

//##########################################################
//######################### DRAW HAIR ######################
//##########################################################
void HairMesh::DrawHair() {
	//return;	
	//if (!SHOULD_DRAW_HAIR) return;
	if (SHOULD_DRAW_HAIR) {
		glDisable(GL_LIGHTING);

		glBegin(GL_LINES);
		glColor3f(0.0, 0.0, 0.0);

		HairStrandList& strandList = this->StrandList;
		for (unsigned int i = 0; i < strandList.size(); i++) {
			//Get current Strand
			const HairStrand strand = strandList.getStrand(i);

			//Get Particles for a strand
			const HairMesh::ParticleList hairParticles = strand.strandParticles;
			for (unsigned int k = 0; k <  (hairParticles.size()/2); k++) {
			
				//Current Particle in a strand
				const Particle p0 = hairParticles[k*2];
				//Next particle in the strand
				const Particle p1 = hairParticles[k*2+2];
	
				//Create vertices for each particle to draw a line between them
				glVertex3f(p0.position[0], p0.position[1], p0.position[2]);
				glVertex3f(p1.position[0], p1.position[1], p1.position[2]);
			}
		}

		glEnd();
		glEnable(GL_LIGHTING);
	}


}

void HairMesh::DrawHairSprings() {
	glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
	glClearColor(0,0,0,0);
	glColor3f(0.0, 1.0, 0.0);
    for (unsigned int i = 0; i < HAIR_SPRINGS.size(); i++)
    {	
		Spring currSpring = HAIR_SPRINGS[i];
        if (!(currSpring.m_type & m_drawflags)) continue;

        switch (currSpring.m_type)
        { 
        case BEND:       glColor3f(1.0, 0.4, 0.7); break;
		case EDGE:		 glColor3f(0.0, 1.0, 1.0); break;
		case TORSION:	 glColor3f(0.0, 1.0, 0.4); break;
		case STICTION: glColor3f(0.3, 0.5, 0.1); break;
        };

		int sIndex1 = currSpring.m_s1;
		int ptIndex1 = currSpring.m_p1;
		int sIndex2 = currSpring.m_s2;
		int ptIndex2 = currSpring.m_p2;

		if (sIndex1 < 0 || ptIndex1 < 0 || sIndex2 < 0 || ptIndex2 < 0) return;

		vec3 p1;
		vec3 p2;
		if (currSpring.m_type == STICTION) {
			//Stiction Case
			p1 =(GetStictionParticleInStrand(sIndex1, ptIndex1)).position;
			p2 = (GetStictionParticleInStrand(sIndex2, ptIndex2)).position;

		} else {
			// all other springs
			p1 = (GetParticleInStrand(sIndex1, ptIndex1)).position;
			p2 = (GetParticleInStrand(sIndex2, ptIndex2)).position;
		}
		//TODO: DO CORRENT DRAWING / ORGANIZE on EB
		glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();
	glEnable(GL_LIGHTING);

}


void HairMesh::DrawHairParticles() {
	
	glDisable(GL_LIGHTING);

	glPointSize(6.0);
	glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
   

    HairStrandList& strandList = this->StrandList;
    for (unsigned int i = 0; i < strandList.size(); i++) {
		//Get current Strand
		const HairStrand strand = strandList.getStrand(i);

		//Get Particles for a strand
		const HairMesh::ParticleList hairParticles = strand.strandParticles;
		for (unsigned int k = 0; k <  hairParticles.size(); k++) {
			if (!SHOULD_DRAW_HAIR_PARTICLES && k%2 == 0) continue;
			if (!SHOULD_DRAW_GHOST_PARTICLES && k%2 == 1) continue;
			//Current Particle in a strand
			const Particle p0 = hairParticles[k];

			// GHOST PARTICLES ARE ODD INDICES, HAIR PARTICLES ARE EVEN
			if (k % 2 == 0) glColor4f(0.0, 1.0, 0.0, 1.0);
			else glColor4f(1.0, 0.0, 0.0, 1.0);
			//Create vertices for each particle to draw a line between them
			glVertex3f(p0.position[0], p0.position[1], p0.position[2]);
		}

		//Draw Stiction PArticles
		//Get Particles for a strand
		const HairMesh::StictionParticleList stictionParticles = strand.stictionParticles;
		if (!SHOULD_DRAW_STICTION_PARTICLES) continue;
		for (unsigned int k = 0; k <  stictionParticles.size(); k++) {
			//Current Particle in a strand
			const StictionParticle p0 = GetStictionParticleInStrand(i,k);// stictionParticles[k];

			// GHOST PARTICLES ARE ODD INDICES, HAIR PARTICLES ARE EVEN
			glColor4f(0.0, 1.0, 0.6, 1.0);
			//Create vertices for each particle to draw a line between them
			glVertex3f(p0.position[0], p0.position[1], p0.position[2]);
		}

    }





    glEnd();
    glEnable(GL_LIGHTING);


}
//--------------------------------------------------------------------
//------------------------Time Integration for Hair ------------------
//--------------------------------------------------------------------
void HairMesh::applyStrainLimiting(double dt) {

	// we want the 3rd edge spring each time, where endpoints are real hair particles
	unsigned int edgeSpringCount = 0;
	// for each spring in the list of springs
	for (unsigned int i = 0; i < HAIR_SPRINGS.size(); i++) {
		// get a spring from this list
		Spring& spring = HAIR_SPRINGS[i];
		// skip if not edge. if edge and not the third 
		if (spring.m_type != EDGE) continue;
		//Type is edge so increment
		edgeSpringCount++;
		if (edgeSpringCount < 3) continue;
		else edgeSpringCount = 0;

		// for each good edge spring, get the start and end particles
		Particle& rootParticle = GetParticleInStrand(spring.m_s1,0);
		Particle& startParticle = GetParticleInStrand(spring.m_s1,spring.m_p1);
		Particle& endParticle = GetParticleInStrand(spring.m_s2,spring.m_p2);

		double dist1 = (rootParticle.position - startParticle.position).Length();
		double dist2 = (rootParticle.position - endParticle.position).Length();

		vec3 direction = vec3(0.0,0.0,0.0);
		// if start is farther than end
		if (dist1 > dist2) direction = endParticle.position - startParticle.position;
		else direction = startParticle.position - endParticle.position;
		double currentLength = direction.Length();

		//normalize the direction
		direction = direction.Normalize();

		// if the spring exceeds 10% deformity
		if (currentLength > spring.m_restLen * 1.10) {
			double distToMoveUp = fabs(currentLength - spring.m_restLen);
			//cout << "dist1 and dist2: " << dist1 << " " << dist2 << endl;
			if (dist1 > dist2)
				startParticle.position = startParticle.position + (direction * distToMoveUp);
			
			else
				endParticle.position = endParticle.position + (direction * distToMoveUp);
		}
	}


}

void HairMesh::applySelfRepulsions(double dt) {
	//Do Nothing
}

void HairMesh::updatePosition(double dt) {
	//DoNothing
}

void HairMesh::resolveBodyCollisions(double dt) {
	//Do Nothing
}

void HairMesh::resolveSelfCollisions(double dt) {
	//Do Nothing
}
void HairMesh::updateVelocity(double dt) {
}

void HairMesh::extrapolateVelocity(double dt) { 
}

//Add Gravity and that good stuff to the particles
void HairMesh::ComputeHairForces(HairStrandList& strands, double dt) {
	//Gravity!
	for (unsigned int i = 0; i < strands.size(); i++) {
		//Current Strand
		HairStrand& strand = strands.getStrand(i);
		//Strand Particles
		ParticleList& particles = strand.strandParticles;
		//Loop through and apple forces
		for (unsigned int j = 0; j < particles.size(); j++) {
			Particle& p = particles[j];
			p.force = m_externalForces * p.mass; // TODO what actually belongs from the paper
			//cout << p.force << endl;
		}
	}
	
    // Update springs - Non-stiction
    for(unsigned int i = 0; i < HAIR_SPRINGS.size(); i++)
    {
        Spring& spring = HAIR_SPRINGS[i];
		/*Particle& a = Particle();
		Particle& b = Particle(); */

		// Stiction Spring
		if (spring.m_type == STICTION) {
			Particle& a = strands.getStictionParticleInStrand(spring.m_s1, spring.m_p1);
			Particle& b = strands.getStictionParticleInStrand(spring.m_s2, spring.m_p2);
			//Spring Variables
			double Ks = spring.m_Ks;				 //Hook's Spring Constant
			double Kd = spring.m_Kd;				 //Damping Constant
			double R = spring.m_restLen;			 //Rest Length of Spring
			vec3 L = a.position - b.position;			 //Vector from B to A (position)
			vec3 normL = L/L.Length();				 //Normalized L vector
			vec3 diffVelocity = a.velocity - b.velocity;	 //Vector Velocities
			double length = L.Length()-R;
			//Stiffness Spring Force:	-Ks(|L|-R)*L/(|L|);
			//Damping Force:			-kd(((v_a-v_b)*L/|L|)* L/|L|);

			vec3 Fs = -1*Ks*(length)*L.Normalize();
			vec3 Fd = -1*Kd*((L*diffVelocity)/L.Length())*L.Normalize();

			//One particle is -F the other is F
			a.force += Fs + Fd;
			b.force += -1*(Fs+Fd);
		
		}
		//All other spring types
		else {
			Particle& a = strands.getParticleInStrand(spring.m_s1, spring.m_p1);
			Particle& b = strands.getParticleInStrand(spring.m_s2, spring.m_p2);
			//Spring Variables
			double Ks = spring.m_Ks;				 //Hook's Spring Constant
			double Kd = spring.m_Kd;				 //Damping Constant
			double R = spring.m_restLen;			 //Rest Length of Spring
			vec3 L = a.position - b.position;			 //Vector from B to A (position)
			vec3 normL = L/L.Length();				 //Normalized L vector
			vec3 diffVelocity = a.velocity - b.velocity;	 //Vector Velocities
			double length = L.Length()-R;
			//Stiffness Spring Force:	-Ks(|L|-R)*L/(|L|);
			//Damping Force:			-kd(((v_a-v_b)*L/|L|)* L/|L|);

			vec3 Fs = -1*Ks*(length)*L.Normalize();
			vec3 Fd = -1*Kd*((L*diffVelocity)/L.Length())*L.Normalize();

			//One particle is -F the other is F
			a.force += Fs + Fd;
			b.force += -1*(Fs+Fd);
		
		}

		
    }

	//TODO: COMPUTE FORCES FOR STICTION PARTICLES
}


void HairMesh::CheckParticleCollisions(const World& world) {
    HAIR_CONTACTS.clear();
    HAIR_COLLISIONS.clear();
	
	int totalNumStrands = StrandList.size();
	//int totalNumParticles = 0;
	for (unsigned int i = 0; i < totalNumStrands; i++) {
		HairStrand& strand = StrandList.getStrand(i);
		int numParticlesInStrand = strand.strandParticles.size();
		int strandIndex = i;
		//cout << "strand: " << strandIndex << endl;

		for (unsigned int j = 0; j < numParticlesInStrand; j++) {
			Particle p = GetParticleInStrand(i,j);
			int particleIndex = j;
			if (particleIndex % 2 == 1) continue; // skipping ghost particles

			// 1. Check collisions with world objects 
			for (unsigned int k = 0; k < world.m_shapes.size(); k++) {
				
				Intersection intersection = Intersection();
				//cout << "i before passing into sphere intersection" << i << endl;
				if (world.m_shapes[k]->GetType() == World::SPHERE &&
					SphereIntersection(p, i, j, (World::Sphere*) world.m_shapes[k], intersection))
				{
					//cout << "SPHEREEEE" << endl;
					if (intersection.m_type == IntersectionType::CONTACT) {
						HAIR_CONTACTS.push_back(intersection);
						//cout<<intersection.m_strand<<endl;
					} else if (intersection.m_type == IntersectionType::COLLISION) {
						HAIR_COLLISIONS.push_back(intersection);
					}
				}
				else if (world.m_shapes[k]->GetType() == World::GROUND && 
					FloorIntersection(p, i, j, intersection))
                {
					//cout << "FLOOOOOOR" << endl;
					if (intersection.m_type == IntersectionType::CONTACT) {
						HAIR_CONTACTS.push_back(intersection);
					} else if (intersection.m_type == IntersectionType::COLLISION) {
						HAIR_COLLISIONS.push_back(intersection);
					}
                }
			} // ----------------END FOR k-----------------------------------
		} // ----------------END FOR j-----------------------------------
	} // ----------------END FOR i-----------------------------------

}

void HairMesh::CheckStrandCollisions() {
	HAIR_STICTIONS.clear();
	HAIR_IMPULSES.clear();

	double collisionEpsilon = 0.002; //0.06
	double contactEpsilon = 0.0000;

	int totalNumStrands = StrandList.size();
	// i loops through all strands but the last, b/c we will compare second to last with last in l loop
	for (unsigned int i = 0; i < totalNumStrands - 1; i++) {
		HairStrand& strand1 = StrandList.getStrand(i);
		int numParticlesInStrand1 = strand1.strandParticles.size();
		int strandIndex1 = i;
		//cout << "strand: " << strandIndex1 << endl;

		
		for (unsigned int j = 0; j < numParticlesInStrand1; j++) {
			// skip the last particle and ghost particles
			if (j == numParticlesInStrand1 - 1 || j % 2 == 1) continue;
			Particle p1 = GetParticleInStrand(strandIndex1,j);
			Particle p2 = GetParticleInStrand(strandIndex1,j+2);
			vec3 segment1 = p2.position - p1.position;

			// if we've checked i against k already, we don't wanna do k against i again
			for (unsigned int k = i+1; k < totalNumStrands; k++) {
				int strandIndex2 = k;
			
				HairStrand& strand2 = StrandList.getStrand(k);
				int numParticlesInStrand2 = strand2.strandParticles.size();
				for (unsigned int l = 0; l < numParticlesInStrand2; l++) {
					// skip the last particle and ghost particles
					if (l == numParticlesInStrand2 - 1 || l % 2 == 1) continue;
					Particle p3 = GetParticleInStrand(strandIndex2,l);
					Particle p4 = GetParticleInStrand(strandIndex2,l+2);
					vec3 segment2 = p4.position - p3.position;

					// Ax = b
					double A11 = segment1 * segment1;
					double A12 = - segment1 * segment2;
					double A21 = - segment1 * segment2;
					double A22 = segment2 * segment2;

					double b1 = segment1 * (p3.position - p1.position);
					double b2 = - segment2 * (p3.position - p1.position);

					double x1 = (b1 - A12 * b2 / A22) / (A11 - A12 * A21 / A22);
					double x2 = (b2 - A11 * x1) / A12;

					// closest point
					vec3 cp1 = p1.position + (x1 * segment1);
					vec3 cp2 = p3.position + (x2 * segment2);

					double dist = (cp1 - cp2).Length();
					//cout << "dist: " << dist << endl;

					if (dist < collisionEpsilon && dist > contactEpsilon) {
						// apply dat impulse (adjusted impulse)
						Impulse impulse;
						impulse.strandIndex1 = strandIndex1;
						impulse.segmentStartIndex1 = j;
						impulse.strandIndex2 = strandIndex2;
						impulse.segmentStartIndex2 = l;
						impulse.p1 = cp1;
						impulse.p2 = cp2;
						impulse.a = x1;
						impulse.b = x2;
						// calculate adjusted impulse; dist is my impulse magnitude?
						dist = dist / 2.0;
						double adjustedImpulse = (2 * dist) / (x1*x1 + (1-x1*x1) + x2*x2 + (1-x2*x2));
						impulse.m_distance = adjustedImpulse;
						HAIR_IMPULSES.push_back(impulse);

					} else if (dist <= contactEpsilon) {
						Stiction stiction;
						stiction.strandIndex1 = strandIndex1;
						stiction.segmentStartIndex1 = j;
						stiction.strandIndex2 = strandIndex2;
						stiction.segmentStartIndex2 = l;
						stiction.p1 = cp1;
						stiction.p2 = cp2;
						HAIR_STICTIONS.push_back(stiction);
					}

				} // ----------------END FOR l-----------------------------------
			} // ----------------END FOR k-----------------------------------
		} // ----------------END FOR j-----------------------------------
	} // ----------------END FOR i-----------------------------------
	//cout << "Done" << endl;
	/* for (int i = 0; i < HAIR_STICTIONS.size(); i++) {
		cout << "closest points: " << HAIR_STICTIONS[i].p1 << endl;
	}*/
}

void HairMesh::ResolveHairContacts() {

	//CONTACTS - Penetration of the other object
	for (unsigned int i = 0; i < HAIR_CONTACTS.size(); i++) {		
		//Con
		const Intersection& result = HAIR_CONTACTS[i];
		//cout << "when passed into resolveHairContacts: " << result.m_strand << endl;
		Particle& p = GetParticleInStrand(result.m_strand, result.m_p);
		double D = result.m_distance;	
        vec3 N = result.m_normal;
		vec3 unitN = N.Normalize();

		//Apply impulse to velocity (directly reflect it across the normal) IF out velocity is moving downwards
		vec3 V = p.velocity;
		double VNorm = p.velocity*unitN;
		//vec3 VTan = V-VNorm*unitN;
		//Check if velocity ismoving downwards
		//if(VNorm < 0.0) {
			//Now reflect the velocity with a restituion (damping on velocity)
			double kR = 0.5;
			p.velocity = kR*(p.velocity-(2*unitN*(unitN*V)));
			//p.velocity *= -1;
			//Move point to the surface of the object it is currently in
			vec3 distanceAdjust = D*unitN;
			p.position = p.position+distanceAdjust;
			
			double kFriction = result.m_ground_friction;
			if (kFriction > 0.0) {
				//Kinectic Friction Force
				//cout<<"Kinectic Friction";
				p.force -= D*N/friction_Mk;
			}
			p.velocity = vec3(0.0,0.0,0.0); //velocity of particle = 0
			//p.force = vec3(0.0,0.0,0.0);
		//}

    }
}

void HairMesh::ResolveHairCollisions() {
    //COLLISION - Close to Hitting other object
	for(unsigned int i = 0; i < HAIR_COLLISIONS.size(); i++) {
		//Collision Details
        const Intersection result = HAIR_COLLISIONS[i];
        Particle& p = GetParticleInStrand(result.m_strand, result.m_p);
		double D = result.m_distance;	
        vec3 N = result.m_normal;
		vec3 unitN = N.Normalize();
		
		//Get Particle's Velocity in direction of surface Normal Direction
		vec3 V = p.velocity;
		double VProj = V*unitN;						
		vec3 VNorm = (V*unitN)*unitN;				
		
		// If Velocity is moving into the collision threshold (towards the surface), apply the force
		if (VProj <= 0.0) {
			//Apply force that is relative to the distance  from the collision threshold
			vec3 Fpenalty = -1*((g_penaltyKs*(D-0) + g_penaltyKd*(V*(unitN)))*unitN);
			p.force += Fpenalty;

			double kFriction = result.m_ground_friction;
			if (kFriction > 0.0) {
				//Kinectic Friction Force
				//cout<<"Kinectic Friction";
				p.force -= D*N/friction_Mk;
			}
		}
		p.velocity = vec3(0.0,0.0,0.0); //velocity of particle = 0
		//p.force = vec3(0.0,0.0,0.0);
	}
}

void HairMesh::applyStiction() {
	for (unsigned int i = 0; i < HAIR_STICTIONS.size(); i++) {
		//Current stiction
		Stiction stiction = HAIR_STICTIONS[i];

		//The first strand particle list
		HairStrand& s1 = StrandList.getStrand(stiction.strandIndex1);
		StictionParticleList& pList1 = s1.stictionParticles;
	
		//First Closest Particle - add stiction point
		StictionParticle p1 = StictionParticle();
		p1.m_s1 = stiction.strandIndex1;
		p1.m_p1 = stiction.segmentStartIndex1;
		p1.m_s2 = stiction.strandIndex2;
		p1.m_p2 = stiction.segmentStartIndex2;
		p1.position = stiction.p1;
		p1.isTemporary = true;
		p1.index = pList1.size();
		
		//The Second strand and particle list
		HairStrand& s2 = StrandList.getStrand(stiction.strandIndex2);
		StictionParticleList& pList2 = s2.stictionParticles;

		//Second Closest Particles - add stiction point
		StictionParticle p2 = StictionParticle();
		p2.m_s1 = stiction.strandIndex2;
		p2.m_p1 = stiction.segmentStartIndex2;
		p2.m_s2 = stiction.strandIndex1;
		p2.m_p2 = stiction.segmentStartIndex1;
		p2.position = stiction.p2;
		p2.isTemporary = true;
		p2.index = pList2.size();
		
		//Prevents Explosion of stiction particles
		if (p2.index > StrandList.size() || p1.index > StrandList.size()){
			return;
		}

		//Insert the first closest particle
		pList1.push_back(p1);
		//Insert the second closest particle
		pList2.push_back(p2);


		//Now add a Stiction Spring
		//AddStictionSpring(stiction.strandIndex1, p1.index, stiction.strandIndex2, p2.index);

	}
}

void HairMesh::applyImpulse() {

	for (unsigned int i = 0; i < HAIR_IMPULSES.size(); i++) {
		//Current stiction
		Impulse impulse = HAIR_IMPULSES[i];

		//Get the 2 particles from first segment
		Particle& p1 = GetParticleInStrand(impulse.strandIndex1,impulse.segmentStartIndex1);
		Particle& p2 = GetParticleInStrand(impulse.strandIndex1,impulse.segmentStartIndex1+1);

		//Get the 2 particles from second segment
		Particle& p3 = GetParticleInStrand(impulse.strandIndex2,impulse.segmentStartIndex2);
		Particle& p4 = GetParticleInStrand(impulse.strandIndex2,impulse.segmentStartIndex2+1);

		vec3 n = impulse.p2 - impulse.p1;
		p1.velocity = p1.velocity + (1 - impulse.a) * (impulse.m_distance / p1.mass) * n;
		p2.velocity = p2.velocity + impulse.a * (impulse.m_distance / p2.mass) * n;
		p3.velocity = p3.velocity - (1 - impulse.b) * (impulse.m_distance / p3.mass) * n;
		p4.velocity = p4.velocity - impulse.b * (impulse.m_distance / p4.mass) * n;
		
    }
}

//---------------------------------------------------------------------
//-------------------------- HairStrandList ---------------------------
//---------------------------------------------------------------------

//#######################################################
//############# HairStrandList Constructor  #############
//#######################################################
HairMesh::HairStrandList::HairStrandList() {
	strands = std::vector<HairStrand>();
}


//############################################################
//############# HairStrandList Copy Constructor  #############
//############################################################
HairMesh::HairStrandList::HairStrandList(const HairStrandList& h) {
	strands = h.strands;
}

//#######################################################
//####### HairStrandList Assignment Constructor #########
//#######################################################
HairMesh::HairStrandList& HairMesh::HairStrandList::operator=(const HairStrandList& h) {
    if (&h == this) return *this;

    strands = h.strands;
    return *this;
}

int HairMesh::HairStrandList::size() {
	return strands.size();
}

HairMesh::HairStrand& HairMesh::HairStrandList::getStrand(unsigned int index) {
	return strands[index];
}

void HairMesh::HairStrandList::addStrand(HairStrand h) {
	strands.push_back(h);
}

HairMesh::Particle& HairMesh::HairStrandList::getParticleInStrand(int sNum, int pNum) {
	HairStrand& strand = getStrand(sNum);
	Particle& p = strand.strandParticles[pNum];
	return p;
}

HairMesh::StictionParticle& HairMesh::HairStrandList::getStictionParticleInStrand(int sNum, int pNum) {
	HairStrand& strand = getStrand(sNum);
	StictionParticle& p = strand.stictionParticles[pNum];
	return p;
}
//---------------------------------------------------------------------
//---------------------------- HairStrand -----------------------------
//---------------------------------------------------------------------
HairMesh::HairStrand HairMesh::HairStrand::EMPTY;


//#######################################################
//############# HairStrand Constructor #1 ###############
//#######################################################
HairMesh::HairStrand::HairStrand(const vec3& p)
{
	position = p;
	strandParticles = ParticleList();
	InitStrand();
}


//#######################################################
//############# HairStrand Constructor #2 ###############
//#######################################################
HairMesh::HairStrand::HairStrand() : index(-1), position(0,0,0)
{
	strandParticles = ParticleList();
	stictionParticles = StictionParticleList();
	InitStrand();
}

//#######################################################
//############# HairStrand Constructor #3 ###############
//#######################################################
HairMesh::HairStrand::HairStrand(const vec3& p, double angle)
{
	position = p;
	strandParticles = ParticleList();
	stictionParticles = StictionParticleList();
	InitStrand(angle);
}

//#######################################################
//########## HairStrand Equality Constructor ############
//#######################################################
HairMesh::HairStrand::HairStrand(const HairMesh::HairStrand& h) : 
    index(h.index), position(h.position)
{
	strandParticles = h.strandParticles;
	stictionParticles = h.stictionParticles;
}

//#######################################################
//############## HairStrand Copy Constructor ############
//#######################################################
HairMesh::HairStrand& HairMesh::HairStrand::operator=(const HairMesh::HairStrand& h)
{
    if (&h == this) return *this;

    index = h.index;
    position = h.position;
	strandParticles = h.strandParticles;
	stictionParticles = h.stictionParticles;
	
    return *this;
}


//#######################################################
//############### Initialize HairStrand #################
//#######################################################
void HairMesh::HairStrand::InitStrand()
{
    strandSprings.clear();
    /*
    if (m_width < 0.01 || m_height < 0.01 || m_depth < 0.01) return;
    if (m_cols < 1 || m_rows < 1 || m_stacks < 1) return;
	*/
	strandParticles = ParticleList();
    // Init particles
	float strandLength = 2.0f;
	int numHairParticles = 8;
	int numGhostParticles = numHairParticles - 1;
	int numTotalParticles = numHairParticles + numGhostParticles;
	float particleDistOffset = strandLength / numTotalParticles;

	strandParticles.resize(numTotalParticles);
	
	// start with 1 single hair particle
	float x = position[0]; // root position instance variable
	float y = position[1]; // starting height of strand for now
	float z = position[2];
	strandParticles[0] = Particle(0, vec3(x,y,z));

	// GHOST PARTICLES HAVE ODD INDICES, HAIR PARTICLES HAVE EVEN
	for (unsigned int i = 1; i < numTotalParticles; i++) {
		if (i % 2 == 1) {
			float xG = position[0] - i * particleDistOffset;
			float yG = position[1] + 0.3;
			float zG = position[2];
			strandParticles[i] = GhostParticle(i, vec3(xG,yG,zG));
		} else {
			float xH = position[0] - i * particleDistOffset;
			float yH = position[1];
			float zH = position[2];
			strandParticles[i] = Particle(i, vec3(xH,yH,zH));
		}

	}
}

void HairMesh::HairStrand::InitStrand(double angle)
{
    strandSprings.clear();
    /*
    if (m_width < 0.01 || m_height < 0.01 || m_depth < 0.01) return;
    if (m_cols < 1 || m_rows < 1 || m_stacks < 1) return;
	*/
	strandParticles = ParticleList();
    // Init particles
	float strandLength = 1.5f;
	int numHairParticles = 18;
	int numGhostParticles = numHairParticles - 1;
	int numTotalParticles = numHairParticles + numGhostParticles;
	float particleDistOffset = strandLength / numTotalParticles;

	strandParticles.resize(numTotalParticles);
	
	// start with 1 single hair particle
	float x = position[0]; // root position instance variable
	float y = position[1]; // starting height of strand for now
	float z = position[2];
	strandParticles[0] = Particle(0, position);

	// GHOST PARTICLES HAVE ODD INDICES, HAIR PARTICLES HAVE EVEN
	for (int i = 1; i < numTotalParticles; i++) {
		if (i % 2 == 1) {
			float xG = position[0] - (i * particleDistOffset) * cos(2 * MATH_PI * angle / 360.0);
			float yG = position[1] + 0.1;
			float zG = position[2] - (i * particleDistOffset) * sin(2 * MATH_PI * angle / 360.0);
			strandParticles[i] = GhostParticle(i, vec3(xG,yG,zG));
		} else {
			float xH = position[0] - (i * particleDistOffset) * cos(2 * MATH_PI * angle / 360.0);
			float yH = position[1];
			float zH = position[2] - (i * particleDistOffset) * sin(2 * MATH_PI * angle / 360.0);
			strandParticles[i] = Particle(i, vec3(xH,yH,zH));
		}

	}

}

//---------------------------------------------------------------------
//---------------------------- Stiction -------------------------------
//---------------------------------------------------------------------

HairMesh::Stiction::Stiction() {
	m_p = -1;
	m_normal = vec3(0,0,0);
	m_distance = 0;
	m_type = CONTACT;
	m_ground_friction = 0.0;
	m_strand = -1;
	strandIndex1 = -1;
	segmentStartIndex1 = -1;
	strandIndex2 = -1;
	segmentStartIndex2 = -1;
	p1 = vec3(0,0,0);
	p2 = vec3(0,0,0);
}

HairMesh::Stiction::Stiction(const HairMesh::Stiction& p) {
	m_p = p.m_p;
	m_normal = p.m_normal;
	m_distance = p.m_distance;
	m_type = p.m_type;
	m_ground_friction = p.m_ground_friction;
	m_strand = -1;
	strandIndex1 = p.strandIndex1;
	segmentStartIndex1 = p.segmentStartIndex1;
	strandIndex2 = p.strandIndex2;
	segmentStartIndex2 = p.segmentStartIndex2;
	p1 = p.p1;
	p2 = p.p2;
}

HairMesh::Stiction& HairMesh::Stiction::operator=(const HairMesh::Stiction& p) {
    if (&p == this) return *this;
    m_p = p.m_p;
    m_normal = p.m_normal;
    m_distance = p.m_distance;
    m_type = p.m_type;
	m_ground_friction = p.m_ground_friction;
	m_strand = -1;
	strandIndex1 = p.strandIndex1;
	segmentStartIndex1 = p.segmentStartIndex1;
	strandIndex2 = p.strandIndex2;
	segmentStartIndex2 = p.segmentStartIndex2;
	p1 = p.p1;
	p2 = p.p2;
    return *this;
}

HairMesh::Stiction::Stiction(IntersectionType type, int p, const vec3& normal, double d,
	int s1, int start1, int s2, int start2, vec3 cp1, vec3 cp2) {
	Intersection::Intersection();
	m_p = p;
    m_normal = normal;
    m_distance = d;
    m_type = type;
	m_ground_friction = 0.0;
	m_strand = -1;
	strandIndex1 = s1;
	segmentStartIndex1 = start1;
	strandIndex2 = s2;
	segmentStartIndex2 = start2;
	p1 = cp1;
	p2 = cp2;
}

//---------------------------------------------------------------------
//---------------------------- Impulse --------------------------------
//---------------------------------------------------------------------

HairMesh::Impulse::Impulse() {
	m_p = -1;
	m_normal = vec3(0,0,0);
	m_distance = 0;
	m_type = CONTACT;
	m_ground_friction = 0.0;
	m_strand = -1;
	strandIndex1 = -1;
	segmentStartIndex1 = -1;
	strandIndex2 = -1;
	segmentStartIndex2 = -1;
	p1 = vec3(0,0,0);
	p2 = vec3(0,0,0);
	a = -1;
	b = -1;
}

HairMesh::Impulse::Impulse(const HairMesh::Impulse& p) {
	m_p = p.m_p;
	m_normal = p.m_normal;
	m_distance = p.m_distance;
	m_type = p.m_type;
	m_ground_friction = p.m_ground_friction;
	m_strand = -1;
	strandIndex1 = p.strandIndex1;
	segmentStartIndex1 = p.segmentStartIndex1;
	strandIndex2 = p.strandIndex2;
	segmentStartIndex2 = p.segmentStartIndex2;
	p1 = p.p1;
	p2 = p.p2;
	a = p.a;
	b = p.b;
}

HairMesh::Impulse& HairMesh::Impulse::operator=(const HairMesh::Impulse& p) {
    if (&p == this) return *this;
    m_p = p.m_p;
    m_normal = p.m_normal;
    m_distance = p.m_distance;
    m_type = p.m_type;
	m_ground_friction = p.m_ground_friction;
	m_strand = -1;
	strandIndex1 = p.strandIndex1;
	segmentStartIndex1 = p.segmentStartIndex1;
	strandIndex2 = p.strandIndex2;
	segmentStartIndex2 = p.segmentStartIndex2;
	p1 = p.p1;
	p2 = p.p2;
	a = p.a;
	b = p.b;
    return *this;
}

HairMesh::Impulse::Impulse(IntersectionType type, int p, const vec3& normal, double d,
	int s1, int start1, int s2, int start2, vec3 cp1, vec3 cp2, double aVal, double bVal) {
	Intersection::Intersection();
	m_p = p;
    m_normal = normal;
    m_distance = d;
    m_type = type;
	m_ground_friction = 0.0;
	m_strand = -1;
	strandIndex1 = s1;
	segmentStartIndex1 = start1;
	strandIndex2 = s2;
	segmentStartIndex2 = start2;
	p1 = cp1;
	p2 = cp2;
	a = aVal;
	b = bVal;
}