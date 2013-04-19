#include "HairMesh.h"
#include <GL/glut.h>
#include <algorithm>


double HairMesh::g_structuralKs = 5000.000; //3k 
double HairMesh::g_structuralKd = 5.000; //10
double HairMesh::g_shearKs = 4000.000; //4000
double HairMesh::g_shearKd = 6.00; //10
double HairMesh::g_bendKs = 4000.0000; //3000
double HairMesh::g_bendKd = 5.00; // 7
double HairMesh::g_penaltyKs = 6000.000; //5000
double HairMesh::g_penaltyKd = 320.000; //10

double HairMesh::friction_Mk = 0.5;
double HairMesh::friction_Amp = 2;
double HairMesh::COLLISION_THRESHOLD = 0.01; 
double HairMesh::jelloStartY = 1.3; //0.0


bool SHOULD_DRAW_HAIR = true;
// TODO
double HairMesh::g_attachmentKs = 0.000;
double HairMesh::g_attachmentKd = 0.000;

HairMesh::HairMesh() :     
    m_integrationType(HairMesh::RK4), m_drawflags(MESH | STRUCTURAL),
    m_cols(0), m_rows(0), m_stacks(0), m_width(0.0), m_height(0.0), m_depth(0.0)
{
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

bool HairMesh::isInterior(const HairMesh::Spring& s) const
{
    int i1,j1,k1,i2,j2,k2;
    GetCell(s.m_p1, i1, j1, k1);
    GetCell(s.m_p2, i2, j2, k2);
    return isInterior(i1,j1,k1) || isInterior(i2,j2,k2);
}


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
    double restLen = (p1.position - p2.position).Length();
    m_vsprings.push_back(Spring(STRUCTURAL, p1.index, p2.index, g_structuralKs, g_structuralKd, restLen));
}

void HairMesh::AddBendSpring(HairMesh::Particle& p1, HairMesh::Particle& p2)
{
    double restLen = (p1.position - p2.position).Length();
    m_vsprings.push_back(Spring(BEND, p1.index, p2.index, g_bendKs, g_bendKd, restLen));
}

void HairMesh::AddShearSpring(HairMesh::Particle& p1, HairMesh::Particle& p2)
{
    double restLen = (p1.position - p2.position).Length();
    m_vsprings.push_back(Spring(SHEAR, p1.index, p2.index, g_shearKs, g_shearKd, restLen));
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
{
    const ParticleGrid& g = m_vparticles;
    glBegin(GL_LINES);
    for (unsigned int i = 0; i < m_vsprings.size(); i++)
    {
        if (!(m_vsprings[i].m_type & m_drawflags)) continue;
        if (isInterior(m_vsprings[i])) continue;

        switch (m_vsprings[i].m_type)
        {
        case BEND:       glColor4f(1.0, 1.0, 0.0, a); break;
        case STRUCTURAL: glColor4f(1.0, 1.0, 0.0, a); break;
        case SHEAR:      glColor4f(0.0, 1.0, 1.0, a); break;
        };

        vec3 p1 = GetParticle(g, m_vsprings[i].m_p1).position;
        vec3 p2 = GetParticle(g, m_vsprings[i].m_p2).position;
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();
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

	if (SHOULD_DRAW_HAIR) {DrawHair(); DrawHairParticles();}

    glEnable(GL_LIGHTING);

}

void HairMesh::Update(double dt, const World& world, const vec3& externalForces)
{
    m_externalForces = externalForces;
	ComputeHairForces(dt);
	updateVelocity(dt);
	applyStrainLimiting(dt);			// v_n+dt/2
	applySelfRepulsions(dt);			// v_n+dt/2
	updatePosition(dt);				// x_n+1 = x_n + (dt/2)
	resolveBodyCollisions(dt);			// modify  x_n+1 and v_n
	resolveSelfCollisions(dt);			// modify  x_n+1 and v_n
	updateVelocity(dt);					// v_n+1/2 = v_n + dt/2a(tn+1/2, x_n, v_n+1/2)
	extrapolateVelocity(dt);			// v_n+1 = 2v_n+1/2 - v_n
	applySelfRepulsions(dt);			// Modify v_n+1 Use n+1
	
//	CheckForCollisions(m_vparticles, world);
//	ComputeForces(m_vparticles);
//	ResolveContacts(m_vparticles);
//	ResolveCollisions(m_vparticles);

    switch (m_integrationType)
    {
    case EULER: EulerIntegrate(dt); break;
    case MIDPOINT: MidPointIntegrate(dt); break;
    case RK4: RK4Integrate(dt); break;
    }
}

void HairMesh::CheckForCollisions(ParticleGrid& grid, const World& world)
{
    m_vcontacts.clear();
    m_vcollisions.clear();

    for (int i = 0; i < m_rows+1; i++)
    {
        for (int j = 0; j < m_cols+1; j++)
        {
            for (int k = 0; k < m_stacks+1; k++)
            {
                Particle& p = GetParticle(grid, i,j,k);

                // 1. Check collisions with world objects 
                for (unsigned int i = 0; i < world.m_shapes.size(); i++)
                {
                    Intersection intersection;

                    if (world.m_shapes[i]->GetType() == World::CYLINDER && 
                        CylinderIntersection(p, (World::Cylinder*) world.m_shapes[i], intersection))
                    {
						if (intersection.m_type == IntersectionType::CONTACT) 
						{
							m_vcontacts.push_back(intersection);
						}
						else if (intersection.m_type == IntersectionType::COLLISION) 
						{
							m_vcollisions.push_back(intersection);
						}
                    }
                    else if (world.m_shapes[i]->GetType() == World::GROUND && 
                        FloorIntersection(p, intersection))
                    {	
						if (intersection.m_type == IntersectionType::CONTACT) 
						{
							m_vcontacts.push_back(intersection);
						}
						else if (intersection.m_type == IntersectionType::COLLISION) 
						{
							m_vcollisions.push_back(intersection);
						}
                    }
					else if (world.m_shapes[i]->GetType() == World::SPHERE &&
						SphereIntersection(p, (World::Sphere*) world.m_shapes[i], intersection))
					{
						if (intersection.m_type == IntersectionType::CONTACT) 
						{
							m_vcontacts.push_back(intersection);
						}
						else if (intersection.m_type == IntersectionType::COLLISION) 
						{
							m_vcollisions.push_back(intersection);
						}
					}
                }
            }
        }
    }
}

void HairMesh::ComputeForces(ParticleGrid& grid)
{
    // Add external froces to all points
    for (int i = 0; i < m_rows+1; i++)
    {
        for (int j = 0; j < m_cols+1; j++)
        {
            for (int k = 0; k < m_stacks+1; k++)
            {
                Particle& p = GetParticle(grid, i,j,k);
                p.force = m_externalForces * p.mass;
            }
        }
    }

    // Update springs
    for(unsigned int i = 0; i < m_vsprings.size(); i++)
    {
        Spring& spring = m_vsprings[i];
        Particle& a = GetParticle(grid, spring.m_p1);
        Particle& b = GetParticle(grid, spring.m_p2);
		
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

//Object particle is below the surface of another object
void HairMesh::ResolveContacts(ParticleGrid& grid)
{
	//CONTACTS - Penetration of the other object
	for (unsigned int i = 0; i < m_vcontacts.size(); i++)
    {		
		//Con
		const Intersection& result = m_vcontacts[i];
		Particle& p = GetParticle(grid, result.m_p);
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

		//}

    }
}


//Object is within a threshold about the surface of another object
void HairMesh::ResolveCollisions(ParticleGrid& grid)
{
    //COLLISION - Close to Hitting other object
	for(unsigned int i = 0; i < m_vcollisions.size(); i++)
    {
		//Collision Details
        const Intersection result = m_vcollisions[i];
        Particle& p = GetParticle(grid, result.m_p);
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
	}
}

bool HairMesh::FloorIntersection(Particle& p, Intersection& intersection)
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
			intersection.m_p = p.index;
			intersection.m_distance = fabs(COLLISION_THRESHOLD-pY);			//Distance from top of the collision threshold
			intersection.m_type = IntersectionType::COLLISION;
			intersection.m_normal = N;
			intersection.m_ground_friction = friction_Mk;
			return true;
		}
		//Check if point is below surface of floor - Contact
		else if (pY < 0) {
			//cout<< "Floor Contact\n";
			intersection.m_p = p.index;
			intersection.m_distance = fabs(0.0-pY);							//Distance from the surface
			intersection.m_type = IntersectionType::CONTACT;
			intersection.m_normal = N;
			intersection.m_ground_friction = friction_Mk;
			return true;
		}
    return false;
}


bool HairMesh::SphereIntersection(Particle& p, World::Sphere* sphere, Intersection& intersection) {

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
		intersection.m_p = p.index;
		intersection.m_normal = N;
		intersection.m_distance = R.Length()-aLength;
		return true;
	}
	else if (radius < aLength && aLength < radius+COLLISION_THRESHOLD) {
		//In threshold right above surface
		intersection.m_type = IntersectionType::COLLISION;
		intersection.m_p = p.index;
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
	
	//if (0.0 <= cLength && cLength <= cylinderRadius) {
	//	//cout<<"within cylinder radius\n";
	//	//cout<<pt.position;
	//	
	//}
	//if ( 0.0 < projB.Length() && projB.Length() <= cylinderHeight) {
	//	//cout<<"Within cylinder height";
	//	//cout<< "projBL: "<<projB.Length()<<" cylinderHeight: " << cylinderHeight;
	//}



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
	ParticleGrid& source = m_vparticles; // source is a ptr!
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				//Update Particle position & velocity
				Particle& s = GetParticle(source, i,j,k);
				//p1 = p0 + t(v0) 
				s.position = s.position + dt*s.velocity;
				//v1 = v0 + t(F/m)
				s.velocity = s.velocity + (dt*s.force)*(1/s.mass);
			}
		}
	}
}

void HairMesh::MidPointIntegrate(double dt)
{
    double halfdt = 0.5 * dt;
	ParticleGrid target = m_vparticles; // target is a copy!
	ParticleGrid& source = m_vparticles; // source is a ptr!

	// Step 1 - take half-step move
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				//Evaluate single Euler Step and stop back into target
				Particle& s = GetParticle(source, i,j,k);
				Particle& t = GetParticle(target, i,j,k);
				// Evaluate half-euler step for velocity and position
				t.velocity = s.velocity + (halfdt*s.force)*1/s.mass;
				t.position = s.position + halfdt * s.velocity;
			}
		}
	}

	ComputeForces(target);

	// Step 2 Now evaluate midpoint and take full step using the midpoint derivatives
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				Particle& s = GetParticle(source, i,j,k);
				Particle& t = GetParticle(target, i,j,k);
				s.velocity = s.velocity + (dt*t.force)*(1/s.mass);
				s.position = s.position + (dt*t.velocity);
			}
		}
	}
}

void HairMesh::RK4Integrate(double dt)
{
	double halfdt = 0.5 * dt;
	ParticleGrid target = m_vparticles; // target is a copy!
	ParticleGrid& source = m_vparticles; // source is a ptr!

	// Step 1
	ParticleGrid accum1 = m_vparticles;
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				Particle& s = GetParticle(source, i,j,k);

				Particle& k1 = GetParticle(accum1, i,j,k);
				k1.force = s.force;
				k1.velocity = s.velocity;

				Particle& t = GetParticle(target, i,j,k);
				t.velocity = s.velocity + halfdt * k1.force * 1/k1.mass;
				t.position = s.position + halfdt * k1.velocity;
			}
		}
	}

	ComputeForces(target);

	// Step 2
	ParticleGrid accum2 = m_vparticles;
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				Particle& t = GetParticle(target, i,j,k);
				Particle& k2 = GetParticle(accum2, i,j,k);

				k2.force = t.force;
				k2.velocity = t.velocity;

				Particle& s = GetParticle(source, i,j,k);
				t.velocity = s.velocity + halfdt * k2.force * 1/k2.mass;
				t.position = s.position + halfdt * k2.velocity;
			}
		}
	}

	ComputeForces(target);

	// Step 3
	ParticleGrid accum3 = m_vparticles;
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				Particle& t = GetParticle(target, i,j,k);
				Particle& k3 = GetParticle(accum3, i,j,k);

				k3.force = t.force;
				k3.velocity = t.velocity;

				Particle& s = GetParticle(source, i,j,k);
				t.velocity = s.velocity + dt * k3.force * 1/k3.mass;
				t.position = s.position + dt * k3.velocity;
			}
		}
	}
	ComputeForces(target);

	// Step 4
	ParticleGrid accum4 = m_vparticles;
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				Particle& t = GetParticle(target, i,j,k);
				Particle& k4 = GetParticle(accum4, i,j,k);

				k4.force = t.force;
				k4.velocity = t.velocity;
			}
		}
	}

	// Put it all together
	double asixth = 1/6.0;
	double athird = 1/3.0;
	for (int i = 0; i < m_rows+1; i++)
	{
		for (int j = 0; j < m_cols+1; j++)
		{
			for (int k = 0; k < m_stacks+1; k++)
			{
				Particle& p = GetParticle(m_vparticles, i,j,k);
				Particle& k1 = GetParticle(accum1, i,j,k);
				Particle& k2 = GetParticle(accum2, i,j,k);
				Particle& k3 = GetParticle(accum3, i,j,k);
				Particle& k4 = GetParticle(accum4, i,j,k);

				p.velocity = p.velocity + dt*(asixth * k1.force +
					athird * k2.force + athird * k3.force + asixth * k4.force)*1/p.mass;

				p.position = p.position + dt*(asixth * k1.velocity +
				athird * k2.velocity + athird * k3.velocity + asixth * k4.velocity);
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
    m_Ks(1.0), m_Kd(1.0), m_restLen(1.0)
{
}

HairMesh::Spring::Spring(const HairMesh::Spring& p) :
    m_type(p.m_type), m_p1(p.m_p1), m_p2(p.m_p2),
    m_Ks(p.m_Ks), m_Kd(p.m_Kd), m_restLen(p.m_restLen)
{
}

HairMesh::Spring& HairMesh::Spring::operator=(const HairMesh::Spring& p)
{
    if (&p == this) return *this;

    m_type = p.m_type;
    m_p1 = p.m_p1;
    m_p2 = p.m_p2;
    m_Ks = p.m_Ks;
    m_Kd = p.m_Kd;
    m_restLen = p.m_restLen;
    return *this;
}

HairMesh::Spring::Spring(HairMesh::SpringType t, 
    int p1, int p2, double Ks, double Kd, double restLen) :
    m_type(t), m_Ks(Ks), m_Kd(Kd), m_p1(p1), m_p2(p2), m_restLen(restLen)
{
}

//---------------------------------------------------------------------
// Particle
//---------------------------------------------------------------------

HairMesh::Particle HairMesh::Particle::EMPTY;

HairMesh::Particle::Particle(int idx, const vec3& p, const vec3& v, double m)
{
    index = idx;
    position = p;
    velocity = v;
    force = vec3(0,0,0);
    mass = m;
}

HairMesh::Particle::Particle() : index(-1), position(0,0,0), velocity(0,0,0), force(0,0,0), mass(1.0)
{
}

HairMesh::Particle::Particle(const HairMesh::Particle& p) : 
    index(p.index), position(p.position), velocity(p.velocity), force(p.force), mass(p.mass)
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
    return *this;
}

//---------------------------------------------------------------------
// Intersection
//---------------------------------------------------------------------

HairMesh::Intersection::Intersection() : 
	m_p(-1), m_normal(0,0,0), m_distance(0) , m_type(CONTACT) , m_ground_friction(0.0)
{
}

HairMesh::Intersection::Intersection(const HairMesh::Intersection& p) :
	m_p(p.m_p), m_normal(p.m_normal), m_distance(p.m_distance), m_type(p.m_type), m_ground_friction(p.m_ground_friction)
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
    const ParticleGrid& g = m.m_vparticles;
    for (unsigned int strip = 0; strip < m_strips.size(); strip++)
    {
        const std::vector<int>& points = m_strips[strip];

        glBegin(GL_TRIANGLE_STRIP);
        for (unsigned int pi = 0; pi < points.size(); pi++)
        {
            int idx = points[pi];
            vec3 p = m.GetParticle(g, idx).position;

            vec3 n(0,0,0);
            const std::vector<int>& neighbors = m_neighbors[idx];
            if (neighbors.size() > 0)
            {
                vec3 pup = m.GetParticle(g, neighbors[0]).position;
                vec3 pdown = m.GetParticle(g, neighbors[1]).position;
                vec3 pleft = m.GetParticle(g, neighbors[2]).position;
                vec3 pright = m.GetParticle(g, neighbors[3]).position;

                vec3 n1 = -((pright - p) ^ (pup - p));
                vec3 n2 = -((pdown - p) ^ (pright - p));
                vec3 n3 = -((pleft - p) ^ (pdown - p));
                vec3 n4 = -((pup - p) ^ (pleft - p));

                n = n1 + n2 + n3 + n4;
                n = n.Normalize();
            }

            glNormal3f(n[0], n[1], n[2]);
            glVertex3f(p[0], p[1], p[2]);
        }
        glEnd();
    }
}

void HairMesh::FaceMesh::DrawNormals(const HairMesh& m)
{
    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 0.0);

    const ParticleGrid& g = m.m_vparticles;
    for (unsigned int strip = 0; strip < m_strips.size(); strip++)
    {
        const std::vector<int>& points = m_strips[strip];
        for (unsigned int pi = 0; pi < points.size(); pi++)
        {
            int idx = points[pi];
            vec3 p = m.GetParticle(g, idx).position;

            const std::vector<int>& neighbors = m_neighbors[idx];
            if (neighbors.size() == 0) continue;

            vec3 pup = m.GetParticle(g, neighbors[0]).position;
            vec3 pdown = m.GetParticle(g, neighbors[1]).position;
            vec3 pleft = m.GetParticle(g, neighbors[2]).position;
            vec3 pright = m.GetParticle(g, neighbors[3]).position;

            vec3 n1 = -((pright - p) ^ (pup - p));
            vec3 n2 = -((pdown - p) ^ (pright - p));
            vec3 n3 = -((pleft - p) ^ (pdown - p));
            vec3 n4 = -((pup - p) ^ (pleft - p));

            vec3 n = n1 + n2 + n3 + n4;
            n = n.Normalize();

            vec3 end = p + 0.2 * n;
            glVertex3f(p[0], p[1], p[2]);
            glVertex3f(end[0], end[1], end[2]);
        }
    }

    glEnd();
    glEnable(GL_LIGHTING);
}

#define R(i) max(0, min(i, m.m_rows)) // CLAMP row index
#define C(j) max(0, min(j, m.m_cols)) // CLAMP col index
#define D(j) max(0, min(j, m.m_stacks)) // CLAMP stack index
HairMesh::FaceMesh::FaceMesh(const HairMesh& m, HairMesh::Face f)
{
    const ParticleGrid& g = m.m_vparticles;
    switch(f)
    {
    case ZFRONT:
        m_strips.resize(m.m_rows);
        for (int i = 0; i < m.m_rows+1; i++)
            for (int j = 0; j < m.m_cols+1; j++)
            {
                if (i < m.m_rows)
                {
                    m_strips[i].push_back(m.GetIndex(i+1,j,0));
                    m_strips[i].push_back(m.GetIndex(i,j,0));
                }

                std::vector<int> neighbors;
                neighbors.push_back(m.GetIndex(R(i), C(j+1), D(0)));
                neighbors.push_back(m.GetIndex(R(i), C(j-1), D(0)));
                neighbors.push_back(m.GetIndex(R(i-1), C(j), D(0)));
                neighbors.push_back(m.GetIndex(R(i+1), C(j), D(0)));
                m_neighbors[m.GetIndex(i,j,0)] = neighbors;
            }
        break;
    case ZBACK:
        m_strips.resize(m.m_rows);
        for (int i = 0; i < m.m_rows+1; i++)
            for (int j = 0; j < m.m_cols+1; j++)
            {
                if (i < m.m_rows)
                {
                    m_strips[i].push_back(m.GetIndex(i+1,j,m.m_stacks));
                    m_strips[i].push_back(m.GetIndex(i,j,m.m_stacks));
                }

                std::vector<int> neighbors;
                neighbors.push_back(m.GetIndex(R(i+1), C(j), D(m.m_stacks)));
                neighbors.push_back(m.GetIndex(R(i-1), C(j), D(m.m_stacks)));
                neighbors.push_back(m.GetIndex(R(i), C(j-1), D(m.m_stacks)));
                neighbors.push_back(m.GetIndex(R(i), C(j+1), D(m.m_stacks)));
                m_neighbors[m.GetIndex(i,j,m.m_stacks)] = neighbors;
            }
        break;
    case XLEFT:
        m_strips.resize(m.m_cols);
        for (int j = 0; j < m.m_cols+1; j++)
            for (int k = 0; k < m.m_stacks+1; k++)
            {
                if (j < m.m_cols)
                {
                    m_strips[j].push_back(m.GetIndex(0,j+1,k));
                    m_strips[j].push_back(m.GetIndex(0,j,k));
                }

                std::vector<int> neighbors;
                neighbors.push_back(m.GetIndex(R(0), C(j), D(k+1)));
                neighbors.push_back(m.GetIndex(R(0), C(j), D(k-1)));
                neighbors.push_back(m.GetIndex(R(0), C(j-1), D(k)));
                neighbors.push_back(m.GetIndex(R(0), C(j+1), D(k)));
                m_neighbors[m.GetIndex(0,j,k)] = neighbors;
            }
        break;
    case XRIGHT:
        m_strips.resize(m.m_cols);
        for (int j = 0; j < m.m_cols+1; j++)
            for (int k = 0; k < m.m_stacks+1; k++)
            {
                if (j < m.m_cols)
                {
                    m_strips[j].push_back(m.GetIndex(m.m_rows,j+1,k));
                    m_strips[j].push_back(m.GetIndex(m.m_rows,j,k));
                }

                std::vector<int> neighbors;
                neighbors.push_back(m.GetIndex(R(m.m_rows), C(j+1), D(k)));
                neighbors.push_back(m.GetIndex(R(m.m_rows), C(j-1), D(k)));
                neighbors.push_back(m.GetIndex(R(m.m_rows), C(j), D(k-1)));
                neighbors.push_back(m.GetIndex(R(m.m_rows), C(j), D(k+1)));
                m_neighbors[m.GetIndex(m.m_rows,j,k)] = neighbors;
            }
        break;
    case YBOTTOM:
        m_strips.resize(m.m_rows);
        for (int i = 0; i < m.m_rows+1; i++)
            for (int k = 0; k < m.m_stacks+1; k++)
            {
                if (i < m.m_rows)
                {
                    m_strips[i].push_back(m.GetIndex(i+1,0,k));
                    m_strips[i].push_back(m.GetIndex(i,0,k));
                }

                std::vector<int> neighbors;
                neighbors.push_back(m.GetIndex(R(i+1), C(0), D(k)));
                neighbors.push_back(m.GetIndex(R(i-1), C(0), D(k)));
                neighbors.push_back(m.GetIndex(R(i), C(0), D(k-1)));
                neighbors.push_back(m.GetIndex(R(i), C(0), D(k+1)));
                m_neighbors[m.GetIndex(i,0,k)] = neighbors;
            }
        break;
    case YTOP:
        m_strips.resize(m.m_rows);
        for (int i = 0; i < m.m_rows+1; i++)
            for (int k = 0; k< m.m_stacks+1; k++)
            {
                if (i < m.m_rows)
                {
                    m_strips[i].push_back(m.GetIndex(i+1,m.m_cols,k));
                    m_strips[i].push_back(m.GetIndex(i,m.m_cols,k));
                }

                std::vector<int> neighbors;
                neighbors.push_back(m.GetIndex(R(i), C(m.m_cols), D(k+1)));
                neighbors.push_back(m.GetIndex(R(i), C(m.m_cols), D(k-1)));
                neighbors.push_back(m.GetIndex(R(i-1), C(m.m_cols), D(k)));
                neighbors.push_back(m.GetIndex(R(i+1), C(m.m_cols), D(k)));
                m_neighbors[m.GetIndex(i,m.m_cols,k)] = neighbors;
            }
        break;
    }
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

//###################################################
//################ InitHairMesh #####################
//###################################################
void HairMesh::InitHairMesh()
{
    m_vsprings.clear();

   // if (m_width < 0.01 || m_height < 0.01 || m_depth < 0.01) return;
   // if (m_cols < 1 || m_rows < 1 || m_stacks < 1) return;

   // // Init particles
   // float wcellsize = m_width / m_cols;
   // float hcellsize = m_height / m_rows;
   // float dcellsize = m_depth / m_stacks;
   // 
   // for (int i = 0; i < m_rows+1; i++)
   // {
   //     for (int j = 0; j < m_cols+1; j++)
   //     {
   //         for (int k = 0; k < m_stacks+1; k++)
   //         {
   //             float x = -m_width*0.5f + wcellsize*i;
   //             float y = 0.5 + hcellsize*j+jelloStartY;  /// Starting Height of cube
   //             float z = -m_depth*0.5f + dcellsize*k;
   //             m_vparticles[i][j][k] = Particle(GetIndex(i,j,k), vec3(x, y, z));
   //         }
   //     }
   // }

   // // Setup structural springs
   // ParticleGrid& g = m_vparticles;
   // for (int i = 0; i < m_rows+1; i++)
   // {
   //     for (int j = 0; j < m_cols+1; j++)
   //     {
   //         for (int k = 0; k < m_stacks+1; k++)
   //         {
			//	//Structural Springs
			//	if (j < m_cols) AddStructuralSpring(GetParticle(g,i,j,k), GetParticle(g,i,j+1,k));
			//	if (i < m_rows) AddStructuralSpring(GetParticle(g,i,j,k), GetParticle(g,i+1,j,k));
			//	if (k < m_stacks) AddStructuralSpring(GetParticle(g,i,j,k), GetParticle(g,i,j,k+1));
			//	
			//	//Bend Springs
			//	if (i < m_rows-1) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i+2,j,k));
			//	if (j < m_cols-1) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i,j+2,k));
			//	if (k < m_stacks-1) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i,j,k+2));
			//	
			//	//Shear Springs (Required) 
			//	if (i < m_rows && j < m_cols){
			//		AddShearSpring(GetParticle(g,i,j,k), GetParticle(g,i+1,j+1,k));
			//		AddShearSpring(GetParticle(g,i+1,j,k), GetParticle(g,i,j+1,k));
			//	}
			//	if (i < m_rows && k < m_stacks){
			//		AddShearSpring(GetParticle(g,i,j,k), GetParticle(g,i+1,j,k+1));
			//		AddShearSpring(GetParticle(g,i+1,j,k), GetParticle(g,i,j,k+1));
			//	}
			//	if (j < m_cols && k < m_stacks){
			//		AddShearSpring(GetParticle(g,i,j,k), GetParticle(g,i,j+1,k+1));
			//		AddShearSpring(GetParticle(g,i,j,k+1), GetParticle(g,i,j+1,k));
			//	}

			//	//Additional shear springs (across diagonals of cube
			//	//Shear Springs (Required) 
			//	if (i < m_rows && j < m_cols && k < m_stacks){
			//		AddShearSpring(GetParticle(g,i,j,k), GetParticle(g, i+1, j+1, k+1));
			//	}
			//	if (i > 0.0 && j > m_cols && k < m_stacks){
			//		AddShearSpring(GetParticle(g,i,j,k), GetParticle(g, i-1, j+1, k+1));
			//	}
			//	if (i < m_rows && j > 0.0 && k < m_stacks){
			//		AddShearSpring(GetParticle(g,i,j,k), GetParticle(g, i+1, j-1, k+1));
			//	}
			//	if ( i > 0.0 && j > 0.0 && k < m_stacks){
			//		AddShearSpring(GetParticle(g,i,j,k), GetParticle(g, i-1, j-1, k+1));
			//	}
			//	

			//	//Additional Bend Springs for 3 & 5 & 7
			//	if (i < m_rows-2) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i+3,j,k));
			//	if (j < m_cols-2) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i,j+3,k));
			//	if (k < m_stacks-2) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i,j,k+3));
			//	
			//	if (i < m_rows-3) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i+4,j,k));
			//	if (j < m_cols-3) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i,j+4,k));
			//	if (k < m_stacks-3) AddBendSpring(GetParticle(g,i,j,k), GetParticle(g,i,j,k+4));
			//	

			//}


	//##########################################################
	//#################### HAIR INITIALIZATION #################
	//##########################################################
	//Init Hairs
	this->StrandList = HairStrandList();
	
	//Make a strand
	HairStrand h = HairStrand(vec3(0, 1, 0));
	//Add to our strandList
	StrandList.addStrand(h);

}

//##########################################################
//######################### DRAW HAIR ######################
//##########################################################
void HairMesh::DrawHair() {
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
			
			//Current Particle in a strand
			const Particle p0 = hairParticles[k];

			// GHOST PARTICLES ARE ODD INDICES, HAIR PARTICLES ARE EVEN
			if (k % 2 == 0) glColor3f(0.0, 1.0, 0.0);
			else glColor3f(1.0, 0.0, 0.0);
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

}

void HairMesh::applySelfRepulsions(double dt) {

}

void HairMesh::updatePosition(double dt) {
	for (unsigned int i = 0; i < StrandList.size(); i++) {
		//Current Strand
		HairStrand& strand = StrandList.getStrand(i);
		//Strand Particles
		ParticleList& particles = strand.strandParticles;
		//Loop through and apple forces
		for (unsigned int j = 0; j < particles.size(); j++) {
			Particle& p = particles[j];
			p.position += (dt)*p.velocity;
		}
	}

}
void HairMesh::resolveBodyCollisions(double dt) {

}

void HairMesh::resolveSelfCollisions(double dt) {

}
void HairMesh::updateVelocity(double dt) {
	for (unsigned int i = 0; i < StrandList.size(); i++) {
		//Current Strand
		HairStrand& strand = StrandList.getStrand(i);
		//Strand Particles
		ParticleList& particles = strand.strandParticles;
		//Loop through and apple forces
		for (unsigned int j = 0; j < particles.size(); j++) {
			Particle& p = particles[j];
			p.velocity += (dt/2)*p.mass*p.force; // TODO what actually belongs from the paper
			cout<< p.velocity << endl;
		}
	}

}

void HairMesh::extrapolateVelocity(double dt) { 

}

//Add Gravity and that good stuff to the particles
void HairMesh::ComputeHairForces(double dt) {
	for (unsigned int i = 0; i < StrandList.size(); i++) {
		//Current Strand
		HairStrand& strand = StrandList.getStrand(i);
		//Strand Particles
		ParticleList& particles = strand.strandParticles;
		//Loop through and apple forces
		for (unsigned int j = 0; j < particles.size(); j++) {
			Particle& p = particles[j];
			p.force = m_externalForces * p.mass; // TODO what actually belongs from the paper
			cout << p.force << endl;
		}
	}
	
   // Update springs
  //  for(unsigned int i = 0; i < m_vsprings.size(); i++)
  //  {
  //      Spring& spring = m_vsprings[i];
  //      Particle& a = GetParticle(grid, spring.m_p1);
  //      Particle& b = GetParticle(grid, spring.m_p2);
		//
		////Spring Variables
		//double Ks = spring.m_Ks;				 //Hook's Spring Constant
		//double Kd = spring.m_Kd;				 //Damping Constant
		//double R = spring.m_restLen;			 //Rest Length of Spring
		//vec3 L = a.position - b.position;			 //Vector from B to A (position)
		//vec3 normL = L/L.Length();				 //Normalized L vector
		//vec3 diffVelocity = a.velocity - b.velocity;	 //Vector Velocities
		//double length = L.Length()-R;
		////Stiffness Spring Force:	-Ks(|L|-R)*L/(|L|);
		////Damping Force:			-kd(((v_a-v_b)*L/|L|)* L/|L|);

		//vec3 Fs = -1*Ks*(length)*L.Normalize();
		//vec3 Fd = -1*Kd*((L*diffVelocity)/L.Length())*L.Normalize();

		////One particle is -F the other is F
		//a.force += Fs + Fd;
		//b.force += -1*(Fs+Fd);
  //  }
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
	InitStrand();
}


//#######################################################
//########## HairStrand Equality Constructor ############
//#######################################################
HairMesh::HairStrand::HairStrand(const HairMesh::HairStrand& h) : 
    index(h.index), position(h.position)
{
	strandParticles = h.strandParticles;
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
	for (int i = 1; i < numTotalParticles; i++) {
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

    // Setup structural springs
    ParticleList& list = strandParticles;
	for (int i = 0; i < numTotalParticles; i++) {
		// TODO: ADD SPRINGS (see InitHairMesh)
		//cout << list[i].position << endl;
    }

    // Init mesh geometry
   /* m_mesh.clear();
    m_mesh.push_back(FaceMesh(*this,XLEFT));
    m_mesh.push_back(FaceMesh(*this,XRIGHT));
    m_mesh.push_back(FaceMesh(*this,YTOP));
    m_mesh.push_back(FaceMesh(*this,YBOTTOM));
    m_mesh.push_back(FaceMesh(*this,ZFRONT));
    m_mesh.push_back(FaceMesh(*this,ZBACK));
    */
}