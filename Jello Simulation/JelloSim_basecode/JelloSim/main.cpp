#include <stdio.h>
#include <math.h>
#include "camera.h"
#include "fps.h"
#include <GL/glut.h>
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>

#include "JelloMesh.h"
#include "HairMesh.h"
#include "world.h"

JelloMesh theJello;
HairMesh theHair;
Camera theCamera;
//World theWorld("worlds/ground.xml");				//Given Ground Scene
//World theWorld("worlds/cylinders.xml");			//Given cylinder scene
//World theWorld("worlds/coolScene.xml");			//Cool Custom scene with bunch of objects
//World theWorld("worlds/myCylinder2.xml");			//Test Cylinder lying on floor
//World theWorld("worlds/sphere.xml");				//Sphere testing
//World theWorld("worlds/cylinderBottom.xml");		//Test for top Cap - Cylinder facing vertically
World theWorld = World("worlds/sphereHead.xml");			//Large sphere head
World theWorldCopy = World("worlds/sphereHead.xml");
mmc::FpsTracker theFpsTracker;

// UI Helpers
int lastX = 0, lastY = 0;
int theMenu = 0;
int theButtonState = 0;
int theModifierState = 0;
int theFrameNum = 0;
bool isRunning = true;
bool isRecording = false;

void initCamera()
{
   double w = theHair.GetWidth()*2;   
   double h = theHair.GetHeight()*2;   
   double d = theHair.GetDepth()*2;   
   double angle = 0.5*theCamera.dfltVfov*M_PI/180.0;
   double dist;
   if (w > h) dist = w*0.5/tan(angle);  // aspect is 1, so i can do this
   else dist = h*0.5/tan(angle);
   theCamera.dfltEye.set(w*0.5, h, -(dist+d));
   theCamera.dfltLook.set(0.0, 0.0, 0.0);
   theCamera.reset();
}

void grabScreen()  
{
    unsigned int image;
    ilGenImages(1, &image);
    ilBindImage(image);

    ILenum error = ilGetError();
    assert(error == IL_NO_ERROR);

    ilTexImage(640, 480, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, NULL);

    error = ilGetError();
    assert(error == IL_NO_ERROR);

    unsigned char* data = ilGetData();

    error = ilGetError();
    assert(error == IL_NO_ERROR);

    for (int i=479; i>=0; i--) 
    {
	    glReadPixels(0,i,640,1,GL_RGB, GL_UNSIGNED_BYTE, 
		    data + (640 * 3 * i));
    }

    char anim_filename[2048];
    sprintf_s(anim_filename, 2048, "output/%04d.png", theFrameNum++); 

    ilSave(IL_PNG, anim_filename);

    error = ilGetError();
    assert(error == IL_NO_ERROR);

    ilDeleteImages(1, &image);

    error = ilGetError();
    assert(error == IL_NO_ERROR);
}

void onMouseMotionCb(int x, int y)
{
   int deltaX = lastX - x;
   int deltaY = lastY - y;
   bool moveLeftRight = abs(deltaX) > abs(deltaY);
   bool moveUpDown = !moveLeftRight;

   if (theButtonState == GLUT_LEFT_BUTTON)  // Rotate
   {
      if (moveLeftRight && deltaX > 0) theCamera.orbitLeft(deltaX);
      else if (moveLeftRight && deltaX < 0) theCamera.orbitRight(-deltaX);
      else if (moveUpDown && deltaY > 0) theCamera.orbitUp(deltaY);
      else if (moveUpDown && deltaY < 0) theCamera.orbitDown(-deltaY);
   }
   else if (theButtonState == GLUT_LEFT_BUTTON && theModifierState == GLUT_ACTIVE_SHIFT)
   {
	   	   // Zoom
	     if (moveUpDown && deltaY > 0) theCamera.moveForward(deltaY);
         else if (moveUpDown && deltaY < 0) theCamera.moveBack(-deltaY);


   }
   else if (theButtonState == GLUT_MIDDLE_BUTTON) // Zoom
   {
       if (theModifierState & GLUT_ACTIVE_ALT) // camera move
       {
            if (moveLeftRight && deltaX > 0) theCamera.moveLeft(deltaX);
            else if (moveLeftRight && deltaX < 0) theCamera.moveRight(-deltaX);
            else if (moveUpDown && deltaY > 0) theCamera.moveUp(deltaY);
            else if (moveUpDown && deltaY < 0) theCamera.moveDown(-deltaY);
       }
       else
       {
           if (moveUpDown && deltaY > 0) theCamera.moveForward(deltaY);
           else if (moveUpDown && deltaY < 0) theCamera.moveBack(-deltaY);
       }

   }    
 
   lastX = x;
   lastY = y;
   glutPostRedisplay();
}

void onMouseCb(int button, int state, int x, int y)
{
   theButtonState = button;
   theModifierState = glutGetModifiers();
   lastX = x;
   lastY = y;
   glutSetMenu(theMenu);
}


void onKeyboardCb(unsigned char key, int x, int y)
{
   unsigned int mask = 0x0;
 
   if (key == ' ') theCamera.reset();
   else if (key == 27) exit(0); // ESC Key
   //else if (key == '8') theHair.SetIntegrationType(HairMesh::EULER);
   //else if (key == '9') theHair.SetIntegrationType(HairMesh::MIDPOINT);
   else if (key == '0') theHair.SetIntegrationType(HairMesh::RK4);
   else if (key == '>') isRunning = true;
   else if (key == '=') isRunning = false;
   else if (key == '<') {

	   theHair.Reset();
	   delete theWorld.m_shapes[1];
	   theWorld.m_shapes[1] = new World::Shape(theWorldCopy.m_shapes[1]);
   }
   else if (key == 'r') isRecording = !isRecording; if (isRecording) theFrameNum = 0;
   else if (key == '1') theHair.SHOULD_DRAW_HAIR = !(theHair.SHOULD_DRAW_HAIR);
   else if (key == '2') mask = theHair.FORCES;
   else if (key == '3') mask = theHair.NORMALS;
   //else if (key == '4') mask = theHair.STRUCTURAL;
   //else if (key == '5') mask = theHair.SHEAR;
   else if (key == '4') mask = theHair.EDGE;
   else if (key == '5') mask = theHair.TORSION;
   else if (key == '6') mask = theHair.BEND;
   else if (key == '7') mask = theHair.STICTION;
   else if (key == 'q') theHair.SHOULD_DRAW_HAIR_PARTICLES = !theHair.SHOULD_DRAW_HAIR_PARTICLES;
   else if (key == 'w') theHair.SHOULD_DRAW_GHOST_PARTICLES = !theHair.SHOULD_DRAW_GHOST_PARTICLES;
   else if (key == 'e') theHair.SHOULD_DRAW_STICTION_PARTICLES = !theHair.SHOULD_DRAW_STICTION_PARTICLES;
   //Move Left
   else if (key == 'a') {
		World::Shape* shape = theWorld.m_shapes[1];
		shape->move(-0.01, 0.0);
		theHair.moveHairStrandTranslate(0, -0.01, 0.00);
   } else if (key == 's') {
	     //Move Up
	   grabScreen();
		World::Shape* shape = theWorld.m_shapes[1];
		shape->move(0.0, 0.01);
		theHair.moveHairStrandTranslate(0, 0.0, 0.01);
   } else if (key == 'x') {
	     //Move Up
		//theHair.moveHairStrandUp(0);
		World::Shape* shape = theWorld.m_shapes[1];
		shape->move(0.0, -0.01);
		theHair.moveHairStrandTranslate(0, 0.0, -0.01);
	} else if (key == 'd') {
		//Move Right
		//theHair.moveHairStrandUp(0);
		  grabScreen();
		World::Shape* shape = theWorld.m_shapes[1];
		shape->move(0.01, 0.0);
		theHair.moveHairStrandTranslate(0, 0.01, 0.0);
   } else if (key == 'z') {
		//Rotate Right
		//theHair.moveHairStrandUp(0);
	   std::vector<World::Shape*>& shapes = theWorld.m_shapes;
		World::Shape* shape = theWorld.m_shapes[1];
		shape->rotate(1.0);
		theHair.moveHairStrandRotate(0, 1);
   } else if (key == 'c') {
		//Rotate Right
		//theHair.moveHairStrandUp(0);
		World::Shape* shape = theWorld.m_shapes[1];
		shape->rotate(-1.0);
		theHair.moveHairStrandRotate(0, -1);
   }

   //World::Shape* shape = theWorld.m_shapes[1];
	//cout << shape->pos <<endl;
	//std::vector<World::Shape*>& shapes = theWorld.m_shapes;	

   if (mask)
   {
       if (theHair.GetDrawFlags() & mask)
       {
           theHair.SetDrawFlags(theHair.GetDrawFlags() & ~mask);
       }
       else
       {
           theHair.SetDrawFlags(theHair.GetDrawFlags() | mask);
       }
   }

   glutPostRedisplay();
}

void onMenuCb(int value)
{
   switch (value)
   {
   case -1: exit(0);
   default: onKeyboardCb(value, 0, 0); break;
   }
}

void onKeyboardSpecialCb(int key, int x, int y)
{
}

void onTimerCb(int value)
{
   glutTimerFunc(100, onTimerCb, 0);
   if (isRunning) 
   {
       theHair.Update(0.01, theWorld); 
       if (isRecording) 
		   grabScreen();
   }

   glutPostRedisplay();
}

void onResizeCb(int width, int height)
{
   // Update viewport
   glViewport(0, 0, width, height);

   // Update camera projection's aspect ratio
   float vfov, aspect, zNear, zFar;
   theCamera.getProjection(&vfov, &aspect, &zNear, &zFar);
   theCamera.setProjection(vfov, ((GLfloat) width)/height, zNear, zFar);
}

void drawOverlay()
{
  // Draw Overlay
  glColor4f(1.0, 1.0, 1.0, 1.0);
  glPushAttrib(GL_LIGHTING_BIT);
     glDisable(GL_LIGHTING);

     glMatrixMode(GL_PROJECTION);
     glLoadIdentity();
     gluOrtho2D(0.0, 1.0, 0.0, 1.0);

     glMatrixMode(GL_MODELVIEW);
     glLoadIdentity();
     glRasterPos2f(0.01, 0.01);
     
     char* intstr;
     switch (theHair.GetIntegrationType())
     {
     case HairMesh::EULER: intstr = "Euler"; break;
     case HairMesh::MIDPOINT: intstr = "Midpoint"; break;
     case HairMesh::RK4: intstr = "RK4"; break;
     }

     char info[1024];
     sprintf_s(info, "Framerate: %3.1f %s %s", 
         theFpsTracker.fpsAverage(),
         intstr, isRecording? "(Recording ON)" : "");
 
     for (unsigned int i = 0; i < strlen(info); i++)
     {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, info[i]);
     }
  glPopAttrib();
}

void drawAxes()
{
  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
      glDisable(GL_LIGHTING);

      glLineWidth(2.0); 
      glBegin(GL_LINES);
         glColor3f(1.0, 0.0, 0.0);
         glVertex3f(0.0, 0.0, 0.0);
         glVertex3f(1.0, 0.0, 0.0);

         glColor3f(0.0, 1.0, 0.0);
         glVertex3f(0.0, 0.0, 0.0);
         glVertex3f(0.0, 1.0, 0.0);

         glColor3f(0.0, 0.0, 1.0);
         glVertex3f(0.0, 0.0, 0.0);
         glVertex3f(0.0, 0.0, 1.0);
      glEnd();
  glPopAttrib();
}

void onDrawCb()
{
    // Keep track of time
    theFpsTracker.timestamp();

    // Draw Scene and overlay
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    theCamera.draw();
    //drawAxes();

    vec3 cpos = theCamera.getPosition();
    float pos[4] = {cpos[0], cpos[1]+2.0, cpos[2],0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    theWorld.Draw();
    theHair.Draw(cpos);
    drawOverlay();
    glutSwapBuffers();
}

void init(void)
{
    initCamera();
    glClearColor(0.8, 0.8, 0.8, 1.0);

    glEnable(GL_BLEND);
    glEnable(GL_ALPHA_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_NORMALIZE);
    glCullFace(GL_FRONT);


    float white[4] = {1.0,1.0,1.0,1.0};
    float black[4] = {0.0,0.0,0.0,1.0};
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_AMBIENT, black);

    GLfloat fogColor[4]= {0.8f, 0.8f, 0.8f, 1.0f};	
    glFogi(GL_FOG_MODE, GL_LINEAR);		// Fog Mode
    glFogfv(GL_FOG_COLOR, fogColor);			// Set Fog Color
    glFogf(GL_FOG_DENSITY, 0.35f);				// How Dense Will The Fog Be
    glHint(GL_FOG_HINT, GL_DONT_CARE);			// Fog Hint Value
    glFogf(GL_FOG_START, 10.0f);				// Fog Start Depth
    glFogf(GL_FOG_END, 40.0f);				// Fog End Depth
    glEnable(GL_FOG);					// Enables GL_FOG
}

int main(int argc, char **argv)
{
    ilInit();
    iluInit();
    ilEnable(IL_FILE_OVERWRITE);
    ilutRenderer(ILUT_OPENGL);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("JelloSim by Michael Rivera");
    glutDisplayFunc(onDrawCb);
    glutKeyboardFunc(onKeyboardCb);
    glutSpecialFunc(onKeyboardSpecialCb);
    glutMouseFunc(onMouseCb);
    glutMotionFunc(onMouseMotionCb); 
    glutTimerFunc(100, onTimerCb, 0);
    glutReshapeFunc(onResizeCb);

    int intMenu = glutCreateMenu(onMenuCb);
    glutAddMenuEntry("Euler\t'8'", '8');
    glutAddMenuEntry("Midpoint\t'9'", '9');
    glutAddMenuEntry("RK4\t'0'", '0');

    int displayMenu = glutCreateMenu(onMenuCb);
    glutAddMenuEntry("Mesh\t'1'", '1');
    glutAddMenuEntry("Forces\t'2'", '2');
    glutAddMenuEntry("Collision Normals\t'3'", '3');
    glutAddMenuEntry("Structural Springs\t'4'", '4');
    glutAddMenuEntry("Shear Springs\t'5'", '5');
    glutAddMenuEntry("Bend Springs\t'6'", '6');

    theMenu = glutCreateMenu(onMenuCb);
    glutAddMenuEntry("Start\t'>'", '>');
    glutAddMenuEntry("Pause\t'='", '=');
    glutAddMenuEntry("Reset\t'<'", '<');
    glutAddMenuEntry("Record\t'r'", 'r');
    glutAddSubMenu("Integration Type", intMenu);
    glutAddSubMenu("Draw Settings", displayMenu);
    glutAddMenuEntry("_________________", -1);
    glutAddMenuEntry("Exit", 27);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    init();

    glutMainLoop();
    return 0;             
}

