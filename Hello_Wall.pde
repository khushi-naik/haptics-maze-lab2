 /**
 **********************************************************************************************************************
 * @file       Haptic_Physics_Template.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 15.0;  
float             worldHeight                         = 15.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* Hello Wall*/
//FBox wall;
FPoly prize;
FCircle ball;
FCircle ball2;
FPoly wall1;
FPoly wall2;
FPoly wall3;
FPoly wall4;
FPoly wall5;
FPoly wall6;
FPoly wall7;
FPoly wall8;
FPoly wall9;
FPoly wall10;
FPoly wall11;
FPoly wall12;
FPoly wall13;
FBox end;

/* end elements definition *********************************************************************************************/
//int gameStart = 10;
boolean prizeCollected = false;


/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(600, 600);
  
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  println(Serial.list());
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);

  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  /* Wall Setup */
  //wall = new FBox(10,1);
  //wall = new FCircle(2);
  //wall.setPosition(worldWidth/2, worldHeight/2);
  //wall.setFill(0,255,0);
  //wall.setStatic(true);  
  //world.add(wall);
  
  prize = new FPoly();
  prize.vertex(10, 4);
  prize.vertex(10.5, 4);
  prize.vertex(10.5, 4.5);
  prize.vertex(11, 4.5);
  prize.vertex(11, 5);
  prize.vertex(10.5,5);
  prize.vertex(10.5, 5.5);
  prize.vertex(10, 5.5);
  prize.vertex(10, 5);
  prize.vertex(9.5, 5);
  prize.vertex(9.5, 4.5);
  prize.vertex(10, 4.5);
  prize.setFill(255,100,100);
  prize.setStaticBody(true);
  //prize.setSensor(true);
  world.add(prize);
  
  ball = new FCircle(1);
  ball.setPosition(6, 2);
  //ball.setPosition(4.5, 10);
  ball.setFill(0,200,100);
  //ball.addForce(5,0);
  //ball.setVelocity(10,0);
  //ball.setStaticBody(true);
  //ball.setHaptic(true);
  //ball.setSensor(true);
  world.add(ball);
  
  ball2 = new FCircle(0.75);
  ball2.setPosition(7, 8);
  ball2.setFill(200,0,255);
  //ball.addForce(5,0);
  ball2.setVelocity(4,0);
  //ball2.setStaticBody(true);
  //ball.setHaptic(true);
  //ball2.setSensor(true);
  world.add(ball2);
  
  wall1 = new FPoly();
  wall1.vertex(2,3);
  wall1.vertex(2,13);
  wall1.vertex(13,13);
  wall1.vertex(13,10);
  wall1.vertex(12.5, 10);
  wall1.vertex(12.5, 12.5);
  wall1.vertex(2.5, 12.5);
  wall1.vertex(2.5, 3);
  wall1.setStaticBody(true);
  wall1.setFill(0,0,255);
  //wall1.setDensity(100);
  world.add(wall1);
  
  wall2 = new FPoly();
  wall2.vertex(11, 8);
  wall2.vertex(13, 8);
  wall2.vertex(13, 3);
  wall2.vertex(6, 3);
  wall2.vertex(6, 5);
  wall2.vertex(8, 5);
  wall2.vertex(8, 4.5);
  wall2.vertex(6.5, 4.5);
  wall2.vertex(6.5, 3.5);
  wall2.vertex(12.5, 3.5);
  wall2.vertex(12.5, 7.5);
  wall2.vertex(11, 7.5);
  //wall2.setFriction(100);
  wall2.setStaticBody(true);
  wall2.setFill(0,0,255);
  wall2.setDensity(2);
  world.add(wall2);
  
  wall3 = new FPoly();
  wall3.vertex(11, 6);
  wall3.vertex(11, 10);
  wall3.vertex(9, 10);
  wall3.vertex(9, 9.5);
  wall3.vertex(10.5, 9.5);
  wall3.vertex(10.5, 6);
  wall3.setStaticBody(true);
  wall3.setFill(0,220,255);
  //wall3.setSensor(true);
  world.add(wall3);
  
  wall13 = new FPoly();
  wall13.vertex(11, 6);
  wall13.vertex(11, 10);
  wall13.vertex(9, 10);
  wall13.vertex(9, 9.5);
  wall13.vertex(10.75, 9.5);
  wall13.vertex(10.75, 6);
  wall13.setStaticBody(true);
  wall13.setFill(200,20,155);
  world.add(wall13);
  
  wall4 = new FPoly();
  wall4.vertex(11, 11);
  wall4.vertex(9, 11);
  wall4.vertex(9, 12.5);
  wall4.vertex(9.5, 12.5);
  wall4.vertex(9.5, 11.5);
  wall4.vertex(11, 11.5);
  wall4.setStaticBody(true);
  wall4.setFill(0,0,255);
  world.add(wall4);
  
  wall5 = new FPoly();
  wall5.vertex(7, 12.5);
  wall5.vertex(7, 10);
  wall5.vertex(7.5, 10);
  wall5.vertex(7.5, 12.5);
  wall5.setStaticBody(true);
  wall5.setFill(0,0,255);
  world.add(wall5);
  
  wall6 = new FPoly();
  wall6.vertex(3.5, 9);
  wall6.vertex(3.5, 10.5);
  wall6.vertex(5, 10.5);
  wall6.vertex(5, 10);
  wall6.vertex(4, 10);
  wall6.vertex(4, 9);
  wall6.setStaticBody(true);
  wall6.setFill(0,0,255);
  world.add(wall6);
  
  wall11 = new FPoly();
  wall11.vertex(5.5, 9.5);
  wall11.vertex(5.5, 7);
  wall11.vertex(6.5, 7);
  wall11.vertex(6.5, 6.5);
  wall11.vertex(5, 6.5);
  wall11.vertex(5, 9.5);
  wall11.setStaticBody(true);
  wall11.setFill(0,220,255);
  //wall11.setSensor(true);
  world.add(wall11);
  
  
  wall12 = new FPoly();
  /*wall12.vertex(5,8);
  wall12.vertex(5,9.5);
  wall12.vertex(5.5,9.5);
  wall12.vertex(5.5,8);*/
  wall12.vertex(5.25, 9.5);
  wall12.vertex(5.25, 7);
  wall12.vertex(6.5, 7);
  wall12.vertex(6.5, 6.5);
  wall12.vertex(5, 6.5);
  wall12.vertex(5, 9.5);
  wall12.setStaticBody(true);
  wall12.setFill(200,20,155);
  world.add(wall12);
  
  wall7 = new FPoly();
  wall7.vertex(2.5, 3);
  wall7.vertex(4.5, 3);
  wall7.vertex(4.5, 5);
  wall7.vertex(4, 5);
  wall7.vertex(4, 3.5);
  wall7.vertex(2.5, 3.5);
  wall7.setStaticBody(true);
  wall7.setFill(0,0,255);
  world.add(wall7);
  
  wall8 = new FPoly();
  wall8.vertex(8, 4.5);
  wall8.vertex(8.5, 4.5);
  wall8.vertex(8.5, 7);
  wall8.vertex(8, 7);
  wall8.setStaticBody(true);
  wall8.setFill(0,0,255);
  world.add(wall8);
    
  wall9 = new FPoly();
  wall9.vertex(5.5, 8);
  wall9.vertex(10.5, 8);
  wall9.vertex(10.5, 8.5);
  wall9.vertex(5.5, 8.5);
  wall9.setStaticBody(true);
  wall9.setFill(0,0,255);
  world.add(wall9);
  
  wall10 = new FPoly();
  wall10.vertex(2.5, 6);
  wall10.vertex(4, 6);
  wall10.vertex(4, 8);
  wall10.vertex(3.5, 8);
  wall10.vertex(3.5, 6.5);
  wall10.vertex(2.5, 6.5);
  wall10.setStaticBody(true);
  wall10.setFill(0,0,255);
  world.add(wall10);
  
  end = new FBox(1,1);
  end.setStaticBody(true);
  end.setFill(255,100,0);
  end.setPosition(13.5,11);
  //end.setSensor(true);
  world.add(end);
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.5)); 
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(255,200,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(rendering_force == false){
    background(255);
    textSize(13);
    fill(255, 0, 0);
    textAlign(CENTER);
    text("You must save yourself from the pink ball, collect the + prize and reach the end of the maze to win!", width/2, height-60);
    /*switch(gameStart){
      case 0:
         background(0,0,255);
         world.draw();
      case 1:
    
      default:
        background(255,255,0);
        fill(0, 255, 0);
        textAlign(CENTER);
        text("Push the ball or square to the red circle", width/2, 60);
    }*/
  
    world.draw();
  }
   
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-7); 
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(20000); //
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
    
    if(ball2.isTouchingBody(wall11)){
      ball2.setVelocity(4,0);
    }
    if(ball2.isTouchingBody(wall3)){
      ball2.setVelocity(-4,0);
    }
    if(ball.isTouchingBody(ball2)){
      ball.setPosition(7,2);
      prizeCollected = false;
      //ball.setVelocity(0,0);
      ball2.setVelocity(4,0);
    }
    if(ball.isTouchingBody(prize)){
      prizeCollected = true;
      ball.setFill(0,40,0);
      ball.setPosition(4.5,9);
      ball.setSize(0.80);
    }
    if(ball.isTouchingBody(end) && prizeCollected){
      //if(prizeCollected){
        fill(0, 200, 0);
        textSize(20);
        textAlign(CENTER);
        text("YAY!!!", width/2, 60);
        fill(0,0,255);
      //}
      //ball.setPosition(7,2);
      //prizeCollected = false;
    }
   
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
