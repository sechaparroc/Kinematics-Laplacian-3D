import processing.core.*;

import java.util.*;

//-----------------------------------------------
//Proscene
//Use InteractiveModelFrame and override actions
import remixlab.proscene.*;
import remixlab.dandelion.constraint.AxisPlaneConstraint;
import remixlab.dandelion.constraint.Constraint;
import remixlab.dandelion.constraint.EyeConstraint;
import remixlab.dandelion.constraint.AxisPlaneConstraint.Type;
import remixlab.dandelion.core.Camera;
import remixlab.dandelion.geom.*;

/*
Sebastian Chaparro
December 3 2015
*/

public class Kinematics extends PApplet{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	//Processing variables
	//PGRAPHICS
	public static PGraphics main_graphics;
	public static PGraphics aux_graphics;
	public static PGraphics control_graphics;

	//SCENES------------------------------------------
	/*Basically 2 scenes are required: one to draw the main Object,
	the other one to control the world as an sphere */
	static Scene main_scene;
	static Scene aux_scene;
	static int TOKEN_W = 5;
	static int TOKEN_H = 10;
	static int TOKEN_DIM = 1;
	static Bone last_selected_bone = null;
	static JointControl control_frame;
	static final int all_width = 800;
	static final int all_height = 600;
	boolean showAid = false;
	final int aux_pos_x = all_width-all_width/4;
	final int aux_pos_y = all_height-all_height/3;
	//Some models
	static Utilities.CustomModelFrame original_fig;
	static PShape figure;
	
	public static Vec r_center;
	public static Vec[] r_bounds;	

	//This is gonna be a serie of tokens to manipulate
	static ArrayList<Skeleton> skeletons = new ArrayList<Skeleton>();
	static ArrayList<Bone> bones = new ArrayList<Bone>();
	
	public static boolean laplacian = true;
	public static boolean fill = false;
	public static boolean bounding_rect = true;
	public static int current_axis = 0;
	
	public void setup(){
	  size(all_width, all_height, P3D);
	  main_graphics = createGraphics(all_width,all_height,P3D);
	  main_scene = new Scene(this, main_graphics);
	  aux_graphics = createGraphics(all_width/4,all_height/3,P3D);
	  aux_scene = new Scene(this, aux_graphics, aux_pos_x, aux_pos_y);    
	  main_scene.setAxesVisualHint(true);
	  main_scene.setGridVisualHint(true);
	  aux_scene.setAxesVisualHint(true);
	  aux_scene.setGridVisualHint(false);
	  main_scene.setRadius(50);
	  main_scene.camera().setType(Camera.Type.ORTHOGRAPHIC);
	  aux_scene.camera().setType(Camera.Type.ORTHOGRAPHIC);
	  AxisPlaneConstraint constrain = new EyeConstraint(aux_scene.eye());
	  constrain.setRotationConstraint(Type.FORBIDDEN, new Vec(0,0,1));
	  aux_scene.eye().frame().setConstraint(constrain);
	  control_frame = new JointControl(aux_scene);
	  //set up the mesh to load
	  figure = loadShape("human2.obj");
	  original_fig = new Utilities.CustomModelFrame(main_scene, figure);
	  //original_fig.translate(-50,50,0);
	  original_fig.scale(3f);
	  //original_fig.rotate(0,0,PI,0);
	  //get bounding rect center
	  r_bounds = Utilities.getCube(figure);
	  r_center = new Vec((r_bounds[0].x() + r_bounds[1].x())/2.f, (r_bounds[0].y() + r_bounds[1].y())/2.f, (r_bounds[0].z() + r_bounds[1].z())/2.f);
	  println(figure.getChildCount());
	  main_graphics.noSmooth();  
	  Utilities.fillWithColor(original_fig, figure, color(255,0,0,200), fill);
	  //add a initial point in the xy plane
	  Vec point_world = new Vec((r_bounds[0].x() + r_bounds[1].x())/2.f, (r_bounds[0].y() + r_bounds[1].y())/2.f, Math.max(r_bounds[0].z(), r_bounds[1].z()));
	  point_world = original_fig.inverseCoordinatesOf(point_world);
	  skeletons.add(new Skeleton(main_scene,point_world.x(), point_world.y(),point_world.z()));
	}

	public void draw(){
	  handleAgents();  
	  main_graphics.beginDraw();
	  main_scene.beginDraw();
	  main_graphics.background(0);
	  original_fig.draw();
	  if(bounding_rect) Utilities.drawCube(original_fig, main_graphics);
	  drawBones();
	  IKinematics.drawAnchors(main_scene, original_fig);	  
	  main_scene.endDraw();
	  main_graphics.endDraw();    
	 image(main_graphics, main_scene.originCorner().x(), main_scene.originCorner().y());
	  if (showAid) {
	    aux_graphics.beginDraw();
	    aux_scene.beginDraw();
	    aux_graphics.background(125, 125, 125, 125);
	    aux_scene.drawFrames();
	    aux_scene.endDraw();
	    aux_graphics.endDraw();    
	    image(aux_graphics, aux_scene.originCorner().x(), aux_scene.originCorner().y());
	  }
	}

	int drag_mode = -1;
	void handleAgents() {
	  aux_scene.disableMotionAgent();
	  aux_scene.disableKeyboardAgent();
	  main_scene.disableMotionAgent();
	  main_scene.disableKeyboardAgent();
	  if ((mouseX >= all_width - aux_scene.width()) && mouseY >= all_height - aux_scene.height()) {
	    aux_scene.enableMotionAgent();
	    aux_scene.enableKeyboardAgent();
	  }else if(drag_mode == -1) {
	    main_scene.enableMotionAgent();
	    main_scene.enableKeyboardAgent();
	  }
	}	
	
	//find the point in the bounding box usingpixel under point
	public Vec getIntersectionPoint(float x, float y){
	  Vec pup = main_scene.pointUnderPixel(new Point(x, y));
	  if(pup == null) return null; 
	  return original_fig.coordinatesOf(pup);
	}

	//HANDLE SOME MOUSE AND KEYBOARD ACTIONS
	static boolean add_bone = false;
	//Bone.add_bone = false;
	public void mousePressed(){
	  /*for(Bone f : bones){
	    if(f.grabsInput(main_scene.motionAgent())){
	      last_selected_bone = f;          
	      return;
	    }
	  }*/
	  if(add_bone){
	    if ((mouseX >= all_width - aux_scene.width()) && mouseY >= all_height - aux_scene.height()) return;
	    if(mouseButton == LEFT){
	      for(Bone f : bones){
	        if(f.checkIfGrabsInput(mouseX,mouseY)){
	          return;
	        }
	      }
	      Vec point_world = getIntersectionPoint(mouseX, mouseY);
	      if(point_world == null) return;
	      point_world = original_fig.inverseCoordinatesOf(point_world);
	      //skeletons.add(new Skeleton(main_scene,point_world.x(), point_world.y(),point_world.z()));
	    }
	    if(mouseButton == RIGHT){
	      //removeSkeleton();
	    }
	  }
	}
	boolean temp = false;
	static boolean enable_ef = false;
	static boolean enable_mod_ef = false;
	static boolean enable_mod_rad = true;
	static boolean enable_mod_w = false;
	
	public void keyPressed(){  
	  //if(key == 'x' || key== 'X'){
	  //  temp = !temp;
	  //  if(temp) main_scene.removeModel(original_fig);
	  //  else main_scene.addModel(original_fig);
	  //}
	  if (key == 'r' || key=='R'){
	    bounding_rect = !bounding_rect;
	  }
	  
	  if(key == 'x' || key=='X'){
		  fill = !fill;
		  Utilities.fillWithColor(original_fig, figure, color(255,0,0,200), fill);
	  }
		
	  if(key=='b' || key=='B'){
	    add_bone = !add_bone;
	    if(last_selected_bone != null){
	      last_selected_bone.selected = false;
	    }
	    last_selected_bone = null;
	  }
	  
	  if(key == 'z' || key == 'Z'){
	    if(last_selected_bone != null){
	    	IKinematics.execSkinning(original_fig,last_selected_bone.skeleton.bones);
	    	System.out.println("sale");
	    }
	  }
	  if(key == 'l' || key == 'L'){
		  IKinematics.applyTransformations(original_fig);
	  }
	  if(key == '4'){
		  current_axis = 0;
	  }
	  if(key == '5'){
		  current_axis = 1;
	  }
	  if(key == '6'){
		  current_axis = 2;
	  }
	  if(key == 'n' || key == 'N'){
		  enable_ef = !enable_ef;
	  }
	  if(key == 'm' || key == 'M'){
		  IKinematics.executeDLS();
	  }
	  if(key == '1'){
		  enable_mod_ef = true;
		  enable_mod_rad = false;
		  enable_mod_w = false;		  
	  }
	  if(key == '2'){
		  enable_mod_ef = false;
		  enable_mod_rad = true;
		  enable_mod_w = false;		  
	  }
	  if(key == '3'){
		  enable_mod_ef = false;
		  enable_mod_rad = false;
		  enable_mod_w = true;		  
	  }
	}

	//change by a deph search
	void drawBones(){
	  for(Skeleton s : skeletons){
	    drawBones(s.frame);
	  }
	}

	void drawBones(Bone root){
	    main_scene.pg().pushMatrix();
	    //root.applyWorldTransformation();
	    main_scene.applyWorldTransformation(root);
	    //main_scene.drawAxes(40);    
	    main_scene.pg().popMatrix();
	    root.drawShape();    
	    for(Bone child : root.children) drawBones(child);
	}

	//Token mods:
	public static void removeSkeleton(){
	  if(skeletons.size() == 1) return;
	  Skeleton sk = null;
	  for (int i = 0; i < bones.size(); i++){ 
	    if (bones.get(i).grabsInput(main_scene.motionAgent())){
	      sk = bones.get(i).skeleton;
	      break;
	    }
	  }
	  if(sk == null) return;
	  for(Bone b : sk.bones){
	    //main_scene.removeModel(b);
	    bones.remove(b);
	  }
	  skeletons.remove(sk);  
	  last_selected_bone = null;
	}	
}