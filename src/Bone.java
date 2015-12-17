import java.util.*;

import remixlab.bias.event.*;
import remixlab.proscene.*;
import remixlab.dandelion.geom.*;
import remixlab.dandelion.core.*;

//the angle of a bone is the angle btwn the bone and its parent

public class Bone extends InteractiveFrame{
  float radius = 10;
  int colour = -1;
  Skeleton skeleton;
  ArrayList<Bone> children = new ArrayList<Bone>();
  Bone parent = null;  
  Joint joint_axis_x = null;
  Joint joint_axis_y = null;
  Joint joint_axis_z = null;
  Vec model_pos;
  Vec prev_angle = new Vec(0,0,0);
  Vec prev_pos = new Vec(0,0,0);  

  float PI = (float) Math.PI;

  public Bone(Scene sc){
    super(sc);
    skeleton = new Skeleton(sc); 
    joint_axis_x = new Joint();
    joint_axis_y = new Joint();
    joint_axis_z = new Joint();
    colour = sc.pApplet().color(0,255,0);
  }
  public Bone(Scene sc, Bone b, boolean isChild){
    super(sc);
    colour = sc.pApplet().color(0,255,0);    
    joint_axis_x = new Joint();
    joint_axis_y = new Joint();
    joint_axis_z = new Joint();
    if(!isChild){
      parent = b;
      b.children.add(this);
    }
    else{
      children.add(b);
      b.parent = this;
    } 
  }

  public Bone(Scene sc, Bone p, Bone c){
    super(sc);
    joint_axis_x = new Joint();
    joint_axis_y = new Joint();
    joint_axis_z = new Joint();
    children.add(c); parent = p;
    p.children.add(this);
  }  

  void updateMainFrame(Frame frame){
    setReferenceFrame(frame);
  }
  
  public void createBone(float pos_x, float pos_y, float pos_z, Skeleton sk){
      //Set initial orientation
	  Vec angle = new Vec(0,0,0);
      //update();
      //add to an empty skeleton
      sk.bones.add(this);
      //relate to a new unconstrained Joint
      joint_axis_x = new Joint((float)-1.*PI,PI, angle.x());
      joint_axis_y = new Joint((float)-1.*PI,PI, angle.y());
      joint_axis_z = new Joint((float)-1.*PI,PI, angle.z());
      //relate to parent frame
      updateMainFrame(skeleton.frame);
      this.translate(pos_x, pos_y, pos_z);
      //relate the skeleton
      skeleton = sk;
      //keep a track of all the bones in the scene
      Kinematics.bones.add(this);
  }

  //Update the angle of the bone given a translation on its final point
  public void updateAngle(){
	Vec aux = parent == null ? new Vec(0,0,0) : parent.inverseCoordinatesOf(new Vec(0,0,0));
	Vec diff = Vec.subtract(inverseCoordinatesOf(new Vec(0,0,0)), aux);    
    Vec v1 = Vec.subtract(inverseCoordinatesOf(new Vec(0,0,0)), aux);    
    Vec v2 = new Vec(1,0,0);    
    Vec v2Xv1 = new Vec();
    Vec.cross(v2, v1, v2Xv1);
    float ang = Vec.angleBetween(v1, v2);
    Quat q = new Quat(v2Xv1, ang);
    q.normalize();
    Vec angle = q.eulerAngles();
    joint_axis_x.angle = angle.x() >= -1* PI && angle.x() <= PI ? angle.x() : joint_axis_x.angle;
    joint_axis_y.angle = angle.y() >= -1* PI && angle.y() <= PI ? angle.y() : joint_axis_y.angle;    
    joint_axis_z.angle = angle.z() >= -1* PI && angle.z() <= PI ? angle.z() : joint_axis_z.angle;
  }

  public float[] getAngleFromPoss(Vec pos){
    float[] angle = new float[3];
    Vec aux = parent == null ? new Vec(0,0,0) : parent.inverseCoordinatesOf(new Vec(0,0,0));
    Vec diff = Vec.subtract(inverseCoordinatesOf(pos), aux);    
    Quat q;
    float angle_z = (float)Math.atan2(diff.y(), diff.x());
    float angle_x = (float)Math.atan2(diff.y(), diff.z());
    float angle_y = (float)Math.atan2(diff.z(), diff.x());
	angle[0] = angle_x >= -1* PI && angle_x <= PI ? angle_x : joint_axis_x.angle;
	angle[1] = angle_y >= -1* PI && angle_y <= PI ? angle_y : joint_axis_y.angle;    
	angle[2] = angle_z >= -1* PI && angle_z <= PI ? angle_z : joint_axis_z.angle;
	return angle;
  }
  
  public Quat getAngleFromPos(Vec pos){
	  Vec aux = parent == null ? new Vec(0,0,0) : parent.inverseCoordinatesOf(new Vec(0,0,0));
      Vec v1 = Vec.subtract(inverseCoordinatesOf(new Vec(0,0,0)), aux);    
      Vec v2 = Vec.subtract(inverseCoordinatesOf(pos), aux);    
      Vec v2Xv1 = new Vec();
      Vec.cross(v1, v2, v2Xv1);
      float ang = Vec.angleBetween(v1, v2);
	  Quat q = new Quat(v2Xv1, ang);
	  return q;
  }
  
  
  public void angleToPos(){
	  System.out.println("---> angle x :" +  joint_axis_x.angle + " y : " + joint_axis_y.angle
			  + " z : " + joint_axis_z.angle + " pos " + position());
    Vec diff = translation();    
    Vec diff_xy = Vec.projectVectorOnPlane(diff, new Vec(0,0,1)); 
    Vec diff_xz = Vec.projectVectorOnPlane(diff, new Vec(0,1,0)); 
    Vec diff_yz = Vec.projectVectorOnPlane(diff, new Vec(1,0,0));     
    float dim_z = diff_xy.magnitude();
    float dim_x = diff_yz.magnitude();
    float dim_y = diff_xz.magnitude();

    this.setTranslation(dim_z*(float)Math.cos(joint_axis_z.angle), 
    					dim_z*(float)Math.sin(joint_axis_z.angle),
    					dim_y*(float)Math.sin(joint_axis_y.angle));
    System.out.println("ax : " + joint_axis_x.angle + " " + "ay : " + joint_axis_y.angle
    		+ "az : " + joint_axis_z.angle + " pos " + position());
  }

  public void angleToPos(float mag){
	  Quat q = new Quat(joint_axis_x.angle, joint_axis_y.angle, joint_axis_z.angle);
	  Vec pos = translation().get();
	  pos.normalize(pos);
	  pos = q.rotate(pos);
	  pos.multiply(mag);
	  this.setTranslation(pos);
	  System.out.println("ax : " + joint_axis_x.angle + " " + "ay : " + joint_axis_y.angle
    		+ "az : " + joint_axis_z.angle + " pos " + position());
  }
  

  //get a vector translated in X according to the projection view
  public Vec getVecTranslatedX(float delta){
	  Vec trans = screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -delta : delta, 0, 0),
			  translationSensitivity()));
	  return trans;
  }
  
  //add a new bone to the skeleton 
  //modify the angle of the Joint of the related bone
  //TO DO - Add functionality to the change of hierarchy
  public void addBone(boolean asParent, Vec place){
      //create a bone as child
      Quat q = getAngleFromPos(place);
	  q.normalize();
	  Vec angle = q.eulerAngles();

	  Bone b = new Bone((Scene) scene, this, false);
      b.joint_axis_x = new Joint((float)-1.* PI,PI, angle.x());
      b.joint_axis_y = new Joint((float)-1.* PI,PI, angle.y());
      b.joint_axis_z = new Joint((float)-1.* PI,PI, angle.z());
      //Apply transformations
      b.updateMainFrame(this);
      b.translate(place);
      //Relate the new bone with the corresponding lists
      b.skeleton = skeleton;
      skeleton.bones.add(b);
      Kinematics.bones.add(b);      
  }

  public Bone getRoot(){
	  if(this.parent == null) return this;
	  return this.parent.getRoot();
  }
  
  public ArrayList<Bone> getChildrenWS(){
	  ArrayList<Bone> bones = new ArrayList<Bone>();
	  ArrayList<Bone> queue = new ArrayList<Bone>();
	  queue.add(this);
	  while(!queue.isEmpty()){
		  Bone current = queue.remove(0);
		  for(Bone b : current.children){
			  queue.add(b);
			  bones.add(b);
		  }
	  }
	  return bones;
  }
 
  public boolean isAncester(Bone cur, Bone p){
    if(cur.parent == null) return false;
    if(cur.parent == p) return true;
    return isAncester(cur.parent, p);
  }

  public boolean isAncester(Bone p){
    return isAncester(this, p);
  }
  
  public float getCorrectAngle(float angle){
    float a = angle;
    //get Angle btwn -PI and PI
    while(a < -PI) a += 2*PI;
    while(a >  PI) a -= 2*PI;
    return a;
  }

  @Override
  public boolean checkIfGrabsInput(float x, float y){
    float threshold = radius;
    Vec proj = scene().eye().projectedCoordinatesOf(position());
    if((Math.abs(x - proj.vec[0]) < threshold) && (Math.abs(y - proj.vec[1]) < threshold)){
      return true;      
    }
    return false;
  }

  //scroll action will increase or decrease the detail of the shape
  @Override
  public void performCustomAction(DOF1Event event) {   
      gestureScale(event, wheelSensitivity());
  }

  
  @Override
  public void performCustomAction(ClickEvent event) {
    if(Kinematics.add_bone){
      if(event.id() == 39){
        Kinematics.removeSkeleton();
        return;
      }
      else{
    	Vec trans = getVecTranslatedX(50);  
        addBone(false, trans);
      }
    } 
    else{
      //change color and highlight as selected
      if(Kinematics.last_selected_bone != null){
    	  Kinematics.last_selected_bone.colour = ((Scene) scene).pApplet().color(0,255,0);
      }
      Kinematics.last_selected_bone = this;
      //update Joint control
      Kinematics.control_frame.setupControlShape(Kinematics.current_axis);
      colour = ((Scene) scene).pApplet().color(0,0,255);
    }
  }
  
  @Override
  public void performCustomAction(DOF2Event event) {
	if(event.id() == 39 || parent == null){  
		if(Kinematics.add_bone){
		  translate(screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -event.dx() : event.dx(),
		      (scene.isRightHanded() ^ isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), translationSensitivity())));    
		  skeleton.updateAngles();
		  return;
		}
		//Translate the skeleton Frame
		skeleton.frame.translate(skeleton.frame.screenToVec(Vec.multiply(new Vec(skeleton.frame.isEyeFrame() ? -event.dx() : event.dx(),
		    (scene.isRightHanded() ^ skeleton.frame.isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), skeleton.frame.translationSensitivity())));    

	}
	else if(event.id() == 37){ 
		rotate(event);
		if(!Kinematics.add_bone) Kinematics.control_frame.setupControlShape(Kinematics.current_axis);
	}
  }
  
  //Simulate rotation
  public void rotate(DOF2Event event){
	  Vec delta =  screenToVec(Vec.multiply(new Vec(isEyeFrame() ? -event.dx() : event.dx(),
	          (scene.isRightHanded() ^ isEyeFrame()) ? -event.dy() : event.dy(), 0.0f), translationSensitivity()));
	  Quat q = getAngleFromPos(delta);
	  q.normalize();
	  Vec angle = q.eulerAngles();
	  float mag = translation().magnitude();
	  joint_axis_x.angle = angle.x() >= -1* PI && angle.x() <= PI ? angle.x() : joint_axis_x.angle;
	  joint_axis_y.angle = angle.y() >= -1* PI && angle.y() <= PI ? angle.y() : joint_axis_y.angle;
	  joint_axis_z.angle = angle.z() >= -1* PI && angle.z() <= PI ? angle.z() : joint_axis_z.angle;
	  angleToPos(mag);
	  skeleton.updateAngles();	  
  }
  
  //CHECK SCENE
  public void drawShape(){
      Vec aux = parent == null ? null : parent.inverseCoordinatesOf(new Vec(0,0,0));
      Vec aux2 = inverseCoordinatesOf(new Vec(0,0,0));
      Kinematics.main_graphics.pushStyle();
      if(aux != null){
    	  Kinematics.main_graphics.stroke(255,255,255);        
    	  Kinematics.main_graphics.line(aux2.x(),aux2.y(),aux2.z(),aux.x(), aux.y(), aux.z());
      }
      Kinematics.main_graphics.strokeWeight(radius);
      Kinematics.main_graphics.stroke(colour);
      Kinematics.main_graphics.point(aux2.x(),aux2.y(),aux2.z());
      Kinematics.main_graphics.popStyle();
  }
}
