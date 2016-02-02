import java.util.*;

import processing.core.*;
import remixlab.bias.event.DOF2Event;
import remixlab.dandelion.geom.*;
import remixlab.proscene.*;

//Joint Interactive Class
public class JointControl extends InteractiveFrame{
  float PI = (float) Math.PI;    

  public class Pointer{
    PShape shape;
    Vec position;
    float value;
    float radius;
    float size;
        
    public boolean isInside(float x, float y){
      if(Math.abs(x - position.x()) <= size && Math.abs(y - position.y()) <= size)
        return true;
      return false;      
    }
    
    public float getDist(float x, float y){
      float x_dist = x - position.x();
      x_dist = x_dist * x_dist; 
      float y_dist = y - position.y();
      y_dist = y_dist * y_dist;
      return (float)Math.sqrt(x_dist + y_dist);      
    }
    
  }  

  ArrayList<Pointer> pointers = new ArrayList<Pointer>();  
  Joint current_Joint; //This is an independent scene which related Joint is gonna be the last chosen
  float main_radius = 100;
  
  public void setupProfile(){
	  this.setMotionBinding(MouseAgent.LEFT_ID, "movePointer");
	  this.setMotionBinding(MouseAgent.RIGHT_ID, "movePointer");
  }  
  
  public JointControl(Scene sc){
    super(sc);
    float[] values = {(float)-1.*PI,0, 0, PI};
    setupControlShape(values);
    setupProfile();
  }
  
  
  public Pointer setupPointer(float r, float v){
    //create the shape
    Pointer p = new Pointer();
    p.value = v;
    p.radius = r;
    p.position = getPosition(v,r);
    float c_r = 15;
    p.size = c_r;     
    scene().pApplet();
	PShape s = scene().pApplet().
    		createShape(PConstants.ELLIPSE, p.position.x() - c_r*(float)(1./2.), 
    				p.position.y() - c_r*(float)(1./2.), c_r,c_r);
    s.fill(255,255,255);
    p.shape = s;
    return p;
  }
  
  public Vec getPosition(float v, float r){
    float v_to_rad = v;
    return new Vec(r*(float)Math.cos(v_to_rad), r*(float)Math.sin(v_to_rad));
  }
  
  public void setupControlShape(int axis){
    if(Kinematics.last_selected_bone == null) return;
    Joint j = null;
    if(axis == 0) j = Kinematics.last_selected_bone.joint_axis_x;
    else if(axis == 1) j = Kinematics.last_selected_bone.joint_axis_y;
    else if(axis == 2) j = Kinematics.last_selected_bone.joint_axis_z;
    else return;
    updatePointer(pointers.get(0), j.min_angle);  
    updatePointer(pointers.get(1), j.stable_angle);  
    updatePointer(pointers.get(2), j.angle);  
    updatePointer(pointers.get(3), j.max_angle);  
  }
  
  public void setupControlShape(float[] values){
    float radius_step = main_radius*(float)1./(values.length + 1);
    float rad = (float)0.;
    PShape p = scene().pApplet().createShape(PConstants.GROUP);
    for(int i = 0; i < values.length; i++){
      PShape circ = scene().pApplet().createShape(PConstants.ELLIPSE,-main_radius + radius_step*i,
    		  - main_radius + radius_step*i, 2*(main_radius - radius_step*i),2*(main_radius - radius_step*i));
      circ.setFill(scene().pApplet().color((int)(Math.random()*255),
    		  (int)(Math.random()*255),(int)(Math.random()*255)));            
      p.addChild(circ);
    }
    rad= radius_step;
    for(int i = 0; i < values.length; i++){
      rad += radius_step;
      //create the appropiate pointer
      Pointer pointer = setupPointer(rad, values[i]);
      pointers.add(pointer);
      p.addChild(pointer.shape);
    }    
    setShape(p); 
  }
  
  public int getNearestPointer(float x, float y){
    //float min_dist = 9999;
    //int pos = -1;
    //int cur = 0;
    for(int i = 0; i < pointers.size(); i++){
      if(pointers.get(i).isInside(x,y)) return i;
    }
    return -1;
    /*
    for(Pointer p : pointers){
      if(p.isInside(x,y)) return cur;
      float dist = p.getDist(x,y);
      if(dist < min_dist){
        min_dist = dist;
        pos = cur;
      }
      cur++;
    }
    return pos;*/
  }
  
  

  public void movePointer(DOF2Event event) {    
      if(Kinematics.last_selected_bone == null) return;
      Vec point_world = scene().eye().unprojectedCoordinatesOf(new Vec(event.x(), event.y()));
      Vec point_shape = coordinatesOf(point_world);
      int p = getNearestPointer(point_shape.x(), point_shape.y());      
      if(p == -1) return;
      Pointer pointer = pointers.get(p);      
      //move the angle to the place where the mouse is
      float new_angle = (float)Math.atan2(point_shape.y(), point_shape.x());
      System.out.println("-- new_angle -- " + new_angle);      
      updatePointer(pointer, new_angle);
      //update current Joint value
      if(Kinematics.current_axis == 0) current_Joint = Kinematics.last_selected_bone.joint_axis_x; 
      else if(Kinematics.current_axis == 1) current_Joint = Kinematics.last_selected_bone.joint_axis_y; 
      else if(Kinematics.current_axis == 2) current_Joint = Kinematics.last_selected_bone.joint_axis_z; 

      updateJointValue(p, pointer);
  }
  
  public void updatePointer(Pointer p, float new_angle){
    //reset
    p.shape.translate(-p.position.x() + p.size*(float)(1./2.), -p.position.y() + p.size*(float)(1./2.));             
    p.position = getPosition(new_angle,p.radius);
    p.value = new_angle;
    p.shape.translate(p.position.x() - p.size*(float)(1./2.), p.position.y() - p.size*(float)(1./2.));          
  }
  
  public void updateJointValue(int i, Pointer p){
    if(Kinematics.last_selected_bone.parent == null) return;
    if(i == 0){
      current_Joint.setMin(p.value); 
    }
    else if(i == 1){
      current_Joint.setStable(p.value); 
    }
    else if(i == 2){
      current_Joint.setAngle(p.value); 
    }
    else if(i == 3){
      current_Joint.setMax(p.value); 
    }   
    Kinematics.last_selected_bone.angleToPos();//setOrientation(new Quat(new Vec(0, 0, 1), p.value));
    Kinematics.last_selected_bone.skeleton.updateAngles();
    setupControlShape(Kinematics.current_axis);
    System.out.println("Joint min: " + current_Joint.min_angle);
    System.out.println("Joint max: " + current_Joint.max_angle);
    System.out.println("Joint ang: " + current_Joint.angle);
    System.out.println("Joint sta: " + current_Joint.stable_angle);
  }
} 