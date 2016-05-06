import java.util.*;

import org.la4j.Matrix;
import org.la4j.LinearAlgebra.InverterFactory;
import org.la4j.matrix.dense.Basic2DMatrix;
import org.la4j.vector.dense.BasicVector;

import processing.core.*;
import remixlab.dandelion.geom.*;
import remixlab.proscene.Scene;


public class IKinematics {
	/*Contains methods to apply IK
	more info look at 
	http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
	http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf
	*/
	static LaplacianDeformation laplacian;
	static Skinning skinning;
	static boolean debug = false;
	static float d_max = -1;
	static float lambda = 0.1f;
	//public  class InverseKinematics{
	  //s : end effector position
	  //theta: rotation joint (1DOF) for each axis
	  //s(theta): end effectors are functions of theta
	  //v: Vector pointing to the axis of rot
	  //P: position of the join
	  //D(S)/D(theta) =  Vj X (Si - Pj)
	  //Vj for 2d is (0,0,1), for 3D could take 3 values
	  public static Vec calculateDiff(Vec Pj, Vec Si, Vec Vj){
	    Vec sub = new Vec(0,0,0); 
	    Vec.subtract(Si, Pj, sub);
	    Vec diff = new Vec(0,0,0);
	    Vec.cross(Vj, sub, diff);
	    return diff;
	  }
	  
	  //Execute Damped Least Squares Algorithm
	  //Jacobian is calculated each time this method is called, assuming that it is not to costly
	  public static void executeDLS(){
		  //get all end effectors
	    for(Skeleton s : Kinematics.skeletons){
	    	ArrayList<Bone> end_effectors = new ArrayList<Bone>();
	    	Bone root = s.frame.getRoot();
	    	for(Bone bone : root.getChildrenWS()){
	    		//add all end effectors
	    		if(bone.is_end_effector)  end_effectors.add(bone);
	    	}
	    	if(end_effectors.size() == 0) continue;
	    	//Get the jacobian
	    	double[][] jacobian = calculateJacobian(s.bones, end_effectors);

	    	//convert jacobian into full row rank
	    	ArrayList<Integer> deleted_idx = new ArrayList<Integer>();
	    	for(int r = 0; r < jacobian.length; r++){
	    		float sum = 0;
		    	for(int c = 0; c < jacobian[0].length; c++){
		    		sum += jacobian[r][c]; 
		    	}
		    	if(sum == 0) deleted_idx.add(r);
	    	}
	    	if(!deleted_idx.isEmpty()){
		    	double[][] jacobian_full_rank = new double[jacobian.length - deleted_idx.size()][jacobian[0].length];
		    	int pos = 0;
		    	for(int r = 0; r < jacobian.length; r++){
		    		if(pos < deleted_idx.size()){
			    		if(r == deleted_idx.get(pos)){
			    			pos++; continue;
			    		}
		    		}
			    	for(int c = 0; c < jacobian[0].length; c++){
			    		jacobian_full_rank[r - pos][c] = jacobian[r][c]; 
			    	}
		    	}
		    	jacobian = jacobian_full_rank;
	    	}
	    	
	    	//Get Delta theta
	    	Basic2DMatrix J = new Basic2DMatrix(jacobian);
	    	System.out.println("JACO : \n\n\n\n" + J.toString());
	    	Basic2DMatrix J_T = (Basic2DMatrix) J.transpose();	 
	    	
	    	//use a weigthed approach
	    	double[][] W_mat = new double[s.bones.size()*3][s.bones.size()*3];
	    	for(int i = 0; i < W_mat.length; i++){
	    		W_mat[i][i] = 1./(s.bones.get(i/3).weight);
	    	}
	    	//Basic2DMatrix W_i = new Basic2DMatrix(W_mat);
	    	//J_T = (Basic2DMatrix) W_i.multiply(J_T);
	    	Basic2DMatrix JJ_T = (Basic2DMatrix) J.multiply(J_T);
	    	//construct identity
	    	Basic2DMatrix lambda_2_diag = Basic2DMatrix.identity(J.rows());
	    	lambda_2_diag = (Basic2DMatrix) lambda_2_diag.multiply(lambda*lambda);
	    	Matrix term = JJ_T.add(lambda_2_diag);
	    	term = term.withInverter(InverterFactory.GAUSS_JORDAN).inverse();
	    	term = J_T.multiply(term);
	    	System.out.println("TERM : " + term.toString());
	    	Vec[] e_vec_t, e_vec;
	    	double e_t = 999, prev_e_t = 999;	    	
	    	int it = 0;
	    	//clamp each e_i
	    	setDMax(s.bones);
	    	//Get error (e = t - s)
	    	e_vec_t = calculateError(end_effectors);
	    	e_vec = new Vec[e_vec_t.length];
	    	for(int i = 0; i < e_vec.length; i++){
	    		e_vec[i] = clampMag(e_vec_t[i], d_max);
	    	}
	    	//change the way that e is stored
	    	double[] e = new double[e_vec.length*3 - deleted_idx.size()];
	    	int pos = 0;
	    	for(int i = 0; i < e_vec.length*3;){
	    		if(pos < deleted_idx.size()){
		    		if(i == deleted_idx.get(pos)){
		    			pos++;
		    		}
	    		}	    		
	    		else e[i-pos] = e_vec[i/3].x();
	    		i++;
	    		if(pos < deleted_idx.size()){
		    		if(i == deleted_idx.get(pos)){
		    			pos++;
		    		}
	    		}
	    		else e[i-pos] = e_vec[i/3].y();
	    		i++;
	    		if(pos < deleted_idx.size()){
		    		if(i == deleted_idx.get(pos)){
		    			pos++;
		    		}
	    		}
	    		else e[i-pos] = e_vec[i/3].z();
	    		i++;	    		
	    	}
	    	BasicVector e_v = new BasicVector(e);
	    	System.out.println("error _v -----------------: + \n" + e_v.toString());
	    	
	    	//Apply Weights
	    	double[] w = new double[s.bones.size()];
	    	double w_t = 0;
	    	for(int i = 0; i < w.length; i++){
	    		w[i] = 1./s.bones.get(i).weight;
	    		w_t += w[i];		
	    	}
	    	for(int i = 0; i < w.length; i++)w[i] = w[i]*1./w_t;
	    	//Basic2DMatrix w_vec = new Basic2DMatrix(w);
	    	BasicVector delta_theta = (BasicVector) term.multiply(e_v);
	    	//System.out.println("delta theta -----------------: + \n" + delta_theta.toString());
	    	//delta_theta = (BasicVector) w_vec.multiply(delta_theta);
	    	System.out.println("delta theta -----------------: + \n" + delta_theta.toString());
	    	//sum delta 
	    	applyDeltaTetha(s.bones, delta_theta.toArray(), w);
	    	e_t = 0;
	    	for(int i = 0; i < e_vec_t.length; i++){
	    		e_t += e_vec_t[i].magnitude()*e_vec_t[i].magnitude();
	    	}
	    	e_t = Math.sqrt(e_t);
	    	System.out.println("Error mag : " + e_t);
	    }
	  }
	  
	  
	  //sum delta
	  public static void applyDeltaTetha(ArrayList<Bone> joints, double[] delta, double[] w){
		  int i = 0;
		  System.out.println("delta size : " + delta.length);
		  System.out.println("num joints : " + joints.size());
		  for(Bone theta : joints){
			  float delta_angle = (float)delta[i];	  
			  if(theta.parent == null){
				  System.out.println("cambio: " + delta_angle);
				  i+=3;
				  continue;
			  }
			  System.out.println("cambio x : " + delta[i]);
			  System.out.println("cambio y : " + delta[i+1]);
			  System.out.println("cambio z : " + delta[i+2]);
			  Quat dx = new Quat(new Vec(1,0,0), (float)delta[i]);dx.normalize();
			  Quat dy = new Quat(new Vec(0,1,0), (float)delta[i+1]);dy.normalize();
			  Quat dz = new Quat(new Vec(0,0,1), (float)delta[i+2]);dz.normalize();			 
			  dx.compose(dy);dx.compose(dz);dx.normalize();
			  float weighted_angle = dx.angle()*(float)w[i/3];
			  dx = new Quat(dx.axis(),weighted_angle);
			  Quat cur = new Quat(theta.joint_axis_x.angle, theta.joint_axis_y.angle, theta.joint_axis_z.angle);
			  System.out.println("antes de : " + cur.eulerAngles());
			  theta.applyRotation(dx);
			  cur = new Quat(theta.joint_axis_x.angle, theta.joint_axis_y.angle, theta.joint_axis_z.angle);			  
			  System.out.println("despues de : " + cur.eulerAngles());
			  //float new_angle = theta.joint.angle + delta_angle;
			  //new_angle = new_angle > theta.joint.max_angle ? theta.joint.max_angle : new_angle;
			  //new_angle = new_angle < theta.joint.min_angle ? theta.joint.min_angle : new_angle;
			  //theta.joint.angle = new_angle;
			  //theta.angleToPos(theta.translation().magnitude());
			  i+=3;
		  }
	  }
	  
	  //Calculate 
	  public static double[][] calculateJacobian(ArrayList<Bone> joints, ArrayList<Bone> end_effectors){
	    double[][] jacobian = new double[end_effectors.size()*3][joints.size()*3];
	    //calc 3 rows corresponding to a end effector    
	    int i = 0;
	    for(Bone s_i : end_effectors){
	      //clamp target dist to a reachable pos
	      float max_dist = 0;
	      Bone aux =  s_i;
	      while(aux.parent != null){
	    	  max_dist += Vec.distance(aux.position(), aux.parent.position());
	    	  aux = aux.parent;
	      }
	      Bone r = s_i.getRoot();
	      Vec new_pos = Vec.subtract(s_i.final_ef_pos, r.position());
	      float dist = new_pos.magnitude();
	      if(dist > max_dist){
	    	  new_pos = Vec.subtract(s_i.final_ef_pos, r.position());
		      new_pos.normalize();
		      new_pos.multiply(max_dist);
		      new_pos.add(r.position());
	    	  s_i.final_ef_pos = new_pos;
	      }
	      int j = 0;
	      for(Bone theta_j : joints){
	        //check if the joint could move the end effector
	        if((!s_i.isAncester(theta_j) || theta_j.parent == null) && theta_j != s_i){
	          jacobian[i][j] = 0;
	          jacobian[i+1][j] = 0;
	          jacobian[i+2][j] = 0;
	          jacobian[i][j+1] = 0;
	          jacobian[i+1][j+1] = 0;
	          jacobian[i+2][j+1] = 0;
	          jacobian[i][j+2] = 0;
	          jacobian[i+1][j+2] = 0;
	          jacobian[i+2][j+2] = 0;

	        }else{
	          Vec Pj = theta_j.parent.position();
	          Vec Si = s_i.position();
	          //respect to axis X
	          Vec res = calculateDiff(Pj, Si, new Vec(1,0,0));        
	          jacobian[i][j] = res.x();
	          jacobian[i+1][j] = res.y();
	          jacobian[i+2][j] = res.z();
	          //respect to axis Y
	          res = calculateDiff(Pj, Si, new Vec(0,1,0));        
	          jacobian[i][j+1] = res.x();
	          jacobian[i+1][j+1] = res.y();
	          jacobian[i+2][j+1] = res.z();
	          //respect to axis Z
	          res = calculateDiff(Pj, Si, new Vec(0,0,1));        
	          jacobian[i][j+2] = res.x();
	          jacobian[i+1][j+2] = res.y();
	          jacobian[i+2][j+2] = res.z();

	        }          
	        j+=3;
	      }
	      i+=3;
	    }  
	    return jacobian;
	  } 
	  
	  public static Vec[] calculateError(ArrayList<Bone> s){
	    Vec[] e = new Vec[s.size()];
	    for(int i = 0; i < s.size(); ){
	      Vec ti = s.get(i).final_ef_pos;
	      Vec si = s.get(i).position();
	      e[i++] = Vec.subtract(ti, si);
	    }
	    return e;
	  }
	  
	  //Setting D_max as the same constant for all the end effectors
	  public static void setDMax(ArrayList<Bone> bones){
		  float avg_mag = 0;
		  for(Bone b : bones){
			  if(b.parent != null){
				  Vec dist = Vec.subtract(b.parent.position(), b.position());
				  avg_mag += dist.magnitude();
			  }
		  }
		  d_max = avg_mag/(bones.size()*1.f); 
	  }
	  
	  public static Vec clampMag(Vec w, float d){
		  if(w.magnitude() <= d) return w;
		  Vec v = w.get();
		  v.normalize();
		  v.multiply(d);
		  return v;
	  }
	  
	  //IK using FABRIK method
	  public static void execFABRIK(){
		  
	  }

	  //The chain to consider goes from  
	  public static void execFABRIK(Bone s, int initial){
		  
	  }

	  //Skinning algorithm
	  static int counter = -1;
	  public static void applyTransformations(Utilities.CustomModelFrame model){
	    if(laplacian == null) return;
	    counter++;
	    if(counter == 1){
	    	counter = 0;
	    	//if(debug) return;
	    }
	    
	    for(Skeleton s : Kinematics.skeletons){
	    	Bone root = s.frame.getRoot();
	    	for(Bone bone : root.getChildrenWS()){
	    		//update old values
		    	float roll  = bone.joint_axis_x.angle - bone.prev_angle.x();      
		    	float pitch = bone.joint_axis_y.angle - bone.prev_angle.y();      
		    	float yaw   = bone.joint_axis_z.angle - bone.prev_angle.z();      
		    	Quat q = new Quat(roll, pitch, yaw);
		    	Quat q1 = new Quat(bone.joint_axis_x.angle, bone.joint_axis_y.angle, bone.joint_axis_z.angle);
		    	Quat q2 = new Quat(bone.prev_angle.x(), bone.prev_angle.y(), bone.prev_angle.z());
		    	q = q1;
		    	q.compose(q2.inverse());
		    	Vec mov = Vec.subtract(bone.parent.model_pos, bone.prev_pos);
		        Vec rot = Vec.subtract(bone.model_pos, bone.prev_pos);
		        rot = q.rotate(rot);
				rot.add(bone.prev_pos);
				//apply translation
				rot.add(mov);
				bone.model_pos = rot;
				bone.prev_pos = bone.parent.model_pos.get();
				bone.prev_angle = new Vec(bone.joint_axis_x.angle, bone.joint_axis_y.angle, bone.joint_axis_z.angle );
	    	}
	    }
	    	    
	    for(LaplacianDeformation.Anchor anchor : laplacian.anchors){	    	
	    	PVector final_pos = new PVector(0,0,0);
	    	PShape face = Kinematics.original_fig.getShape().getChild(
	    			anchor.vertex.idx_shape.get(0)[0]);
	    	anchor.pos = face.getVertex(anchor.vertex.idx_shape.get(0)[1]);
	    	
	        if(debug)System.out.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& size ats :" + anchor.attribs.size());	    	
	    	for(LaplacianDeformation.Anchor.AnchorAttribs ats : anchor.attribs){
		    	//get the movement of the bone
		    	Bone bone = ats.related_bone;

		    	float roll  = bone.joint_axis_x.angle - ats.initial_angle.x;      
		    	float pitch = bone.joint_axis_y.angle - ats.initial_angle.y;      
		    	float yaw   = bone.joint_axis_z.angle - ats.initial_angle.z;      
		    	Quat q = new Quat(roll, pitch, yaw);
		    	Quat q1 = new Quat(bone.joint_axis_x.angle, bone.joint_axis_y.angle, bone.joint_axis_z.angle);
		    	Quat q2 = new Quat(ats.initial_angle.x, ats.initial_angle.y, ats.initial_angle.z);
		    	q = q1;
		    	q.compose(q2.inverse());
		        Vec vec = new Vec(anchor.pos.x,anchor.pos.y,anchor.pos.z);
		    	Vec mov = Vec.subtract(bone.parent.model_pos, new Vec(ats.initial_pos.x, ats.initial_pos.y, ats.initial_pos.z));
		        //do all transformations in model space
		    	Vec rot = Vec.subtract(vec, new Vec(ats.initial_pos.x, ats.initial_pos.y, ats.initial_pos.z));
		    	Vec to_rot = rot.get();

		    	System.out.println("sin rot : " + rot);
		    	rot = q.rotate(rot);
		    	System.out.println("con rot : " + rot);
		    	//rot.add(bone.parent.model_pos);		    	
		        Vec new_pos = Vec.add(rot,new Vec(ats.initial_pos.x, ats.initial_pos.y, ats.initial_pos.z));        
		        //apply translation	        
		        new_pos.add(mov);
		        if(debug)System.out.println("rot: "  + rot);
		        if(debug)System.out.println("mov: "  + mov);
		        if(debug)System.out.println("pos : " + vec + " new pos : " + new_pos);	        
		        //apply the weights
		        PVector new_pos_wi = anchor.pos.get();
		        new_pos_wi.mult(ats.weight*0);
		        final_pos.add(new_pos_wi);
		        if(debug)System.out.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& asdfasd sad _  :" + ats.weight);
		        PVector new_pos_w = new PVector(new_pos.x(), new_pos.y(),new_pos.z());
		        new_pos_w.mult(ats.weight);
		        final_pos.add(new_pos_w);
		        
	        	//anchor.weight = 1f;		        	
		        //anchor.pos = new_pos_w;
		        //update old values
				ats.initial_pos = new PVector(bone.parent.model_pos.x() , bone.parent.model_pos.y(), bone.parent.model_pos.z());
				//rot = Vec.subtract(bone.model_pos, bone.parent.model_pos);
				//rot.rotate(rot_angle); rot.add(bone.parent.model_pos);
				//apply translation
				//rot.add(mov);
				//bone.model_pos = rot;
				ats.initial_angle = new PVector(0,0,0);
				ats.initial_angle.x = bone.joint_axis_x.angle;	 
				ats.initial_angle.y = bone.joint_axis_y.angle;	 
				ats.initial_angle.z = bone.joint_axis_z.angle;	 

				if(debug)System.out.println("final_pos " + final_pos);
	    	}
	    	anchor.pos = final_pos;
	    	anchor.pos.mult(1.f/anchor.weight);
	    }
	    //solve the laplacian system
	    //set to true to use Native approach
	    laplacian.getLHS(false);	    
		ArrayList<PVector> new_positions = laplacian.solveLaplacian();
	    //ArrayList<PVector> new_positions = laplacian.solveLaplacianNative();
		for(LaplacianDeformation.Vertex vertex : laplacian.vertices.values()){
			for(int[] idxs : vertex.idx_shape){
				PShape face = Kinematics.original_fig.getShape().getChild(idxs[0]);
				//System.out.println("prev - : " + face.getVertex(idxs[1]));
				face.setVertex(idxs[1], new_positions.get(vertex.idx));
				//System.out.println("next - : " + face.getVertex(idxs[1]));
			}
		}				
	  }
	  
	  //Skinning based on Smooth Skinning algorithm
	  public static void applyTransformationsLinearBlending(Utilities.CustomModelFrame model){
	    if(skinning == null) return;	    
	    for(Skeleton s : Kinematics.skeletons){
	    	Bone root = s.frame.getRoot();
	    	for(Bone bone : root.getChildrenWS()){
	    		//update old values
		    	float roll  = bone.joint_axis_x.angle - bone.prev_angle.x();      
		    	float pitch = bone.joint_axis_y.angle - bone.prev_angle.y();      
		    	float yaw   = bone.joint_axis_z.angle - bone.prev_angle.z();      
		    	Quat q = new Quat(roll, pitch, yaw);
		    	Quat q1 = new Quat(bone.joint_axis_x.angle, bone.joint_axis_y.angle, bone.joint_axis_z.angle);
		    	Quat q2 = new Quat(bone.prev_angle.x(), bone.prev_angle.y(), bone.prev_angle.z());
		    	q = q1;
		    	q.compose(q2.inverse());
		    	Vec mov = Vec.subtract(bone.parent.model_pos, bone.prev_pos);
		        Vec rot = Vec.subtract(bone.model_pos, bone.prev_pos);
		        rot = q.rotate(rot);
				rot.add(bone.prev_pos);
				//apply translation
				rot.add(mov);
				bone.model_pos = rot;
				bone.prev_pos = bone.parent.model_pos.get();
				bone.prev_angle = new Vec(bone.joint_axis_x.angle, bone.joint_axis_y.angle, bone.joint_axis_z.angle );
	    	}
	    }
    	for(Skinning.Vertex v : skinning.vertices.values()){
    		v.applyTransformation(model);
			for(int[] idxs : v.idx_shape){
				PShape face = model.getShape().getChild(idxs[0]);
				//System.out.println("prev - : " + face.getVertex(idxs[1]));
				face.setVertex(idxs[1], v.v);
				//System.out.println("next - : " + face.getVertex(idxs[1]));
			}	    		
    	}
	  }	  
	  
	  
	  //Skinning based on laplacian deformation
	  public static void execSkinning(Utilities.CustomModelFrame model, ArrayList<Bone> bones){
	    laplacian = new LaplacianDeformation();
	    //update laplacian info of each bone
	  	laplacian.setup(model.getShape());
	  	//for each bone get its position and  add an Anchor to nearest vertices
	  	for(int i = 0; i < bones.size(); i++){
	  		bones.get(i).model_pos = model.coordinatesOf(bones.get(i).position().get());
	    	float roll  = bones.get(i).joint_axis_x.angle;      
	    	float pitch = bones.get(i).joint_axis_y.angle;      
	    	float yaw   = bones.get(i).joint_axis_z.angle;      	  		
	  		bones.get(i).prev_angle = new Vec(roll, pitch, yaw);
	  		if(bones.get(i).parent == null) continue;
	  		bones.get(i).prev_pos = model.coordinatesOf(bones.get(i).parent.position().get());
	  		laplacian.addAnchorByDist(laplacian.anchors, model, bones.get(i), i, 0.8f);
	  	}
	  	laplacian.calculateLaplacian();
	  }  
	  
	  public static void execSkinningLinearBlending(Utilities.CustomModelFrame model, ArrayList<Bone> bones){
		    skinning = new Skinning();
		  	//for each bone get its position and  add an Anchor to nearest vertices
		  	for(int i = 0; i < bones.size(); i++){
		  		bones.get(i).model_pos = model.coordinatesOf(bones.get(i).position().get());
		    	float roll  = bones.get(i).joint_axis_x.angle;      
		    	float pitch = bones.get(i).joint_axis_y.angle;      
		    	float yaw   = bones.get(i).joint_axis_z.angle;      	  		
		  		bones.get(i).prev_angle = new Vec(roll, pitch, yaw);
		  		if(bones.get(i).parent == null) continue;
		  		bones.get(i).prev_pos = model.coordinatesOf(bones.get(i).parent.position().get());
		  	}
		  	skinning.setup(model, bones);
	  }
	  
	//}
	  public static void drawWeights(Scene sc, Utilities.CustomModelFrame model){
		  if(skinning == null) return;
		  for(Skinning.Vertex vertex : skinning.vertices.values()){
			  PShape f = model.getShape().getChild(vertex.idx_shape.get(0)[0]);
		      PVector v = f.getVertex(vertex.idx_shape.get(0)[1]);
		      Vec v_w = model.inverseCoordinatesOf(new Vec(v.x, v.y, v.z));
		      Skinning.Vertex.VertexAttribs  atss = null;
		      for(Skinning.Vertex.VertexAttribs ats : vertex.attribs){
		    	  if(atss == null) atss = ats;
		    	  else if(atss.weight < ats.weight){
		    		  atss = ats;
		    	  }
		      }
			  if(atss.weight < 0.5) continue;
		      sc.pg().pushStyle();
			  sc.pg().strokeWeight(5);
			  sc.pg().stroke(atss.bone.colour);
			  sc.pg().point(v_w.x(), v_w.y(), v_w.z());
			  sc.pg().popStyle();

		  }
	  }

	  
	  public static void drawAnchors(Scene sc, Utilities.CustomModelFrame model){
		  if(laplacian == null) return;
		  for(LaplacianDeformation.Anchor anchor : laplacian.anchors){
			  LaplacianDeformation.Vertex vertex = anchor.vertex;
			  PShape f = model.getShape().getChild(vertex.idx_shape.get(0)[0]);
		      PVector v = f.getVertex(vertex.idx_shape.get(0)[1]);
		      Vec v_w = model.inverseCoordinatesOf(new Vec(v.x, v.y, v.z));
			  for(LaplacianDeformation.Anchor.AnchorAttribs ats : anchor.attribs){
				  sc.pg().pushStyle();
				  sc.pg().strokeWeight(5);
				  sc.pg().stroke(ats.related_bone.colour);
				  sc.pg().point(v_w.x(), v_w.y(), v_w.z());
				  sc.pg().popStyle();
			  }
		  }
	  }
}
