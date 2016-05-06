import java.util.*;
import java.util.Map.Entry;

import processing.core.*;
import remixlab.dandelion.geom.Quat;
import remixlab.dandelion.geom.Vec;

public class Skinning {
	public HashMap<PVector,Vertex> vertices;
	
	public class Vertex{
		PVector base;
		PVector v; 
		ArrayList<int[]> idx_shape; 
		PShape shape;
		ArrayList<VertexAttribs> attribs;
		
		
		public class VertexAttribs{
			Bone bone;
			double weight;
			PVector initial_position;
			PVector initial_angle;			
			
			public VertexAttribs(Bone b, double w, PVector ini_pos){
				bone = b;
				weight= w;
				initial_position = ini_pos.get();
				initial_angle = new PVector(0,0,0); 
				initial_angle.x = b.joint_axis_x.angle;
				initial_angle.y = b.joint_axis_y.angle;
				initial_angle.z = b.joint_axis_z.angle;
			}			
		}		
				
		public Vertex(PVector b, int is, int ic){
			base = b.get();
			v = base.get();
			idx_shape = new ArrayList<int[]>();
			int[] idxs = new int[]{ic,is}; 
			idx_shape.add(idxs);
			attribs = new ArrayList<VertexAttribs>();
		}		
		
		public void addIdxs(int is, int ic){
			int[] idxs = new int[]{ic,is}; 
			idx_shape.add(idxs);
		}
		
		public void applyTransformation(Utilities.CustomModelFrame model){	    
			PVector final_pos = new PVector(0,0,0);
	    	PShape face = model.getShape().getChild(idx_shape.get(0)[0]);
	    	this.v = face.getVertex(idx_shape.get(0)[1]);
	    	Vertex.VertexAttribs atss = null;
	    	for(Vertex.VertexAttribs ats : this.attribs){
	    		if(atss == null) atss = ats;
	    		else if(atss.weight < ats.weight)atss = ats;
	    	}
	    	
	    	System.out.println("---weight : " + atss.weight);
	    	
			for(Vertex.VertexAttribs ats : this.attribs){
				//if(ats != atss) continue;
			    Bone bone = ats.bone;
		    	float roll  = bone.joint_axis_x.angle - ats.initial_angle.x;      
		    	float pitch = bone.joint_axis_y.angle - ats.initial_angle.y;      
		    	float yaw   = bone.joint_axis_z.angle - ats.initial_angle.z;      
		    	Quat q = new Quat(roll, pitch, yaw);
		    	Quat q1 = new Quat(bone.joint_axis_x.angle, bone.joint_axis_y.angle, bone.joint_axis_z.angle);
		    	Quat q2 = new Quat(ats.initial_angle.x, ats.initial_angle.y, ats.initial_angle.z);
		    	q = q1;
		    	q.compose(q2.inverse());
		        Vec vec = new Vec(v.x,v.y,v.z);
		    	Vec mov = Vec.subtract(bone.parent.model_pos, new Vec(ats.initial_position.x, 
		    			ats.initial_position.y, ats.initial_position.z));
			    
		        //do all transformations in model space
		    	Vec rot = Vec.subtract(vec, new Vec(ats.initial_position.x, ats.initial_position.y, 
		    			ats.initial_position.z));
		    	Vec to_rot = rot.get();
		    	rot = q.rotate(rot);
		    	//rot.add(bone.parent.model_pos);		    	
		    	Vec new_pos = Vec.add(rot,new Vec(ats.initial_position.x, ats.initial_position.y,
		        		ats.initial_position.z));        
		        //apply translation	        
		        new_pos.add(mov);
		        
		        //apply the weights
		        PVector new_pos_w = new PVector(new_pos.x(), new_pos.y(),new_pos.z());
			    new_pos_w.mult((float)ats.weight);
		        final_pos.add(new_pos_w);

				ats.initial_position = new PVector(bone.parent.model_pos.x() , bone.parent.model_pos.y(), 
						bone.parent.model_pos.z());
				ats.initial_angle = new PVector(0,0,0);
				ats.initial_angle.x = bone.joint_axis_x.angle;	 
				ats.initial_angle.y = bone.joint_axis_y.angle;	 
				ats.initial_angle.z = bone.joint_axis_z.angle;	 
				System.out.println("cambiaaa: " + v + "\n" + final_pos);
			}
			if(PVector.dist(final_pos, v) > 0 ){
				System.out.println("cambiaaa: " + v + "\n" + final_pos);
			} 
			this.v = final_pos;
		}
		
		public void addAttrib(Bone b, float dist, Vec ini_pos ){
			attribs.add(new Vertex.VertexAttribs(b,
					dist, new PVector (ini_pos.x(), ini_pos.y(), ini_pos.z())));
		}
		
	}
	
	public void setup(Utilities.CustomModelFrame model, ArrayList<Bone> bones){
		PShape shape = model.getShape();
		vertices = new HashMap<PVector,Vertex>();
		for(int i = 0; i < shape.getChildCount(); i++){
			PShape child = shape.getChild(i);
			for(int j = 0; j < child.getVertexCount(); j++){
				PVector v = child.getVertex(j);
				if(!vertices.containsKey(v)){
					Vertex vertex = new Vertex(v, j, i);
					Vec vec = new Vec(v.x,v.y,v.z);					
					double total_dist = 0.;
					double max_dist = -999;
					for(Bone b : bones){
				  		if(b.parent == null) continue;
				  		double dist = Utilities.getDistance(vec, b)[1];
				  		max_dist = dist > max_dist ? dist : max_dist;
					}

					for(Bone b : bones){
				  		if(b.parent == null) continue;
						double dist = Utilities.getDistance(vec, b)[1];
						dist = 1/Math.pow(dist, 10);
						total_dist += dist;
						Vec ini_pos = model.coordinatesOf(b.parent.position().get());				
						vertex.addAttrib(b,	(float) dist, ini_pos);
					}
					//The more near, the more weight the bone applies
					float sum = 0;
					for(Vertex.VertexAttribs ats : vertex.attribs){
						ats.weight = ats.weight/(total_dist*1.); 
						sum += ats.weight;
						System.out.print("--->bone num: " + ats.bone.idx);
						System.out.println("--->pos : "+ model.inverseCoordinatesOf(new Vec(vertex.v.x,
								vertex.v.y,vertex.v.z)));
						System.out.println("--->dist: " + Utilities.getDistance(vec, ats.bone)[1]);
						System.out.println("--->w: " + ats.weight);
					}
					System.out.println("<--->w_sum: " + sum);
					vertices.put(v, vertex);
				}else{
					Vertex vertex = vertices.get(v);
					//add the idx of the face and the num
					vertex.addIdxs(j,i);									
				}

			}
		}
	}
}
