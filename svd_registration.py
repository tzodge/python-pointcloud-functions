
import numpy as np
def register_clouds(P, Q):
	'''
	aligns pointclouds P and Q given one to one correspondence.
	i.e. i^th point in P corresponds to i^th point in Q
	
	input: 
		P = nx3 (Source)
		Q = nx3 (Target)
	output: 
		rotation matrix R = 3x3 
		translation vector t = 1x3

	Such that
	SUM over  i ( | P_i*R + t - Q_i |^2 )
	is minimised 
	
	'''
	assert P.shape == Q.shape
	n, dim = P.shape

	centeredP = P - P.mean(axis=0)
	centeredQ = Q - Q.mean(axis=0)

	C = np.dot(np.transpose(centeredP), centeredQ) / n

	U, S, Vt = np.linalg.svd(C)

	if (np.linalg.det(U) * np.linalg.det(Vt)) < 0.0:
		S[-1] = -S[-1]
		U[:, -1] = -U[:, -1]

	R = np.dot(U, Vt).T

	t = Q.mean(axis=0) - P.mean(axis=0).dot(R.T)
 
	return  R, t

def random_rotation_matrix():
	import transforms3d as t3d

	axis = np.random.rand(3,) - np.array([0.5,0.5,0.5])
	axis = axis/np.linalg.norm(axis)
	rotation_angle = 2*(np.random.uniform()-0.5) * np.pi  
	rotation_matrix = t3d.axangles.axangle2mat(axis, rotation_angle)

	return rotation_matrix

def main():
	n=25
	# create random pointcloud
	pointcloud1 = np.random.rand(n,3) - np.array([0.5,0.5,0.5])

	# generate a valid rotation
	rotation_matrix  = random_rotation_matrix()     
	
	translation = np.random.rand(3,) #random translation

	pointcloud2 = rotation_matrix.dot(pointcloud1.T).T + translation

	R_pred, t_pred   = register_clouds(pointcloud1,pointcloud2) 


	print(R_pred,"pred rotation \n") 
	print(rotation_matrix,"gt rotation \n")

	print(t_pred,"pred translation\n") 
	print(translation,"gt translation \n") 

if __name__ == '__main__':
	main()
 
