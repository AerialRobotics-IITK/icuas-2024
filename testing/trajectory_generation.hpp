
void create_graph(double x, double y,double z){
	struct node* temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 0; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 0; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 0; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 0; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 0; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 0; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 0; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 0; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 0; temp_node->z = 2*z;
	nodes.push_back(temp_node);




	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = y; temp_node->z = 2*z;
	nodes.push_back(temp_node);




	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 2*y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 2*y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 2*y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 2*y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 2*y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 2*y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

}


int find_distance(int point1, int point2){
	struct node node1;
	struct node node2;

	if(point1 ==0){
		node1.x = -1;
		node1.y = -1;
		node1.z = 0;
	}else{
		node1 = *nodes[point1-1];
	}	
	node2 = *nodes[point2-1];

	return (node1.x - node2.x)*(node1.x - node2.x) + (node1.y - node2.y)*(node1.y - node2.y) + (node1.z - node2.z)*(node1.z - node2.z);
}



void get_all_points(int* possible,int count,vector<int> &store){
	if(count == 9){
		vector<int> hello;
		for(int i =0;i<store.size();i++){
			if(store[i]!=0){
				hello.push_back(store[i]);
			}
		}
		all.push_back(hello);
		return;
	}
	store[count] = 0;
	get_all_points(possible,count+1,store);	
	store[count] = possible[count];
	get_all_points(possible,count+1,store);
}



int check_path(vector<int> path,int start_location){
	double sum=0;
	for(int i=0;i<path.size();i++){
		double val;
		if(i==0){
			val = find_distance(start_location,path[i]);
		}else{
			val = find_distance(path[i],path[i-1]);
		}sum = sum+val;
	}
	return sum;
}




void permutations(vector<int> nums, int l, int h,int start_location) 
{ 	
    if (l == h){ 
		int tmp =check_path(nums,start_location);
		if(tmp<min_imum){
			minimum_vector = nums;
			min_imum = tmp;
		}		
        return; 
    } 

    for (int i = l; i <= h; i++) { 
        swap(nums[l], nums[i]);   
        permutations(nums, l + 1, h,start_location); 
        swap(nums[l], nums[i]); 
    } 
} 
  

void permute(vector<int>& nums,int start_location) 
{ 
    permutations(nums, 0,nums.size()-1,start_location); 
} 


vector<int> find_shortest_permutation(vector<int> points, int start_location){
		vector<int> min_vec;
		if(points.size()>2){
			permute(points,start_location);
			min_vec = minimum_vector;
		}
	return min_vec;	
}

int find_next_plane_point(vector<int> plane0,vector<int> plane1){
	int min_dist=INF;
	int next;
	for(int i=0;i<plane1.size();i++){
		int tmp = find_distance(plane1[i],plane0[0]);
		if(min_dist>tmp){
			min_dist = find_distance(plane1[i],plane0[0]);
			next = plane1[i];
		}
	}
	return next;
}